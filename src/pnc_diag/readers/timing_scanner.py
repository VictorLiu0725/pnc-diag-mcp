"""Chunk-bounded zero-deserialization timing & health scanner for MCAP, SQLite3, and ROS 1."""

from __future__ import annotations

import heapq
import math
import random
import sqlite3
from collections.abc import Callable
from pathlib import Path

import numpy as np
from mcap.data_stream import ReadDataStream
from mcap.opcode import Opcode
from mcap.reader import NonSeekingReader, make_reader
from mcap.records import MessageIndex
from rosbags.rosbag1 import Reader as Ros1Reader

from pnc_diag.models.signals import GapInterval, TopicTimingSummary
from pnc_diag.readers.discovery import discover_bag


class _TopicTimingAccumulator:
    """Single-pass, bounded-memory collector preserving stored order for regression checks."""

    def __init__(
        self,
        topic: str,
        message_type: str,
        *,
        reservoir_size: int = 8192,
        max_gap_events: int = 64,
    ) -> None:
        self.topic = topic
        self.message_type = message_type
        self.reservoir_size = reservoir_size
        self.max_gap_events = max_gap_events

        self.message_count = 0
        self.start_ns: int | None = None
        self.end_ns: int | None = None
        self.monotonic_violations = 0
        self.duplicate_timestamps = 0
        self.zero_timestamps = 0

        self._last_ns: int | None = None
        self._positive_count = 0
        self._delta_reservoir: list[float] = []
        self._max_gap_ms: float | None = None
        self._top_gaps_heap: list[tuple[float, int, int]] = []
        self._rng = random.Random(2026)

    def add(self, ts_ns: int) -> None:
        self.message_count += 1
        if ts_ns == 0:
            self.zero_timestamps += 1
        if self.start_ns is None or ts_ns < self.start_ns:
            self.start_ns = ts_ns
        if self.end_ns is None or ts_ns > self.end_ns:
            self.end_ns = ts_ns

        if self._last_ns is not None:
            delta = ts_ns - self._last_ns
            if delta < 0:
                self.monotonic_violations += 1
            elif delta == 0:
                self.duplicate_timestamps += 1
            else:
                delta_ms = delta / 1e6
                self._positive_count += 1
                if self._max_gap_ms is None or delta_ms > self._max_gap_ms:
                    self._max_gap_ms = delta_ms

                # Algorithm R bounded reservoir sample for O(1)-memory quantiles.
                if len(self._delta_reservoir) < self.reservoir_size:
                    self._delta_reservoir.append(delta_ms)
                else:
                    idx = self._rng.randrange(self._positive_count)
                    if idx < self.reservoir_size:
                        self._delta_reservoir[idx] = delta_ms

                # Min-heap tracks exact Top-K largest gaps without retaining all timestamps.
                item = (delta_ms, self._last_ns, ts_ns)
                if len(self._top_gaps_heap) < self.max_gap_events:
                    heapq.heappush(self._top_gaps_heap, item)
                elif delta_ms > self._top_gaps_heap[0][0]:
                    heapq.heapreplace(self._top_gaps_heap, item)

        self._last_ns = ts_ns

    def finalize(self) -> TopicTimingSummary:
        count = self.message_count
        if count == 0 or self.start_ns is None or self.end_ns is None:
            return TopicTimingSummary(
                topic=self.topic,
                message_type=self.message_type,
                message_count=0,
                start_ns=None,
                end_ns=None,
                duration_s=0.0,
                mean_rate_hz=None,
                median_period_ms=None,
                p95_period_ms=None,
                p99_period_ms=None,
                p95_jitter_ms=None,
                max_gap_ms=None,
                monotonic_violations=self.monotonic_violations,
                duplicate_timestamps=self.duplicate_timestamps,
                zero_timestamps=self.zero_timestamps,
            )

        duration_s = max(0.0, (self.end_ns - self.start_ns) / 1e9)
        mean_rate_hz = ((count - 1) / duration_s) if (count > 1 and duration_s > 0.0) else None

        median_ms: float | None = None
        p95_ms: float | None = None
        p99_ms: float | None = None
        p95_jitter_ms: float | None = None
        max_gap_ms = self._max_gap_ms
        heuristic_large_gap = False
        gap_intervals: tuple[GapInterval, ...] = ()

        if self._delta_reservoir:
            arr = np.asarray(self._delta_reservoir, dtype=np.float64)
            median_ms = float(np.median(arr))
            p95_ms = float(np.percentile(arr, 95))
            p99_ms = float(np.percentile(arr, 99))
            p95_jitter_ms = float(np.percentile(np.abs(arr - median_ms), 95))
            if (
                count >= 20
                and max_gap_ms is not None
                and p95_ms <= median_ms * 1.5
                and max_gap_ms > max(100.0, median_ms * 5.0)
            ):
                heuristic_large_gap = True

            gap_thresh = max(100.0, median_ms * 3.0)
            detected = [
                GapInterval(start_ns=s, end_ns=e, gap_ms=round(g, 4))
                for g, s, e in self._top_gaps_heap
                if g > gap_thresh
            ]
            detected.sort(key=lambda item: item.start_ns)
            gap_intervals = tuple(detected)

        return TopicTimingSummary(
            topic=self.topic,
            message_type=self.message_type,
            message_count=count,
            start_ns=self.start_ns,
            end_ns=self.end_ns,
            duration_s=round(duration_s, 6),
            mean_rate_hz=round(mean_rate_hz, 4) if (mean_rate_hz and math.isfinite(mean_rate_hz)) else None,
            median_period_ms=round(median_ms, 4) if median_ms is not None else None,
            p95_period_ms=round(p95_ms, 4) if p95_ms is not None else None,
            p99_period_ms=round(p99_ms, 4) if p99_ms is not None else None,
            p95_jitter_ms=round(p95_jitter_ms, 4) if p95_jitter_ms is not None else None,
            max_gap_ms=round(max_gap_ms, 4) if max_gap_ms is not None else None,
            monotonic_violations=self.monotonic_violations,
            duplicate_timestamps=self.duplicate_timestamps,
            zero_timestamps=self.zero_timestamps,
            heuristic_large_gap=heuristic_large_gap,
            gap_intervals=gap_intervals,
        )


def _scan_sqlite_file(
    file_path: Path,
    accumulators: dict[str, _TopicTimingAccumulator],
    topic_filter: set[str] | None,
    factory: Callable[[str, str], _TopicTimingAccumulator],
) -> None:
    uri = f"{file_path.resolve().as_uri()}?mode=ro"
    conn = sqlite3.connect(uri, uri=True)
    try:
        conn.execute("PRAGMA query_only = ON")
        topic_by_id: dict[int, tuple[str, str]] = {}
        for tid, name, msg_type in conn.execute("SELECT id, name, type FROM topics ORDER BY id"):
            topic_by_id[tid] = (name, msg_type or "unknown")
            if (topic_filter is None or name in topic_filter) and name not in accumulators:
                accumulators[name] = factory(name, msg_type or "unknown")

        cursor = conn.execute("SELECT topic_id, timestamp FROM messages ORDER BY id")
        while rows := cursor.fetchmany(100_000):
            for tid, ts_ns in rows:
                info = topic_by_id.get(tid)
                if info is None:
                    continue
                name, msg_type = info
                if topic_filter is not None and name not in topic_filter:
                    continue
                if name not in accumulators:
                    accumulators[name] = factory(name, msg_type)
                accumulators[name].add(int(ts_ns))
    finally:
        conn.close()


def _scan_mcap_file(
    file_path: Path,
    accumulators: dict[str, _TopicTimingAccumulator],
    topic_filter: set[str] | None,
    factory: Callable[[str, str], _TopicTimingAccumulator],
) -> None:
    with file_path.open("rb") as f:
        summary = make_reader(f, validate_crcs=False).get_summary()
        if summary is not None:
            for ch in summary.channels.values():
                if topic_filter is not None and ch.topic not in topic_filter:
                    continue
                schema = summary.schemas.get(ch.schema_id)
                schema_name = schema.name if schema else "unknown"
                if ch.topic not in accumulators:
                    accumulators[ch.topic] = factory(ch.topic, schema_name)

            # Fast O(1)-RAM index-only path: read MessageIndex offsets without decompressing chunks.
            chunk_indexes = sorted(summary.chunk_indexes, key=lambda c: c.chunk_start_offset)
            if chunk_indexes and all(
                c.message_index_length > 0 and c.message_index_offsets for c in chunk_indexes
            ):
                for chunk_idx in chunk_indexes:
                    per_topic_records: dict[str, list[list[tuple[int, int]]]] = {}
                    for cid, offset in chunk_idx.message_index_offsets.items():
                        ch = summary.channels.get(cid)
                        if ch is None or (topic_filter is not None and ch.topic not in topic_filter):
                            continue
                        f.seek(offset)
                        ds = ReadDataStream(f)
                        opcode = ds.read1()
                        _ = ds.read8()
                        if opcode != Opcode.MESSAGE_INDEX:
                            break
                        mi = MessageIndex.read(ds)
                        records = mi.records
                        if not records:
                            continue
                        # Recover exact physical write order inside chunk via uncompressed offset.
                        if len(records) > 1 and any(
                            records[i][1] > records[i + 1][1] for i in range(len(records) - 1)
                        ):
                            records = sorted(records, key=lambda item: item[1])
                        per_topic_records.setdefault(ch.topic, []).append(records)

                    for topic_name, rec_lists in per_topic_records.items():
                        acc = accumulators[topic_name]
                        if len(rec_lists) == 1:
                            for ts_ns, _ in rec_lists[0]:
                                acc.add(int(ts_ns))
                        else:
                            for ts_ns, _ in heapq.merge(*rec_lists, key=lambda item: item[1]):
                                acc.add(int(ts_ns))
                return

    # Fallback streaming path for unindexed/unchunked MCAP files (bounded memory, preserves order).
    with file_path.open("rb") as f:
        reader = NonSeekingReader(f, validate_crcs=False)
        for schema, channel, message in reader.iter_messages(
            topics=list(topic_filter) if topic_filter is not None else None,
            log_time_order=False,
        ):
            if channel.topic not in accumulators:
                schema_name = schema.name if schema else "unknown"
                accumulators[channel.topic] = factory(channel.topic, schema_name)
            accumulators[channel.topic].add(int(message.log_time))


def _scan_ros1_file(
    file_path: Path,
    accumulators: dict[str, _TopicTimingAccumulator],
    topic_filter: set[str] | None,
    factory: Callable[[str, str], _TopicTimingAccumulator],
) -> None:
    with Ros1Reader(file_path) as reader:
        conns = [
            c for c in reader.connections if topic_filter is None or c.topic in topic_filter
        ]
        for conn in conns:
            if conn.topic not in accumulators:
                accumulators[conn.topic] = factory(conn.topic, conn.msgtype)
        for conn, ts_ns, _ in reader.messages(connections=conns):
            accumulators[conn.topic].add(int(ts_ns))


def scan_bag_timing(
    bag_path: str | Path,
    topics: set[str] | list[str] | None = None,
    *,
    reservoir_size: int = 8192,
    max_gap_events: int = 64,
) -> list[TopicTimingSummary]:
    """Perform a zero-deserialization, bounded-memory timing scan over a recording.

    Preserves stored message order before checking monotonic regressions, and uses fixed-size
    reservoir sampling + a Top-K gap heap. Memory is bounded by O(max_chunk_messages + reservoir),
    remaining independent of total bag duration/shards for standard MCAP chunk sizes.
    """
    desc = discover_bag(bag_path)
    topic_filter = set(topics) if topics is not None else None
    accumulators: dict[str, _TopicTimingAccumulator] = {}

    def _make_acc(topic_name: str, msg_type: str) -> _TopicTimingAccumulator:
        return _TopicTimingAccumulator(
            topic_name,
            msg_type,
            reservoir_size=reservoir_size,
            max_gap_events=max_gap_events,
        )

    for file_path in desc.files:
        if desc.storage_format == "sqlite3":
            _scan_sqlite_file(file_path, accumulators, topic_filter, _make_acc)
        elif desc.storage_format == "mcap":
            _scan_mcap_file(file_path, accumulators, topic_filter, _make_acc)
        elif desc.storage_format == "rosbag1":
            _scan_ros1_file(file_path, accumulators, topic_filter, _make_acc)

    return [accumulators[name].finalize() for name in sorted(accumulators)]
