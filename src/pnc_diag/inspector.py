"""Open-source thin recording inspector and bounded standardized signal sampler."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import asdict, dataclass
from importlib import resources
from pathlib import Path
from statistics import median
from typing import Any

from pnc_diag.models.signals import PathSample, StorageFormat
from pnc_diag.readers import (
    SignalMapping,
    TopicMapConfig,
    discover_bag,
    iter_bag_messages,
    load_topic_map,
    map_message,
    scan_bag_timing,
)

_BUILTIN_TOPIC_MAP_FILES = {
    "nav2": "topic_map.nav2.yaml",
    "nav2_default": "topic_map.nav2.yaml",
    "autoware": "topic_map.autoware.yaml",
    "autoware_default": "topic_map.autoware.yaml",
}


def resolve_builtin_topic_map_path(preset: str = "nav2") -> Path:
    """Resolve a built-in topic_map YAML path across repo checkouts and wheel installs."""
    filename = _BUILTIN_TOPIC_MAP_FILES.get(preset)
    if filename is None:
        raise FileNotFoundError(f"Unknown built-in topic_map preset: {preset!r}")

    repo_candidate = Path(__file__).resolve().parents[2] / "configs" / filename
    if repo_candidate.is_file():
        return repo_candidate

    bundled_candidate = Path(__file__).resolve().parent / "_configs" / filename
    if bundled_candidate.is_file():
        return bundled_candidate

    res = resources.files("pnc_diag").joinpath("_configs", filename)
    if res.is_file():
        return Path(str(res))

    raise FileNotFoundError(f"Bundled topic_map config not found: {filename}")


DEFAULT_TOPIC_MAP_PATH = resolve_builtin_topic_map_path("nav2")


@dataclass(frozen=True, slots=True)
class TopicTimingAudit:
    """Zero-deserialization timing health paired with bounded header inspection."""

    topic: str
    role: str | None
    message_type: str
    message_count: int
    duration_s: float
    mean_rate_hz: float | None
    median_period_ms: float | None
    p95_period_ms: float | None
    p95_jitter_ms: float | None
    max_gap_ms: float | None
    monotonic_violations: int
    duplicate_timestamps: int
    zero_timestamps: int
    heuristic_large_gap: bool
    gap_count: int
    sample_header_present: bool | None
    sample_zero_header_count: int
    observed_header_minus_log_ms: float | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class BagSummaryReport:
    """Standardized offline recording summary exposed by thin CLI and MCP interfaces."""

    bag_path: str
    storage_format: StorageFormat
    file_count: int
    duration_s: float
    total_messages: int
    available_roles: dict[str, str]
    missing_roles: tuple[str, ...]
    unmapped_topics: tuple[str, ...]
    clock_comparability: str
    warnings: tuple[str, ...]
    topics: tuple[TopicTimingAudit, ...]

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["missing_roles"] = list(self.missing_roles)
        payload["unmapped_topics"] = list(self.unmapped_topics)
        payload["warnings"] = list(self.warnings)
        return payload


def _resolve_topic_map(map_config: TopicMapConfig | str | Path | None) -> TopicMapConfig:
    if isinstance(map_config, TopicMapConfig):
        return map_config
    if map_config is None:
        return load_topic_map(resolve_builtin_topic_map_path("nav2"))
    if (
        isinstance(map_config, str)
        and map_config in _BUILTIN_TOPIC_MAP_FILES
        and not Path(map_config).exists()
    ):
        return load_topic_map(resolve_builtin_topic_map_path(map_config))
    return load_topic_map(Path(map_config))


def _sample_header_stats(
    bag_path: str | Path,
    mapped_topics: set[str],
    bag_start_ns: int | None,
    bag_end_ns: int | None,
    *,
    sample_per_topic: int = 20,
) -> dict[str, tuple[bool, int, float | None]]:
    if not mapped_topics or bag_start_ns is None:
        return {}
    stop_ns = (
        min(bag_end_ns, bag_start_ns + 2_500_000_000)
        if bag_end_ns is not None
        else bag_start_ns + 2_500_000_000
    )
    has_header: dict[str, bool] = {}
    zero_headers: dict[str, int] = defaultdict(int)
    diffs_ms: dict[str, list[float]] = defaultdict(list)
    seen: dict[str, int] = defaultdict(int)

    for raw in iter_bag_messages(
        bag_path,
        topics=sorted(mapped_topics),
        start_ns=bag_start_ns,
        stop_ns=stop_ns,
    ):
        count = seen[raw.topic]
        if count >= sample_per_topic:
            if len(seen) == len(mapped_topics) and all(
                v >= sample_per_topic for v in seen.values()
            ):
                break
            continue
        seen[raw.topic] = count + 1
        if raw.header_stamp_ns is None:
            has_header.setdefault(raw.topic, False)
            continue
        has_header[raw.topic] = True
        if raw.header_stamp_ns <= 0:
            zero_headers[raw.topic] += 1
        else:
            diffs_ms[raw.topic].append((raw.header_stamp_ns - raw.log_time_ns) / 1e6)

    out: dict[str, tuple[bool, int, float | None]] = {}
    for topic in seen:
        obs = round(median(diffs_ms[topic]), 3) if diffs_ms.get(topic) else None
        out[topic] = (has_header.get(topic, False), zero_headers.get(topic, 0), obs)
    return out


def inspect_bag_summary(
    bag_path: str | Path,
    map_config: TopicMapConfig | str | Path | None = None,
    *,
    sample_per_topic: int = 20,
) -> BagSummaryReport:
    """Run zero-deserialization bag timing scan paired with role mapping and header check."""
    descriptor = discover_bag(bag_path)
    cfg = _resolve_topic_map(map_config)
    timings = scan_bag_timing(descriptor.root_path)

    timing_by_topic = {item.topic: item for item in timings}
    available_roles: dict[str, str] = {}
    missing_roles: list[str] = []
    topic_roles: dict[str, list[str]] = defaultdict(list)

    for role, sig in cfg.signals.items():
        if sig.topic in timing_by_topic:
            available_roles[role] = sig.topic
            topic_roles[sig.topic].append(role)
        else:
            missing_roles.append(role)

    unmapped_topics = tuple(sorted(t for t in timing_by_topic if t not in topic_roles))
    starts = [item.start_ns for item in timings if item.start_ns is not None]
    ends = [item.end_ns for item in timings if item.end_ns is not None]
    bag_start_ns = min(starts) if starts else None
    bag_end_ns = max(ends) if ends else None
    duration_s = (
        round(max(0.0, (bag_end_ns - bag_start_ns) / 1e9), 4)
        if (bag_start_ns is not None and bag_end_ns is not None)
        else 0.0
    )

    mapped_present = set(topic_roles.keys())
    hdr_stats = _sample_header_stats(
        descriptor.root_path,
        mapped_present,
        bag_start_ns,
        bag_end_ns,
        sample_per_topic=sample_per_topic,
    )

    warnings: list[str] = []
    audits: list[TopicTimingAudit] = []
    for item in timings:
        roles_str = ",".join(topic_roles[item.topic]) if item.topic in topic_roles else None
        hdr_info = hdr_stats.get(item.topic)
        zero_hdr = hdr_info[1] if hdr_info is not None else 0
        if item.zero_timestamps > 0:
            warnings.append(f"zero_log_timestamp:{item.topic} ({item.zero_timestamps})")
        if item.monotonic_violations > 0:
            warnings.append(f"monotonic_violation:{item.topic} ({item.monotonic_violations})")
        if item.heuristic_large_gap:
            warnings.append(f"large_gap_detected:{item.topic} (max={item.max_gap_ms}ms)")
        if zero_hdr > 0:
            warnings.append(f"zero_header_stamp_in_sample:{item.topic} ({zero_hdr})")

        audits.append(
            TopicTimingAudit(
                topic=item.topic,
                role=roles_str,
                message_type=item.message_type,
                message_count=item.message_count,
                duration_s=item.duration_s,
                mean_rate_hz=item.mean_rate_hz,
                median_period_ms=item.median_period_ms,
                p95_period_ms=item.p95_period_ms,
                p95_jitter_ms=item.p95_jitter_ms,
                max_gap_ms=item.max_gap_ms,
                monotonic_violations=item.monotonic_violations,
                duplicate_timestamps=item.duplicate_timestamps,
                zero_timestamps=item.zero_timestamps,
                heuristic_large_gap=item.heuristic_large_gap,
                gap_count=len(item.gap_intervals),
                sample_header_present=hdr_info[0] if hdr_info is not None else None,
                sample_zero_header_count=zero_hdr,
                observed_header_minus_log_ms=hdr_info[2] if hdr_info is not None else None,
            )
        )

    return BagSummaryReport(
        bag_path=str(descriptor.root_path),
        storage_format=descriptor.storage_format,
        file_count=len(descriptor.files),
        duration_s=duration_s,
        total_messages=sum(item.message_count for item in timings),
        available_roles=available_roles,
        missing_roles=tuple(missing_roles),
        unmapped_topics=unmapped_topics,
        clock_comparability="unverified (open-source thin edition does not assume cross-topic sync)",
        warnings=tuple(warnings),
        topics=tuple(audits),
    )


def _sample_to_dict(sample: Any) -> dict[str, Any]:
    if isinstance(sample, PathSample):
        return {
            "sample_type": "PathSample",
            "stamp_ns": sample.stamp_ns,
            "log_time_ns": sample.log_time_ns,
            "frame_id": sample.frame_id,
            "num_points": len(sample.points),
            "head_points": [asdict(pt) for pt in sample.points[:3]],
            "time_domain": sample.time_domain,
            "stamp_is_valid": sample.stamp_is_valid,
            "alignment_uncertainty_ns": sample.alignment_uncertainty_ns,
        }
    data = asdict(sample)
    data["sample_type"] = type(sample).__name__
    return data


def _select_mappings(
    cfg: TopicMapConfig,
    *,
    topic: str | None,
    role: str | None,
) -> list[SignalMapping]:
    if role is not None:
        if role not in cfg.signals:
            raise ValueError(f"Unknown role {role!r}; available={sorted(cfg.signals.keys())}")
        mapping = cfg.signals[role]
        if topic is not None and mapping.topic != topic:
            raise ValueError(f"Role {role!r} maps to {mapping.topic!r}, not {topic!r}")
        return [mapping]

    if topic is not None:
        matched = cfg.roles_for_topic(topic)
        if not matched:
            raise ValueError(f"Topic {topic!r} is not configured in topic map {cfg.name!r}")
        return matched

    return list(cfg.signals.values())


def sample_mapped_signals(
    bag_path: str | Path,
    *,
    topic: str | None = None,
    role: str | None = None,
    map_config: TopicMapConfig | str | Path | None = None,
    limit: int = 5,
    t_start_s: float | None = None,
    t_end_s: float | None = None,
) -> dict[str, Any]:
    """Read and convert up to `limit` standardized samples per selected stream."""
    if limit < 1 or limit > 50:
        raise ValueError(f"limit must be between 1 and 50, got {limit}")

    descriptor = discover_bag(bag_path)
    cfg = _resolve_topic_map(map_config)
    mappings = _select_mappings(cfg, topic=topic, role=role)

    timings = scan_bag_timing(descriptor.root_path)
    starts = [item.start_ns for item in timings if item.start_ns is not None]
    bag_start_ns = min(starts) if starts else None

    start_ns = (
        bag_start_ns + round(t_start_s * 1_000_000_000.0)
        if (t_start_s is not None and bag_start_ns is not None)
        else None
    )
    stop_ns = (
        bag_start_ns + round(t_end_s * 1_000_000_000.0)
        if (t_end_s is not None and bag_start_ns is not None)
        else None
    )

    present_topics = {item.topic for item in timings}
    selected_mappings = [m for m in mappings if m.topic in present_topics]
    target_topics = sorted({m.topic for m in selected_mappings})

    per_role_samples: dict[str, list[dict[str, Any]]] = {m.role: [] for m in selected_mappings}
    mappings_by_topic: dict[str, list[SignalMapping]] = defaultdict(list)
    for m in selected_mappings:
        mappings_by_topic[m.topic].append(m)

    if target_topics:
        for raw in iter_bag_messages(
            descriptor.root_path,
            topics=target_topics,
            start_ns=start_ns,
            stop_ns=stop_ns,
        ):
            for mapping in mappings_by_topic[raw.topic]:
                bucket = per_role_samples[mapping.role]
                if len(bucket) >= limit:
                    continue
                for sample in map_message(raw.log_time_ns, raw.msg, mapping):
                    bucket.append(_sample_to_dict(sample))
                    if len(bucket) >= limit:
                        break
            if all(len(bucket) >= limit for bucket in per_role_samples.values()):
                break

    return {
        "bag_path": str(descriptor.root_path),
        "storage_format": descriptor.storage_format,
        "topic_map": cfg.name,
        "limit_per_role": limit,
        "roles": {
            r: {
                "topic": next(m.topic for m in selected_mappings if m.role == r),
                "sample_count": len(items),
                "samples": items,
            }
            for r, items in per_role_samples.items()
        },
    }
