"""Selective, ROS-free streaming reader across MCAP, SQLite3, and ROS 1 bags."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from pnc_diag.readers.discovery import discover_bag


@dataclass(frozen=True, slots=True)
class RawBagMessage:
    """Deserialized message paired with recording metadata."""

    topic: str
    msg_type: str
    log_time_ns: int
    header_stamp_ns: int | None
    msg: Any


def resolve_typestore(ros_distro: str | None) -> Any:
    """Resolve a rosbags Typestore from a ROS distribution hint (e.g. 'humble', 'jazzy')."""
    if not ros_distro:
        return None
    normalized = ros_distro.lower().removeprefix("ros2_").removeprefix("ros1_")
    for store in Stores:
        candidate = store.value.lower().removeprefix("ros2_").removeprefix("ros1_")
        if candidate == normalized:
            return get_typestore(store)
    raise ValueError(f"Unsupported ros_distro: {ros_distro!r}")


def extract_header_stamp_ns(msg: Any) -> int | None:
    """Extract std_msgs/Header stamp in nanoseconds if present on a message."""
    header = getattr(msg, "header", None)
    if header is None and isinstance(msg, dict):
        header = msg.get("header")
    if header is None:
        return None

    stamp = getattr(header, "stamp", None)
    if stamp is None and isinstance(header, dict):
        stamp = header.get("stamp")
    if stamp is None:
        return None

    sec = getattr(stamp, "sec", None) if not isinstance(stamp, dict) else stamp.get("sec")
    nanosec = (
        getattr(stamp, "nanosec", None) if not isinstance(stamp, dict) else stamp.get("nanosec")
    )
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def iter_bag_messages(
    bag_path: str | Path,
    *,
    topics: set[str] | list[str] | None = None,
    start_ns: int | None = None,
    stop_ns: int | None = None,
    ros_distro: str | None = None,
) -> Iterator[RawBagMessage]:
    """Stream deserialized messages filtered by topic list and [start_ns, stop_ns] window."""
    desc = discover_bag(bag_path)
    topic_set = set(topics) if topics is not None else None
    typestore = resolve_typestore(ros_distro)

    with AnyReader(list(desc.files), default_typestore=typestore) as reader:
        selected = [
            conn for conn in reader.connections if topic_set is None or conn.topic in topic_set
        ]
        if not selected:
            return
        for conn, log_time_ns, rawdata in reader.messages(
            connections=selected,
            start=start_ns,
            stop=stop_ns,
        ):
            msg = reader.deserialize(rawdata, conn.msgtype)
            header_ns = extract_header_stamp_ns(msg)
            yield RawBagMessage(
                topic=conn.topic,
                msg_type=conn.msgtype,
                log_time_ns=int(log_time_ns),
                header_stamp_ns=header_ns,
                msg=msg,
            )

