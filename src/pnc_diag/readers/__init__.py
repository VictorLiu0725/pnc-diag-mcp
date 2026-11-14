"""Offline bag discovery, zero-deserialization timing scan, streaming & mapping."""

from pnc_diag.readers.discovery import BagDiscoveryError, discover_bag
from pnc_diag.readers.stream_reader import RawBagMessage, extract_header_stamp_ns, iter_bag_messages
from pnc_diag.readers.timing_scanner import scan_bag_timing
from pnc_diag.readers.topic_mapper import (
    SignalMapping,
    TopicMapConfig,
    extract_dot_path,
    load_topic_map,
    map_message,
    resolve_message_time,
)

__all__ = [
    "BagDiscoveryError",
    "RawBagMessage",
    "SignalMapping",
    "TopicMapConfig",
    "discover_bag",
    "extract_dot_path",
    "extract_header_stamp_ns",
    "iter_bag_messages",
    "load_topic_map",
    "map_message",
    "resolve_message_time",
    "scan_bag_timing",
]
