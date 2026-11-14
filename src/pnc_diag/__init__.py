"""pnc-diag-mcp (open-source thin edition): offline bag interface & zero-deser inspector."""

from pnc_diag.inspector import (
    DEFAULT_TOPIC_MAP_PATH,
    BagSummaryReport,
    TopicTimingAudit,
    inspect_bag_summary,
    resolve_builtin_topic_map_path,
    sample_mapped_signals,
)
from pnc_diag.readers import (
    BagDiscoveryError,
    discover_bag,
    iter_bag_messages,
    load_topic_map,
    scan_bag_timing,
)

__version__ = "0.1.0"

__all__ = [
    "DEFAULT_TOPIC_MAP_PATH",
    "BagDiscoveryError",
    "BagSummaryReport",
    "TopicTimingAudit",
    "__version__",
    "discover_bag",
    "inspect_bag_summary",
    "iter_bag_messages",
    "load_topic_map",
    "resolve_builtin_topic_map_path",
    "sample_mapped_signals",
    "scan_bag_timing",
]
