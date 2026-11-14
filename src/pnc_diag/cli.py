"""Thin open-source CLI (`inspect` and `preview`) for offline ROS recordings."""

from __future__ import annotations

import argparse
import json
import sys

from pnc_diag import __version__
from pnc_diag.inspector import BagSummaryReport, inspect_bag_summary, sample_mapped_signals
from pnc_diag.readers.discovery import BagDiscoveryError


def build_parser() -> argparse.ArgumentParser:
    """Build argument parser for thin open-source subcommands."""
    parser = argparse.ArgumentParser(
        prog="pnc-diag",
        description=(
            "Open-source thin ROS recording inspector and standardized signal mapper "
            "(mcap / db3 / bag)."
        ),
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    subparsers = parser.add_subparsers(dest="command")

    inspect_p = subparsers.add_parser(
        "inspect",
        help="Run zero-deserialization timing scan, role check, and header observation.",
    )
    inspect_p.add_argument("bag_path", help="Path to bag directory or .mcap/.db3/.bag file.")
    inspect_p.add_argument("--map-config", default=None, help="Path to custom topic_map YAML.")
    inspect_p.add_argument("--json", action="store_true", help="Emit structured JSON output.")

    preview_p = subparsers.add_parser(
        "preview",
        help="Deserialize and convert up to N standardized physical signal samples.",
    )
    preview_p.add_argument("bag_path", help="Path to bag directory or .mcap/.db3/.bag file.")
    preview_p.add_argument("--topic", default=None, help="Filter by configured ROS topic.")
    preview_p.add_argument("--role", default=None, help="Filter by logical role in topic_map.")
    preview_p.add_argument("--limit", type=int, default=5, help="Max samples per role (1..50).")
    preview_p.add_argument("--t-start", type=float, default=None, help="Window start (rel s).")
    preview_p.add_argument("--t-end", type=float, default=None, help="Window end (rel s).")
    preview_p.add_argument("--map-config", default=None, help="Path to custom topic_map YAML.")
    preview_p.add_argument("--json", action="store_true", help="Emit structured JSON output.")
    return parser


def _format_inspect_text(report: BagSummaryReport) -> str:
    lines = [
        (
            f"Bag={report.bag_path} format={report.storage_format} "
            f"files={report.file_count} duration={report.duration_s:.2f}s "
            f"messages={report.total_messages}"
        ),
        f"Available roles: {sorted(report.available_roles.keys())}",
        f"Missing roles: {list(report.missing_roles)}",
        f"Clock comparability: {report.clock_comparability}",
    ]
    if report.warnings:
        lines.append(f"Timing warnings: {list(report.warnings)}")
    for item in report.topics:
        lines.append(
            f"  - {item.topic} (role={item.role or 'unmapped'}): count={item.message_count}, "
            f"rate={item.mean_rate_hz}Hz, p95_jitter={item.p95_jitter_ms}ms, "
            f"obs_hdr_minus_log={item.observed_header_minus_log_ms}ms"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    """Run thin CLI entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 0

    try:
        if args.command == "inspect":
            report = inspect_bag_summary(args.bag_path, map_config=args.map_config)
            if args.json:
                print(json.dumps(report.to_dict(), indent=2))
            else:
                print(_format_inspect_text(report))
            return 0

        if args.command == "preview":
            payload = sample_mapped_signals(
                args.bag_path,
                topic=args.topic,
                role=args.role,
                map_config=args.map_config,
                limit=args.limit,
                t_start_s=args.t_start,
                t_end_s=args.t_end,
            )
            if args.json:
                print(json.dumps(payload, indent=2))
            else:
                print(
                    f"Bag={payload['bag_path']} format={payload['storage_format']} "
                    f"map={payload['topic_map']}"
                )
                for role, info in payload["roles"].items():
                    print(
                        f"  - role={role} topic={info['topic']} "
                        f"samples={info['sample_count']}"
                    )
            return 0
    except (BagDiscoveryError, ValueError, FileNotFoundError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
