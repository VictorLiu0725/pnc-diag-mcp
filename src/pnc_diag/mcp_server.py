"""Open-source thin stdio MCP server exposing standard offline bag inspection tools."""

from __future__ import annotations

from typing import Any

from mcp.server.mcpserver import MCPServer

from pnc_diag import __version__
from pnc_diag.inspector import inspect_bag_summary as _inspect_bag_summary
from pnc_diag.inspector import sample_mapped_signals as _sample_mapped_signals

mcp = MCPServer("pnc-diag")


@mcp.tool()
def ping() -> dict[str, str]:
    """Check open-source thin MCP server availability and capability edition."""
    return {
        "server": "pnc-diag",
        "version": __version__,
        "edition": "open-source-thin",
        "status": "ready",
    }


@mcp.tool()
def inspect_bag_summary(
    bag_path: str,
    map_config: str | None = None,
) -> dict[str, Any]:
    """Run zero-deserialization timing scan, role check, and neutral header-log observation."""
    return _inspect_bag_summary(bag_path, map_config=map_config).to_dict()


@mcp.tool()
def sample_mapped_signals(
    bag_path: str,
    topic: str | None = None,
    role: str | None = None,
    map_config: str | None = None,
    limit: int = 5,
    t_start_s: float | None = None,
    t_end_s: float | None = None,
) -> dict[str, Any]:
    """Deserialize and normalize up to `limit` standardized samples without flooding context."""
    return _sample_mapped_signals(
        bag_path,
        topic=topic,
        role=role,
        map_config=map_config,
        limit=limit,
        t_start_s=t_start_s,
        t_end_s=t_end_s,
    )


def main() -> None:
    """Run stdio MCP server."""
    mcp.run()


if __name__ == "__main__":
    main()
