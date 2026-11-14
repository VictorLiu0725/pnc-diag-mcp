"""Smoke tests verifying thin open-source imports, entry points, and edition scope."""

from __future__ import annotations

import importlib

from pnc_diag import __version__
from pnc_diag.cli import main as cli_main
from pnc_diag.mcp_server import ping


def test_core_dependencies_importable() -> None:
    for mod_name in ("mcap", "mcp", "numpy", "rosbags", "yaml"):
        module = importlib.import_module(mod_name)
        assert module is not None


def test_mcp_ping_and_cli_version(capsys) -> None:
    res = ping()
    assert res["server"] == "pnc-diag"
    assert res["version"] == __version__
    assert res["edition"] == "open-source-thin"
    assert res["status"] == "ready"

    assert cli_main([]) == 0
    captured = capsys.readouterr()
    assert "pnc-diag" in captured.out
