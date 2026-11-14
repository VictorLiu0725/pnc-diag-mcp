"""Verification tests for the open-source thin edition: examples, CLI, MCP, Skill, and boundary."""

from __future__ import annotations

import asyncio
import dataclasses
import json
import re
from pathlib import Path

import pytest
import yaml

from fault_sim_kit import FaultScenarioConfig, generate_golden_dataset
from pnc_diag import (
    DEFAULT_TOPIC_MAP_PATH,
    inspect_bag_summary,
    mcp_server,
    resolve_builtin_topic_map_path,
    sample_mapped_signals,
)
from pnc_diag.cli import main as cli_main
from pnc_diag.readers.discovery import BagDiscoveryError

REPO_ROOT = Path(__file__).resolve().parents[1]
SKILL_DIR = REPO_ROOT / ".agents" / "skills" / "pnc-slam-diag"
SKILL_MD = SKILL_DIR / "SKILL.md"
FAULT_PATTERNS_MD = SKILL_DIR / "references" / "fault_patterns.md"
_LINK_RE = re.compile(r"\[[^\]]+\]\(([^)#]+)(?:#[^)]*)?\)")


@pytest.fixture(scope="module")
def sample_bags(tmp_path_factory: pytest.TempPathFactory) -> Path:
    out_dir = tmp_path_factory.mktemp("thin_oss_examples")
    # Build a compact representative subset using FaultScenarioConfig for fast CI runtime.
    from fault_sim_kit.generator import DEFAULT_SCENARIOS

    picked = tuple(
        (
            dataclasses.replace(cfg, params={**cfg.params, "drop_start_s": 7.0})
            if cfg.name == "case_03_time_desync"
            else cfg
        )
        for cfg in DEFAULT_SCENARIOS
        if cfg.name
        in {
            "nav_baseline",
            "case_01_tf_jump",
            "case_03_time_desync",
            "case_08_missing_imu_and_scan",
            "case_09_invalid_zero_stamps",
        }
    )
    generate_golden_dataset(out_dir, scenarios=picked, duration_override_s=12.0)
    return out_dir


def test_boundary_excludes_private_kernel_and_rules() -> None:
    private_targets = [
        REPO_ROOT / "src" / "pnc_diag" / "analyzers",
        REPO_ROOT / "src" / "pnc_diag" / "causal_engine.py",
        REPO_ROOT / "src" / "pnc_diag" / "plotter.py",
        REPO_ROOT / "src" / "pnc_diag" / "models" / "rules.py",
        REPO_ROOT / "src" / "pnc_diag" / "models" / "contracts.py",
        REPO_ROOT / "configs" / "rules.default.yaml",
    ]
    for path in private_targets:
        assert not path.exists(), f"Private module/rule unexpectedly present in OSS candidate: {path}"


def test_skill_spec_readme_and_mcp_parity() -> None:
    assert SKILL_MD.is_file()
    assert FAULT_PATTERNS_MD.is_file()

    text = SKILL_MD.read_text(encoding="utf-8")
    assert text.startswith("---\n")
    _, fm_raw, body = text.split("---\n", 2)
    meta = yaml.safe_load(fm_raw)
    assert meta["name"] == "pnc-slam-diag"

    for doc in (SKILL_MD, FAULT_PATTERNS_MD):
        links = _LINK_RE.findall(doc.read_text(encoding="utf-8"))
        for target in links:
            if target.startswith(("http://", "https://")):
                continue
            assert (doc.parent / target).resolve().exists(), f"Broken link {target!r} in {doc}"

    tools = asyncio.run(mcp_server.mcp.list_tools())
    tool_names = {t.name for t in tools}
    assert tool_names == {"ping", "inspect_bag_summary", "sample_mapped_signals"}

    readme_text = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
    for name in tool_names:
        assert name in body
        assert name in readme_text

    # Ensure public docs do not leak excluded private tools or CLI flags.
    forbidden = (
        "correlate_root_cause_and_plot",
        "analyze_localization_health",
        "analyze_planning_health",
        "analyze_control_health",
        "detect_anomaly_windows",
        "--assert-kpi",
        "--assert-healthy",
        "rules.default.yaml",
    )
    for token in forbidden:
        assert token not in body
        assert token not in FAULT_PATTERNS_MD.read_text(encoding="utf-8")


def test_inspect_bag_summary_and_example_cases(sample_bags: Path) -> None:
    manifest = json.loads((sample_bags / "golden_truth.json").read_text(encoding="utf-8"))
    assert len(manifest["cases"]) == 5

    base = inspect_bag_summary(sample_bags / "nav_baseline")
    assert base.storage_format == "mcap"
    assert "odom_pose" in base.available_roles
    assert base.clock_comparability.startswith("unverified")

    missing = inspect_bag_summary(sample_bags / "case_08_missing_imu_and_scan")
    assert {"imu", "laser_scan"} <= set(missing.missing_roles)

    zeros = inspect_bag_summary(sample_bags / "case_09_invalid_zero_stamps")
    assert any(w.startswith("zero_header_stamp_in_sample:") for w in zeros.warnings)

    desync = inspect_bag_summary(sample_bags / "case_03_time_desync")
    assert any(w.startswith("large_gap_detected:/scan") for w in desync.warnings)


def test_sample_mapped_signals_and_thin_cli_and_mcp(sample_bags: Path, capsys) -> None:
    bag = sample_bags / "nav_baseline"
    sampled = sample_mapped_signals(bag, role="odom_pose", limit=3)
    assert sampled["roles"]["odom_pose"]["sample_count"] == 3
    assert sampled["roles"]["odom_pose"]["samples"][0]["sample_type"] == "PoseSample"

    assert cli_main(["inspect", str(bag), "--json"]) == 0
    inspect_json = json.loads(capsys.readouterr().out)
    assert inspect_json["storage_format"] == "mcap"

    assert cli_main(["preview", str(bag), "--role", "local_plan", "--limit", "2", "--json"]) == 0
    preview_json = json.loads(capsys.readouterr().out)
    assert preview_json["roles"]["local_plan"]["sample_count"] == 2
    assert preview_json["roles"]["local_plan"]["samples"][0]["sample_type"] == "PathSample"

    mcp_summary = mcp_server.inspect_bag_summary(str(bag))
    assert mcp_summary["storage_format"] == "mcap"
    mcp_preview = mcp_server.sample_mapped_signals(str(bag), role="control_cmd", limit=2)
    assert mcp_preview["roles"]["control_cmd"]["sample_count"] == 2

    assert cli_main(["inspect", str(sample_bags / "does_not_exist")]) == 2
    _ = capsys.readouterr()
    with pytest.raises(BagDiscoveryError):
        inspect_bag_summary(sample_bags / "does_not_exist")
    assert isinstance(FaultScenarioConfig.__name__, str)


def test_builtin_topic_map_presets_and_missing_path_errors(sample_bags: Path, tmp_path: Path) -> None:
    assert DEFAULT_TOPIC_MAP_PATH.is_file()
    assert resolve_builtin_topic_map_path("nav2").is_file()
    assert resolve_builtin_topic_map_path("autoware").is_file()

    bag = sample_bags / "nav_baseline"
    auto = inspect_bag_summary(bag, map_config="autoware")
    assert {"ndt_pose", "chassis_feedback", "localization_pose"} <= set(auto.missing_roles)
    assert "tf" in auto.available_roles

    with pytest.raises(FileNotFoundError):
        inspect_bag_summary(bag, map_config=tmp_path / "does_not_exist.yaml")

    with pytest.raises(FileNotFoundError):
        resolve_builtin_topic_map_path("unknown_preset")
