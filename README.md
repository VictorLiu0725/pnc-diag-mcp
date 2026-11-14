# pnc-diag-mcp (Open-Source Thin Edition)

A read-only, open-source foundation for inspecting offline **Planning and Control (PNC)** and **LiDAR-SLAM** recordings. It provides a multi-format bag reader, a timing health check that does not deserialize message payloads, and an Agent Skill interface. It supports `.mcap`, `.db3` / `.sqlite3`, and ROS 1 `.bag` files **without a local ROS 2 installation**.

---

## Features and open-source scope

This edition provides standardized recording access, timing checks, and reproducible synthetic examples for Agent integration.

### Included and working

1. **Offline multi-format discovery and timing scans without payload deserialization** (`src/pnc_diag/readers/`, `src/pnc_diag/inspector.py`)
   - Detects `.mcap`, `.db3` / `.sqlite3`, ROS 1 `.bag`, and ROS 2 bag directories; validates relative paths in `metadata.yaml`.
   - Audits each topic in its original storage order: message count, duration, mean rate, median and P95 period, P95 jitter, gaps (`heuristic_large_gap`), timestamp regressions (`monotonic_violations`), and zero timestamps (`zero_timestamps`).
   - Uses bounded early-message sampling to check for `header.stamp`, count zero header stamps, and report the neutral `observed_header_minus_log_ms` observation. It does not assume cross-topic clock synchronization.
2. **Standard signal models and declarative YAML mapping** (`src/pnc_diag/models/`, `configs/`)
   - Includes standard mappings for Nav2 ([configs/topic_map.nav2.yaml](configs/topic_map.nav2.yaml)) and Autoware Universe ([configs/topic_map.autoware.yaml](configs/topic_map.autoware.yaml)).
   - Extracts dot paths and array elements (for example, `pose.covariance[0]`), converts `FLU` / `FRD` frames and `rad` / `deg` units, and returns `PoseSample`, `TwistSample`, `ImuSample`, `PathSample`, `ControlCmdSample`, `LaserScanSample`, and `DiagnosticEvent` values.
3. **Small CLI and stdio MCP server** (`src/pnc_diag/cli.py`, `src/pnc_diag/mcp_server.py`)
   - CLI commands: `pnc-diag inspect` and `pnc-diag preview`.
   - The `pnc-diag-mcp` stdio server exposes three tools: `ping`, `inspect_bag_summary`, and `sample_mapped_signals`.
   - Includes the orchestration [SKILL.md](.agents/skills/pnc-slam-diag/SKILL.md) and [fault pattern reference](.agents/skills/pnc-slam-diag/references/fault_patterns.md).
4. **Reproducible synthetic examples** (`fault_sim_kit/`)
   - Generates one baseline recording and nine synthetic fault, missing-data, or zero-timestamp examples under `data/golden/` for integration and regression exercises.

### Outside this edition's scope

The following components remain in the private diagnostic core:

- Automated physical diagnostic operators for localization jumps versus odometry slip, planning path jumps / curvature / solver checks, and control cross-correlation delay and FFT oscillation analysis.
- The First-Domino cross-module causal arbitration engine and five-panel diagnostic PNG plotter.
- Calibrated detection rules and enterprise CI KPI gates such as `--assert-kpi`.
- Customer-specific `.msg` / `.proto` adapters.

---

## Requirements and installation

- **Python:** `>=3.10` (verified in clean Python `3.10.12` and `3.13.17` environments)
- **Package manager:** [`uv`](https://docs.astral.sh/uv/) is recommended; `uv.lock` is included.
- **ROS 2 runtime:** Not required.

> **Notes**
> 1. The default Nav2 and Autoware topic maps are bundled in the wheel under `pnc_diag._configs`. The default Nav2 map loads without `--map-config` whether you use `uv sync`, `pip install -e .`, or install the wheel from another directory. Use `--map-config autoware` for the bundled Autoware map, or pass a path to a custom YAML map.
> 2. In a source checkout, `python -m fault_sim_kit.generator` writes to `<repo>/data/golden/` by default. After a non-editable install, it writes to `./data/golden/` in the current working directory. Use `--out <directory>` to choose another location.
> 3. If your shell has sourced ROS 2 (for example, Humble), prefix commands with `env -u PYTHONPATH` to keep system Python paths out of the virtual environment.

```bash
# Install dependencies.
env -u PYTHONPATH uv sync

# Check the CLI.
env -u PYTHONPATH uv run pnc-diag --version
```

---

## Synthetic examples and tests

The distribution does not contain the approximately 47 MB of generated `.mcap` files. Run the tests or generate the examples locally:

```bash
# Lint and run all unit tests; pre-existing data/golden/ files are not required.
env -u PYTHONPATH uv run ruff check .
env -u PYTHONPATH uv run pytest

# Generate all 10 synthetic MCAP examples under data/golden/.
env -u PYTHONPATH uv run python -m fault_sim_kit.generator
```

---

## Try it and share feedback

Start with the generated synthetic examples, then try `pnc-diag inspect <bag_path>` or `pnc-diag preview <bag_path>` on a recording you are authorized to analyze. This edition provides inspection and signal sampling, not a definitive root-cause diagnosis.

- **Reproducible bugs:** Open a [Bug report](https://github.com/VictorLiu0725/pnc-diag-mcp/issues/new?template=bug_report.md). Include your Python version, operating system, install method, recording format, command, expected and actual behavior, and sanitized output.
- **Usage questions, workflow feedback, and feature ideas:** Start a [Discussion](https://github.com/VictorLiu0725/pnc-diag-mcp/discussions).

Do not upload raw customer recordings, proprietary bags, credentials, or identifying logs to public Issues or Discussions. Share a synthetic or anonymized minimal reproduction when possible.

---

## CLI usage

This edition provides the `inspect` and `preview` commands:

```bash
# Inspect timing, topic-role coverage, and neutral header-time observations.
env -u PYTHONPATH uv run pnc-diag inspect data/golden/case_01_tf_jump

# Inspect missing-sensor and zero-timestamp counterexamples.
env -u PYTHONPATH uv run pnc-diag inspect data/golden/case_08_missing_imu_and_scan
env -u PYTHONPATH uv run pnc-diag inspect data/golden/case_09_invalid_zero_stamps --json

# Preview bounded samples of mapped standard signals.
env -u PYTHONPATH uv run pnc-diag preview data/golden/nav_baseline --role odom_pose --limit 3
env -u PYTHONPATH uv run pnc-diag preview data/golden/case_01_tf_jump --topic /amcl_pose --t-start 44.5 --t-end 46.0 --json

# Inspect a recording with the bundled Autoware preset or a custom mapping file.
env -u PYTHONPATH uv run pnc-diag inspect /path/to/autoware_bag --map-config autoware
env -u PYTHONPATH uv run pnc-diag inspect /path/to/autoware_bag --map-config configs/topic_map.autoware.yaml
```

---

## MCP server and Agent Skill

### `pnc-diag-mcp` stdio tools

| Tool | Arguments | Returns |
|---|---|---|
| `ping` | None | Service status, `version`, and `edition` (`open-source-thin`) |
| `inspect_bag_summary` | `bag_path`, `map_config` | Storage format, duration, available and missing roles, `clock_comparability`, warnings, and per-topic timing statistics |
| `sample_mapped_signals` | `bag_path`, `topic`, `role`, `map_config`, `limit`, `t_start_s`, `t_end_s` | Bounded standardized signal samples (`limit`: `1..50`) |

### Example MCP client configuration

```json
{
  "mcpServers": {
    "pnc-diag": {
      "command": "env",
      "args": [
        "-u",
        "PYTHONPATH",
        "uv",
        "--directory",
        "<PATH_TO_REPO>",
        "run",
        "pnc-diag-mcp"
      ]
    }
  }
}
```

The Agent Skill is in [.agents/skills/pnc-slam-diag/SKILL.md](.agents/skills/pnc-slam-diag/SKILL.md), with a [fault pattern reference](.agents/skills/pnc-slam-diag/references/fault_patterns.md).

---

## Data and diagnostic limitations

1. **Inspection foundation only:** This edition does not calculate quantitative localization, planning, or control anomaly metrics, or make a definitive multi-sensor root-cause determination. Review the evidence manually or use specialized analyzers for diagnosis.
2. **No clock synchronization inference:** `observed_header_minus_log_ms` records the difference between a message header timestamp and its recorded log timestamp. By itself, it cannot distinguish sensor clock drift from queueing or transport delay. This edition reports `clock_comparability="unverified"` by default.
3. **Synthetic examples are exercises, not a blind benchmark:** `fault_sim_kit` produces simplified closed-loop kinematic data, not representative sensor noise, vehicle dynamics, or real-vehicle acceptance data. The repository does not commit `golden_truth.json` or generated bag files, but it does publish the definitions of all 10 scenarios and the code that generates their ground truth. Use a separate, unpublished dataset for blind tests or benchmark evaluations.
4. **Read-only offline operation:** The tools do not send vehicle commands, change parameters, or operate ROS nodes.

---

## License and third-party notices

- **License:** This repository is released under the [Apache License 2.0](LICENSE); see [NOTICE](NOTICE). Subject to the license terms, others may use, modify, and redistribute the published code commercially. The private physical diagnostic core, calibrated rules, and enterprise gates are outside this repository.
- **Dependencies and provenance:** See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for dependency licenses and acknowledgments of upstream code and design references. The documented runtime and development dependencies use permissive MIT, BSD, or Apache-2.0 licenses; no GPL code is included in this repository.
