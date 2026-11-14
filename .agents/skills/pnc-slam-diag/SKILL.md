---
name: pnc-slam-diag
description: >-
  Inspect offline autonomous driving Planning & Control (PNC) and LiDAR-SLAM ROS recordings
  (.mcap, .db3, .sqlite3, .bag) via the pnc-diag open-source MCP interface. TRIGGER when the user
  asks to inspect recording metadata, timing drops, clock domains, topic role coverage, or sample
  standardized telemetry from an offline rosbag/MCAP/db3 file. DO NOT TRIGGER for general ROS/ROS 2
  tutorials, launch/URDF authoring, live node tuning, or tasks without a recording path.
---

# PNC & LiDAR-SLAM Recording Inspection Skill (Open-Source Thin Edition)

Use the `pnc-diag` stdio MCP server as the read-only bag interface for `.mcap`, `.db3`/`.sqlite3`, and ROS 1 `.bag` files without requiring a local ROS 2 installation. Load [references/fault_patterns.md](references/fault_patterns.md) when mapping telemetry symptoms to standard Nav2/Autoware topic roles or reproducible synthetic example bags.

## Capability Scope & Hard Rules

1. **Thin Open-Source Scope**: This edition provides zero-deserialization timing inspection, topic role checks, neutral `header - log_time` observations, and bounded standardized signal sampling. It does **not** include automated cross-module physical analyzers, causal precedence arbitration engines, calibrated domain rules, or CI KPI gates.
2. **No Clock Guessing**: Never infer or claim cross-topic clock synchronization or calibration offsets from observed `header - log_time` differences (`observed_header_minus_log_ms`). Without external synchronization evidence, treat `clock_comparability="unverified"` as a hard barrier against cross-topic causal timing claims.
3. **No Fabrication or Overclaiming**: Every count, rate, gap, timestamp, role mapping, and sampled value must come directly from `inspect_bag_summary` or `sample_mapped_signals`. Do not assert a definitive root cause using this thin inspection interface alone (`INSUFFICIENT_EVIDENCE` without domain analyzers and verified calibration).

## Workflow

### 1. Confirm Inputs & Stack Mapping
- Collect `bag_path` (`.mcap`, `.db3`, `.sqlite3`, `.bag`, or ROS 2 bag directory) and optional `map_config` (defaults to Nav2 `configs/topic_map.nav2.yaml`; use `configs/topic_map.autoware.yaml` for standard Autoware Universe topics).

### 2. Verify Server & Inspect Bag Summary (`ping` + `inspect_bag_summary`)
- Call `ping()` and confirm `status == "ready"` (note `edition == "open-source-thin"`).
- Call `inspect_bag_summary(bag_path, map_config)`.
- Record:
  - `storage_format`, `file_count`, `duration_s`, `total_messages`
  - `available_roles`, `missing_roles`, `unmapped_topics`
  - `clock_comparability` and `warnings` (`zero_log_timestamp`, `monotonic_violation`, `large_gap_detected`, `zero_header_stamp_in_sample`)
  - Per-topic `mean_rate_hz`, `median_period_ms`, `p95_period_ms`, `p95_jitter_ms`, `max_gap_ms`, `observed_header_minus_log_ms`

### 3. Sample Normalized Telemetry (`sample_mapped_signals`)
- Call `sample_mapped_signals(bag_path, topic, role, map_config, limit, t_start_s, t_end_s)` with small `limit` (e.g. `5`, max `50`) to verify frame IDs, valid header stamps (`stamp_is_valid`, `time_domain`), pose/twist/control fields, and laser/path structure around user-specified intervals.

### 4. Structured Inspection Output
Present findings clearly under four sections:
1. **Recording Overview & Operational State**: `ANALYZED` / `PARTIALLY_ANALYZED` / `UNAVAILABLE`, storage format, duration, and `available_roles` vs `missing_roles`.
2. **Timing & Timestamp Observations**: Topic rates, jitter, detected drop gaps, monotonic/zero-stamp warnings, and neutral `observed_header_minus_log_ms` values with explicit `clock_comparability="unverified"` reminder.
3. **Standardized Signal Samples**: Key observations from `sample_mapped_signals` (`PoseSample`, `ImuSample`, `PathSample`, `ControlCmdSample`, `LaserScanSample`, `DiagnosticEvent`).
4. **Checklist & Next-Step Hypotheses**: Unconfirmed troubleshooting directions mapped from missing signals, timing gaps, or sampled anomalies, plus what external evidence or specialized analyzers are required before concluding root cause.

## Operational States & Live System Safety Gate

- **`ANALYZED`**: Bag opened and timing/sampling tools completed cleanly.
- **`PARTIALLY_ANALYZED`**: Timing scan succeeded, while custom/proprietary topics could not be deserialized (`unmapped_topics` or schema gaps).
- **`UNAVAILABLE`**: Server unreachable or unreadable bag (`BagDiscoveryError`).
- **Live System Safety Gate**: This toolset is strictly **offline read-only**. If a user asks to actuate `/cmd_vel`, tune parameters, or restart nodes on a physical vehicle:
  - `Observe` / `Record`: `PERMITTED` (`Simulation`, `HIL`, `Real Hardware`)
  - `Restart Node` / `Change Parameter` (non-safety): `PERMITTED` (`Simulation`), `REQUIRES_CONFIRMATION` (`HIL`, `Real Hardware`)
  - `Send Motion Command` (`/cmd_vel`): `PERMITTED` (`Simulation`), `REQUIRES_CONFIRMATION` (`HIL`), `FORBIDDEN` (`Real Hardware`)
  - `Disable Safety/E-Stop`: `FORBIDDEN` everywhere. Never self-elevate permissions.

## MCP Tool Reference (`src/pnc_diag/mcp_server.py`)

| Tool | Args | Key Outputs |
|---|---|---|
| `ping()` | — | `server`, `version`, `edition`, `status` |
| `inspect_bag_summary(...)` | `bag_path`, `map_config` | `storage_format`, `duration_s`, `available_roles`, `missing_roles`, `clock_comparability`, `warnings`, `topics[]` |
| `sample_mapped_signals(...)` | `bag_path`, `topic`, `role`, `map_config`, `limit`, `t_start_s`, `t_end_s` | `topic_map`, `limit_per_role`, `roles{topic, sample_count, samples[]}` |
