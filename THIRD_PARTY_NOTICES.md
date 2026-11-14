# Third-Party Dependencies & Attribution Notices

> **License Status**: This repository is licensed under the **Apache License 2.0 (`Apache-2.0`)** — see [LICENSE](LICENSE) and [NOTICE](NOTICE).

---

## 1. Runtime & Development Dependencies (`pyproject.toml` + `uv.lock`)

All Python dependencies required by this open-source thin edition use permissive open-source licenses; no GPL or AGPL runtime/development dependencies are included (`scipy`, `duckdb`, and `matplotlib` are **not** required by this thin edition):

| Package | Version Constraint | Tested Locked Version | Declared License | Role in Open-Source Thin Edition |
|---|---|---|---|---|
| `mcap` | `>=1.1.1` | `1.6.0` | MIT | Chunk-indexed MCAP header & `MessageIndex` timing scan |
| `mcp` | `>=2.0.0` | `2.3.0` | MIT | Model Context Protocol stdio server (`MCPServer`) |
| `numpy` | `>=1.26.0` | `2.5.4` | BSD-3-Clause (with bundled 0BSD / MIT / Zlib / CC0-1.0 notices) | Numeric arrays & period percentile statistics |
| `pyyaml` | `>=6.0.1` | `6.0.3` | MIT | YAML `topic_map` & ROS 2 `metadata.yaml` parsing |
| `rosbags` | `>=0.10.0` | `0.11.9` | Apache-2.0 | ROS-free reading across `.mcap` / `.db3` / `.bag` & synthetic MCAP writing |
| `pytest` (dev) | `>=8.0.0` | `9.2.0` | MIT | Unit & reproducible-example tests |
| `ruff` (dev) | `>=0.6.0` | `0.17.5` | MIT | Linting & formatting checks |

---

## 2. Upstream Open-Source Design & Code Provenance Audit

During design and implementation, publicly available permissive ROS tooling was inspected. No source files were copied verbatim; where specific helper idioms or architectural patterns were adapted, attribution and upstream licenses are recorded below:

| Local File(s) | Upstream Reference & License | What Was Adapted vs Rewritten |
|---|---|---|
| `src/pnc_diag/readers/discovery.py` | **sylvesterkaczmarek/rosbag-doctor** (`MIT License`, Copyright (c) 2026 Sylvester Kaczmarek) | Natural numeric filename sorting (`_natural_key`) and `metadata.yaml` relative-path escape checks follow `rosbag_doctor/readers.py` idioms; error types and ROS 1 discovery support were independently implemented. |
| `src/pnc_diag/readers/timing_scanner.py` | **sylvesterkaczmarek/rosbag-doctor** (`MIT License`); **Afloat16/bag-timeline-audit** (`MIT License`, Copyright (c) 2026 Afloat16) | Read-only SQLite URI `?mode=ro` + `PRAGMA query_only = ON`, `median` / `p95` / `p95_jitter` summary statistics, and heuristic large-gap check (`p95 <= 1.5 * median` and `max_gap > max(100, 5 * median)`) draw on `rosbag-doctor`; non-pre-sorted stored-order clock regression auditing (`ORDER BY id`, `log_time_order=False`) draws on `bag-timeline-audit`. Algorithm R reservoir sampling, min-heap Top-K intervals, and chunk-offset multi-channel `heapq.merge` were independently written. |
| `src/pnc_diag/readers/stream_reader.py` & `fault_sim_kit/generator.py` | **1405264556/robotdev-tools** (`Apache-2.0 License`) | Multi-file `rosbags.highlevel.AnyReader` + `Stores` typestore matching pattern (`analyzer.py`) and pure-Python `rosbags.rosbag2.Writer` synthetic MCAP generation approach (`demo.py`); closed-loop kinematic trajectories and 10 fault/refusal scenarios in `fault_sim_kit/generator.py` are original. |
| `src/pnc_diag/readers/topic_mapper.py` | **binabik-ai/mcp-rosbags** (`Apache-2.0 License`) | Concept of declarative YAML dot-path signal mapping (`SchemaManager`); FLU/FRD, degree/radian, and multi-signal mappers are original. |
| `.agents/skills/pnc-slam-diag/SKILL.md` | Author's internal prototype skills (`autonomy-debug`, `vibe-decision-archaeologist`) | Environment execution-safety table (`Simulation` / `HIL` / `Real Hardware`) and evidence-gating phrasing adapted from the author's prior skill drafts. |

### Non-Inclusion Confirmation for Copyleft Survey References

Early open-source landscape notes also reviewed two **GPL-3.0** tools (`pixel-robotics/mcap_mcp_server` and `MichaelGrupp/evo`) strictly for public library capabilities and high-level benchmark concepts. Automated 4-line code shingle comparison confirmed **0 matches** against all files in `src/` and `fault_sim_kit/`; no GPL-3.0 code or dependency is present in this package.
