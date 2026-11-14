# pnc-diag-mcp (Open-Source Thin Edition)

面向自动驾驶 **PNC（规划与控制）** 与 **LiDAR-SLAM** 离线录包的**开源基础接口、零反序列化体检与标准 Skill 工具包**，支持只读解析 `.mcap`、`.db3` / `.sqlite3` 与 ROS 1 `.bag`，**无需在本地安装 ROS 2**。

---

## 功能与开源边界说明

本开源版本聚焦**开源标准层与可复现数据基座**，核心定位是标准化录包读取、时序健康体检与 Agent Skill 接入接口。

### 本版本包含（已实现且可运行）

1. **离线多格式发现与零反序列化时序扫描（`src/pnc_diag/readers/`、`src/pnc_diag/inspector.py`）**
   - 统一识别 `.mcap`、`.db3` / `.sqlite3`、ROS 1 `.bag` 与 ROS 2 目录（校验 `metadata.yaml` 相对路径安全）；
   - 按存储原始顺序统计各话题消息数、时长、平均频率、周期中位数/P95、P95 抖动、断流间隙（`heuristic_large_gap`）、时间戳倒退（`monotonic_violations`）与零时间戳（`zero_timestamps`）；
   - 基于有界前段采样统计话题是否包含 `header.stamp`、零 `header` 数以及中性观测值 `observed_header_minus_log_ms`（不作跨时钟同步假设）。
2. **标准信号模型与声明式 YAML 映射（`src/pnc_diag/models/`、`configs/`）**
   - 内置 Nav2（[configs/topic_map.nav2.yaml](configs/topic_map.nav2.yaml)）与 Autoware Universe（[configs/topic_map.autoware.yaml](configs/topic_map.autoware.yaml)）标准话题映射；
   - 支持点路径/数组下标抽取（如 `pose.covariance[0]`）以及 `FLU`/`FRD`、`rad`/`deg` 转换，输出 `PoseSample`、`TwistSample`、`ImuSample`、`PathSample`、`ControlCmdSample`、`LaserScanSample`、`DiagnosticEvent`。
3. **精简 CLI 与 stdio MCP Server（`src/pnc_diag/cli.py`、`src/pnc_diag/mcp_server.py`）**
   - CLI 子命令：`pnc-diag inspect`、`pnc-diag preview`；
   - stdio MCP 服务 `pnc-diag-mcp` 暴露 3 个工具：`ping`、`inspect_bag_summary`、`sample_mapped_signals`；
   - 配套编排规范 [SKILL.md](.agents/skills/pnc-slam-diag/SKILL.md) 与 [references/fault_patterns.md](.agents/skills/pnc-slam-diag/references/fault_patterns.md)。
4. **可复现合成示例套件（`fault_sim_kit/`）**
   - 离线生成 1 个基线与 9 个合成故障/缺失/零时间戳示例录包（`data/golden/`）供工具联调与回归练习。

### 本版本不包含（保留在私有/商业内核）

- 自动物理诊断算子（定位跳变 vs 轮速打滑判别、规划轨迹跳变/曲率/求解器检查、控制互相关延时与 FFT 画龙频谱）；
- First-Domino 跨模块因果裁决引擎与 5 联诊断 PNG 绘图器；
- 经领域标定的检测阈值规则库（`rules`）与企业 CI KPI 门禁参数（如 `--assert-kpi`）；
- 客户专有 `.msg` / `.proto` 消息适配器。

---

## 环境要求与安装

- **Python**: `>= 3.10`（已在 Python `3.10.12` 与 `3.13.17` 洁净环境验证通过）
- **包管理工具**: 推荐使用 [`uv`](https://docs.astral.sh/uv/)（根目录附带 `uv.lock`）
- **ROS 2 运行时**: **无依赖**

> **提示**
> 1. 默认 Nav2 与 Autoware 话题映射已随 wheel 内置打包（`pnc_diag._configs`）：无论采用源码仓库 `uv sync` / `pip install -e .`，还是独立目录 `pip install <wheel>` 安装，不传 `--map-config` 即自动加载内置 Nav2 默认配置；传 `--map-config autoware` 可直接切换内置 Autoware 配置，或传入自定义 YAML 文件路径。
> 2. 运行 `python -m fault_sim_kit.generator` 时，在源码仓库内默认输出至 `<repo>/data/golden/`；在独立非源码环境安装运行时默认输出至当前工作目录 `./data/golden/`，亦可通过 `--out <目录>` 指定输出位置。
> 3. 若宿主机终端已 `source` ROS 2（如 Humble），建议前缀 `env -u PYTHONPATH` 以避免系统 Python 路径污染虚拟环境。

```bash
# 1. 安装依赖
env -u PYTHONPATH uv sync

# 2. 检查 CLI
env -u PYTHONPATH uv run pnc-diag --version
```

---

## 可复现合成示例与测试验证

发行包不预打包约 47 MB 的 `.mcap` 二进制示例数据。可运行测试套件，或在本地生成 `data/golden/` 示例录包用于交互体验：

```bash
# 1. 代码风格检查与全套单元测试（不依赖预先存在 data/golden/）
env -u PYTHONPATH uv run ruff check .
env -u PYTHONPATH uv run pytest

# 2. 生成全部 10 个合成 MCAP 示例录包至 data/golden/
env -u PYTHONPATH uv run python -m fault_sim_kit.generator
```

---

## CLI 使用方式

本版本 `pnc-diag` 提供 `inspect` 与 `preview` 两个子命令：

```bash
# 1. 零反序列化时序体检、角色覆盖率与头文件时间戳中性观测
env -u PYTHONPATH uv run pnc-diag inspect data/golden/case_01_tf_jump

# 2. 缺传感器 / 零时间戳反例体检
env -u PYTHONPATH uv run pnc-diag inspect data/golden/case_08_missing_imu_and_scan
env -u PYTHONPATH uv run pnc-diag inspect data/golden/case_09_invalid_zero_stamps --json

# 3. 抽取并映射标准化物理信号样本（控制上下文长度）
env -u PYTHONPATH uv run pnc-diag preview data/golden/nav_baseline --role odom_pose --limit 3
env -u PYTHONPATH uv run pnc-diag preview data/golden/case_01_tf_jump --topic /amcl_pose --t-start 44.5 --t-end 46.0 --json

# 4. 使用内置 Autoware 映射预设或自定义映射文件检查录制包
env -u PYTHONPATH uv run pnc-diag inspect /path/to/autoware_bag --map-config autoware
env -u PYTHONPATH uv run pnc-diag inspect /path/to/autoware_bag --map-config configs/topic_map.autoware.yaml
```

---

## MCP Server 与 Skill 使用方式

### `pnc-diag-mcp` stdio 工具列表

| 工具名 | 参数 | 返回内容 |
|---|---|---|
| `ping` | 无 | 服务状态、`version`、`edition`（`open-source-thin`） |
| `inspect_bag_summary` | `bag_path`, `map_config` | 存储格式、总时长、可用/缺失角色、`clock_comparability`、告警与各话题时序体检统计 |
| `sample_mapped_signals` | `bag_path`, `topic`, `role`, `map_config`, `limit`, `t_start_s`, `t_end_s` | 映射后的标准化物理样本预览（`limit` 为 `1..50`） |

### MCP 客户端配置示例

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

配套 Skill 位于 [.agents/skills/pnc-slam-diag/SKILL.md](.agents/skills/pnc-slam-diag/SKILL.md) 与 [.agents/skills/pnc-slam-diag/references/fault_patterns.md](.agents/skills/pnc-slam-diag/references/fault_patterns.md)。

---

## 数据与诊断局限性

1. **仅提供读取与体检基础层**：本开源版不计算定位/规划/控制量化异常指标，也不做多传感器根因定论；面对问题录包应结合人工复核或后端专业分析工具进一步确认。
2. **禁止从 `header - log_time` 推定同步**：`observed_header_minus_log_ms` 仅反映包头时间与写盘时间的差值，无法独自区分传感器时钟漂移与录包队列/传输延迟；本版本默认将 `clock_comparability` 标记为 `"unverified"`。
3. **合成数据仅为复现练习示例（非盲测基准）**：`fault_sim_kit` 生成的是简化运动学闭环合成数据，不代表真实传感器噪声、动力学特性或实车验收基线；仓库未预提交 `golden_truth.json` 与二进制录包，但本发行版包含 10 个合成场景定义与可离线生成 `golden_truth.json` 的代码（仓库已公开，任何人均可据此复现完整场景与真值），因此后续盲测或基准评估必须另用独立、未公开的数据集。
4. **只读离线安全**：不执行实车控制指令、参数修改或节点操作。

---

## 许可证与第三方致谢

- **开源许可证（`Apache-2.0`）**：本项目采用 **Apache License 2.0** 发布，详见 [LICENSE](LICENSE) 与 [NOTICE](NOTICE)。`Apache-2.0` 与核心依赖 `rosbags` 兼容，具备明确专利许可及再分发条款，允许第三方在遵守许可证条件的前提下商业使用与再分发已公开的代码本身，但不会为本公开部分保留独占性；因此差异化物理诊断内核、精确规则库与商业门禁保留在私有仓库中维护。
- **第三方依赖与源码署名核对**：完整依赖许可证及上游借鉴/启发来源核对见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)（已核查仅采用 MIT / BSD / Apache-2.0 等宽松许可依赖与设计，未包含 GPL 代码）。
