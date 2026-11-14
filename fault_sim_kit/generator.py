"""Closed-loop kinematic simulator and 5-fault Golden MCAP dataset generator."""

from __future__ import annotations

import json
import math
import shutil
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from pnc_diag.models.geometry import euler_to_quaternion, normalize_angle

TYPESTORE = get_typestore(Stores.LATEST)
BASE_STAMP_NS = 1_700_000_000_000_000_000


@dataclass(frozen=True, slots=True)
class FaultScenarioConfig:
    """Definition of a baseline, fault-injected, or negative-gate simulation scenario."""

    name: str
    fault_type: str
    root_cause_module: str
    description: str
    expected_verdict: str = "DIAGNOSED"
    acceptable_root_causes: tuple[str, ...] = ()
    forbidden_root_causes: tuple[str, ...] = ()
    causal_chain: dict[str, Any] = field(default_factory=dict)
    onset_s: float | None = None
    end_s: float | None = None
    duration_s: float = 60.0
    seed: int = 42
    params: dict[str, Any] = field(default_factory=dict)


DEFAULT_SCENARIOS: tuple[FaultScenarioConfig, ...] = (
    FaultScenarioConfig(
        name="nav_baseline",
        fault_type="none",
        root_cause_module="none",
        description="Healthy closed-loop navigation baseline with smooth tracking and consistent timing.",
        expected_verdict="HEALTHY",
        acceptable_root_causes=("none",),
        forbidden_root_causes=("localization", "control", "timing", "planner", "odometry"),
        duration_s=60.0,
        seed=100,
    ),
    FaultScenarioConfig(
        name="case_01_tf_jump",
        fault_type="localization_tf_jump",
        root_cause_module="localization",
        description="Sudden map->odom relocalization step jump at t=45.0s triggering downstream replan and steering transient.",
        expected_verdict="DIAGNOSED",
        acceptable_root_causes=("localization",),
        forbidden_root_causes=("odometry",),
        causal_chain={
            "upstream_event": {"module": "localization", "role": "tf", "onset_s": 45.0},
            "downstream_events": [
                {"module": "planning", "role": "local_plan", "expected_lag_ms_range": [20, 250]},
                {"module": "control", "role": "control_cmd", "expected_lag_ms_range": [20, 120]},
            ],
        },
        onset_s=45.0,
        end_s=60.0,
        duration_s=60.0,
        seed=101,
        params={"step_dx_m": 0.85, "step_dy_m": -0.62, "step_dyaw_rad": 0.28, "cov_spike": 1.25},
    ),
    FaultScenarioConfig(
        name="case_02_ctrl_delay",
        fault_type="control_loop_delay_oscillation",
        root_cause_module="control",
        description="220ms transport delay on control actuation inducing lateral snake oscillation (huolong).",
        expected_verdict="DIAGNOSED",
        acceptable_root_causes=("control",),
        forbidden_root_causes=("localization", "planner"),
        causal_chain={
            "upstream_event": {"module": "control", "role": "control_cmd", "onset_s": 15.0},
            "downstream_events": [
                {"module": "odometry", "role": "odom_pose", "expected_lag_ms_range": [180, 320]},
            ],
        },
        onset_s=15.0,
        end_s=55.0,
        duration_s=60.0,
        seed=102,
        params={"delay_ms": 220, "ctrl_yaw_gain": 3.4, "ctrl_lat_gain": 2.4},
    ),
    FaultScenarioConfig(
        name="case_03_time_desync",
        fault_type="sensor_clock_drift_and_drop",
        root_cause_module="timing",
        description="Chain 1: /scan +90ms header drift (t=20-55s); Chain 2: 500ms drop at t=35.0s degrades downstream AMCL.",
        expected_verdict="DIAGNOSED",
        acceptable_root_causes=("timing",),
        forbidden_root_causes=("localization", "odometry", "control"),
        causal_chain={
            "chains": [
                {
                    "name": "clock_drift",
                    "upstream_event": {
                        "module": "timing",
                        "role": "laser_scan",
                        "metric": "header_minus_log_ms",
                        "onset_s": 20.0,
                    },
                    "downstream_events": [],
                },
                {
                    "name": "frame_drop_cascade",
                    "upstream_event": {
                        "module": "timing",
                        "role": "laser_scan",
                        "metric": "frame_drop_gap_ms",
                        "onset_s": 35.0,
                    },
                    "downstream_events": [
                        {
                            "module": "localization",
                            "role": "localization_pose",
                            "metric": "cov_trace_spike",
                            "expected_lag_ms_range": [100, 300],
                        },
                    ],
                },
            ],
        },
        onset_s=20.0,
        end_s=55.0,
        duration_s=60.0,
        seed=103,
        params={
            "header_drift_ms": 90,
            "drop_start_s": 35.0,
            "drop_duration_s": 0.45,
            "downstream_loc_lag_s": 0.12,
        },
    ),
    FaultScenarioConfig(
        name="case_04_planner_stuck",
        fault_type="ghost_obstacle_recovery_brake",
        root_cause_module="perception",
        description="Ghost obstacle appears in /scan at t=30.0s (first domino) -> /plan freezes (+120ms) -> BT recovery (+160ms) -> brake (+200ms).",
        expected_verdict="DIAGNOSED",
        acceptable_root_causes=("perception",),
        forbidden_root_causes=("localization", "odometry", "planner", "control"),
        causal_chain={
            "upstream_event": {"module": "perception", "role": "laser_scan", "onset_s": 30.0},
            "downstream_events": [
                {"module": "planner", "role": "local_plan", "expected_lag_ms_range": [80, 220]},
                {"module": "planner", "role": "diagnostics", "expected_lag_ms_range": [120, 450]},
                {"module": "control", "role": "control_cmd", "expected_lag_ms_range": [160, 350]},
            ],
        },
        onset_s=30.0,
        end_s=42.0,
        duration_s=60.0,
        seed=104,
        params={
            "ghost_range_m": 1.15,
            "plan_lag_s": 0.12,
            "diag_lag_s": 0.16,
            "cmd_brake_lag_s": 0.20,
            "recovery_cycles": 3,
        },
    ),
    FaultScenarioConfig(
        name="case_05_odom_slip",
        fault_type="wheel_odometry_turning_slip",
        root_cause_module="odometry",
        description="Wheel odometry yaw/linear scale slips (0.78x) during curved segments t=18-38s, diverging from IMU and AMCL.",
        expected_verdict="DIAGNOSED",
        acceptable_root_causes=("odometry",),
        forbidden_root_causes=("localization",),
        causal_chain={
            "upstream_event": {"module": "odometry", "role": "odom_pose", "onset_s": 18.0},
            "downstream_events": [
                {"module": "localization", "role": "tf", "expected_lag_ms_range": [0, 100]},
            ],
        },
        onset_s=18.0,
        end_s=38.0,
        duration_s=60.0,
        seed=105,
        params={"yaw_scale_factor": 0.78, "linear_scale_factor": 0.92},
    ),
    FaultScenarioConfig(
        name="case_06_healthy_sharp_turn",
        fault_type="none",
        root_cause_module="none",
        description="Healthy dynamic sharp turn and comfortable speed reduction; tests zero false positives.",
        expected_verdict="HEALTHY",
        acceptable_root_causes=("none",),
        forbidden_root_causes=("localization", "control", "timing", "planner", "odometry"),
        onset_s=22.0,
        end_s=36.0,
        duration_s=60.0,
        seed=106,
        params={"sharp_turn_speed_profile": True},
    ),
    FaultScenarioConfig(
        name="case_07_mixed_slip_plus_ctrl_delay",
        fault_type="mixed_slip_and_control_delay",
        root_cause_module="odometry",
        description="Overlapping wheel slip (t=18-38s) and 200ms control lag (t=32-54s); tests multi-root-cause attribution.",
        expected_verdict="DIAGNOSED",
        acceptable_root_causes=("odometry", "control"),
        causal_chain={
            "upstream_event": {"module": "odometry", "role": "odom_pose", "onset_s": 18.0},
            "downstream_events": [
                {"module": "control", "role": "control_cmd", "expected_lag_ms_range": [180, 320]},
            ],
        },
        onset_s=18.0,
        end_s=54.0,
        duration_s=60.0,
        seed=107,
        params={
            "slip_start_s": 18.0,
            "slip_end_s": 38.0,
            "yaw_scale_factor": 0.78,
            "linear_scale_factor": 0.92,
            "delay_start_s": 32.0,
            "delay_end_s": 54.0,
            "delay_ms": 200,
            "ctrl_yaw_gain": 3.2,
            "ctrl_lat_gain": 2.2,
        },
    ),
    FaultScenarioConfig(
        name="case_08_missing_imu_and_scan",
        fault_type="wheel_odometry_turning_slip",
        root_cause_module="insufficient_evidence",
        description="Identical map->odom drift as odom_slip but /imu/data and /scan are omitted; forbids guessing loc vs slip.",
        expected_verdict="INSUFFICIENT_EVIDENCE",
        acceptable_root_causes=("insufficient_evidence",),
        forbidden_root_causes=("localization", "odometry"),
        onset_s=18.0,
        end_s=38.0,
        duration_s=60.0,
        seed=108,
        params={
            "yaw_scale_factor": 0.78,
            "linear_scale_factor": 0.92,
            "omit_topics": ["/imu/data", "/scan"],
        },
    ),
    FaultScenarioConfig(
        name="case_09_invalid_zero_stamps",
        fault_type="localization_tf_jump",
        root_cause_module="indeterminate_order",
        description="Zeroed header.stamp on /plan & /amcl_pose during fault transient; forbids using fallback log_time for sub-100ms causality.",
        expected_verdict="INDETERMINATE_ORDER",
        acceptable_root_causes=("indeterminate_order",),
        onset_s=45.0,
        end_s=60.0,
        duration_s=60.0,
        seed=109,
        params={
            "step_dx_m": 0.85,
            "step_dy_m": -0.62,
            "step_dyaw_rad": 0.28,
            "cov_spike": 1.25,
            "zero_stamp_topics": ["/plan", "/amcl_pose"],
        },
    ),
)


def _ros_time(stamp_ns: int) -> Any:
    Time = TYPESTORE.types["builtin_interfaces/msg/Time"]
    return Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


def _ros_header(stamp_ns: int, frame_id: str) -> Any:
    Header = TYPESTORE.types["std_msgs/msg/Header"]
    return Header(stamp=_ros_time(stamp_ns), frame_id=frame_id)


def _build_reference_track(total_length_m: float = 95.0, ds: float = 0.05) -> np.ndarray:
    """Generate dense (N, 3) reference path [x, y, yaw] with straight and curved segments."""
    n = int(total_length_m / ds) + 1
    pts = np.zeros((n, 3), dtype=np.float64)
    x, y, yaw = 0.0, 0.0, 0.0
    for i in range(n):
        s = i * ds
        if 12.0 <= s < 24.0:
            kappa = 0.09
        elif 24.0 <= s < 38.0:
            kappa = -0.10
        elif 44.0 <= s < 58.0:
            kappa = 0.14 * math.sin(2.0 * math.pi * (s - 44.0) / 14.0)
        elif 64.0 <= s < 78.0:
            kappa = -0.12
        else:
            kappa = 0.0
        pts[i] = (x, y, normalize_angle(yaw))
        x += ds * math.cos(yaw)
        y += ds * math.sin(yaw)
        yaw = normalize_angle(yaw + kappa * ds)
    return pts


def _closest_track_index(ref: np.ndarray, x: float, y: float, hint_idx: int) -> int:
    lo = max(0, hint_idx - 40)
    hi = min(len(ref), hint_idx + 120)
    sub = ref[lo:hi, :2]
    d2 = (sub[:, 0] - x) ** 2 + (sub[:, 1] - y) ** 2
    return lo + int(np.argmin(d2))


def _serialize_odom(
    stamp_ns: int,
    frame_id: str,
    child_frame_id: str,
    x: float,
    y: float,
    yaw: float,
    vx: float,
    wz: float,
    cov_xy: float = 0.015,
    cov_yaw: float = 0.005,
) -> bytes:
    Odometry = TYPESTORE.types["nav_msgs/msg/Odometry"]
    PoseWithCovariance = TYPESTORE.types["geometry_msgs/msg/PoseWithCovariance"]
    Pose = TYPESTORE.types["geometry_msgs/msg/Pose"]
    Point = TYPESTORE.types["geometry_msgs/msg/Point"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]
    TwistWithCovariance = TYPESTORE.types["geometry_msgs/msg/TwistWithCovariance"]
    Twist = TYPESTORE.types["geometry_msgs/msg/Twist"]
    Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]

    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
    cov_pose = np.zeros(36, dtype=np.float64)
    cov_pose[0] = cov_xy
    cov_pose[7] = cov_xy
    cov_pose[35] = cov_yaw
    msg = Odometry(
        header=_ros_header(stamp_ns, frame_id),
        child_frame_id=child_frame_id,
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            ),
            covariance=cov_pose,
        ),
        twist=TwistWithCovariance(
            twist=Twist(
                linear=Vector3(x=vx, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=wz),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    return bytes(TYPESTORE.serialize_cdr(msg, "nav_msgs/msg/Odometry"))


def _serialize_pose_cov(
    stamp_ns: int, x: float, y: float, yaw: float, cov_xy: float, cov_yaw: float
) -> bytes:
    PoseWithCovarianceStamped = TYPESTORE.types["geometry_msgs/msg/PoseWithCovarianceStamped"]
    PoseWithCovariance = TYPESTORE.types["geometry_msgs/msg/PoseWithCovariance"]
    Pose = TYPESTORE.types["geometry_msgs/msg/Pose"]
    Point = TYPESTORE.types["geometry_msgs/msg/Point"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]
    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
    cov = np.zeros(36, dtype=np.float64)
    cov[0] = cov_xy
    cov[7] = cov_xy
    cov[35] = cov_yaw
    msg = PoseWithCovarianceStamped(
        header=_ros_header(stamp_ns, "map"),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            ),
            covariance=cov,
        ),
    )
    return bytes(TYPESTORE.serialize_cdr(msg, "geometry_msgs/msg/PoseWithCovarianceStamped"))


def _serialize_tf(
    stamp_ns: int,
    map_odom: tuple[float, float, float],
    odom_base: tuple[float, float, float],
) -> bytes:
    TFMessage = TYPESTORE.types["tf2_msgs/msg/TFMessage"]
    TransformStamped = TYPESTORE.types["geometry_msgs/msg/TransformStamped"]
    Transform = TYPESTORE.types["geometry_msgs/msg/Transform"]
    Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]

    transforms = []
    for parent, child, (tx, ty, tyaw) in (
        ("map", "odom", map_odom),
        ("odom", "base_link", odom_base),
    ):
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, tyaw)
        transforms.append(
            TransformStamped(
                header=_ros_header(stamp_ns, parent),
                child_frame_id=child,
                transform=Transform(
                    translation=Vector3(x=tx, y=ty, z=0.0),
                    rotation=Quaternion(x=qx, y=qy, z=qz, w=qw),
                ),
            )
        )
    return bytes(TYPESTORE.serialize_cdr(TFMessage(transforms=transforms), "tf2_msgs/msg/TFMessage"))


def _serialize_path(stamp_ns: int, waypoints: np.ndarray) -> bytes:
    PathMsg = TYPESTORE.types["nav_msgs/msg/Path"]
    PoseStamped = TYPESTORE.types["geometry_msgs/msg/PoseStamped"]
    Pose = TYPESTORE.types["geometry_msgs/msg/Pose"]
    Point = TYPESTORE.types["geometry_msgs/msg/Point"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]

    poses = []
    for px, py, pyaw in waypoints:
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, float(pyaw))
        poses.append(
            PoseStamped(
                header=_ros_header(stamp_ns, "map"),
                pose=Pose(
                    position=Point(x=float(px), y=float(py), z=0.0),
                    orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
                ),
            )
        )
    return bytes(TYPESTORE.serialize_cdr(PathMsg(header=_ros_header(stamp_ns, "map"), poses=poses), "nav_msgs/msg/Path"))


def _serialize_cmd_vel(vx: float, wz: float) -> bytes:
    Twist = TYPESTORE.types["geometry_msgs/msg/Twist"]
    Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    msg = Twist(linear=Vector3(x=vx, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=wz))
    return bytes(TYPESTORE.serialize_cdr(msg, "geometry_msgs/msg/Twist"))


def _serialize_imu(stamp_ns: int, wz: float, ax: float, ay: float) -> bytes:
    Imu = TYPESTORE.types["sensor_msgs/msg/Imu"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]
    Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    cov = np.zeros(9, dtype=np.float64)
    msg = Imu(
        header=_ros_header(stamp_ns, "imu_link"),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=cov,
        angular_velocity=Vector3(x=0.0, y=0.0, z=wz),
        angular_velocity_covariance=cov,
        linear_acceleration=Vector3(x=ax, y=ay, z=9.81),
        linear_acceleration_covariance=cov,
    )
    return bytes(TYPESTORE.serialize_cdr(msg, "sensor_msgs/msg/Imu"))


def _serialize_scan(stamp_ns: int, rng: np.random.Generator, ghost_range_m: float | None) -> bytes:
    LaserScan = TYPESTORE.types["sensor_msgs/msg/LaserScan"]
    num_beams = 72
    angles = np.linspace(-math.pi, math.pi, num_beams, endpoint=False)
    # Corridor walls +/- 4.0m with small noise
    ranges = np.clip(4.0 / np.maximum(np.abs(np.sin(angles)), 0.35), 1.5, 12.0)
    ranges = (ranges + rng.normal(0.0, 0.02, size=num_beams)).astype(np.float32)
    if ghost_range_m is not None:
        front_mask = np.abs(angles) <= 0.25
        ranges[front_mask] = np.float32(ghost_range_m)

    msg = LaserScan(
        header=_ros_header(stamp_ns, "laser_frame"),
        angle_min=-math.pi,
        angle_max=math.pi - (2.0 * math.pi / num_beams),
        angle_increment=2.0 * math.pi / num_beams,
        time_increment=0.001,
        scan_time=0.1,
        range_min=0.15,
        range_max=25.0,
        ranges=ranges,
        intensities=np.ones(num_beams, dtype=np.float32) * 100.0,
    )
    return bytes(TYPESTORE.serialize_cdr(msg, "sensor_msgs/msg/LaserScan"))


def _serialize_diagnostics(
    stamp_ns: int, level: int, name: str, message: str, kv_pairs: dict[str, str]
) -> bytes:
    DiagnosticArray = TYPESTORE.types["diagnostic_msgs/msg/DiagnosticArray"]
    DiagnosticStatus = TYPESTORE.types["diagnostic_msgs/msg/DiagnosticStatus"]
    KeyValue = TYPESTORE.types["diagnostic_msgs/msg/KeyValue"]
    msg = DiagnosticArray(
        header=_ros_header(stamp_ns, ""),
        status=[
            DiagnosticStatus(
                level=level,
                name=name,
                message=message,
                hardware_id="nav2_stack",
                values=[KeyValue(key=k, value=v) for k, v in kv_pairs.items()],
            )
        ],
    )
    return bytes(TYPESTORE.serialize_cdr(msg, "diagnostic_msgs/msg/DiagnosticArray"))


def _compose_2d(
    parent: tuple[float, float, float], child: tuple[float, float, float]
) -> tuple[float, float, float]:
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, normalize_angle(pyaw + cyaw)


def _relative_map_to_odom(
    map_pose: tuple[float, float, float], odom_pose: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Compute map->odom transform T such that compose(T, odom_pose) == map_pose."""
    mx, my, myaw = map_pose
    ox, oy, oyaw = odom_pose
    dyaw = normalize_angle(myaw - oyaw)
    c, s = math.cos(dyaw), math.sin(dyaw)
    tx = mx - (c * ox - s * oy)
    ty = my - (s * ox + c * oy)
    return tx, ty, dyaw


def simulate_scenario(scenario: FaultScenarioConfig, output_root: Path) -> Path:
    """Simulate closed-loop vehicle motion under a fault scenario and write an MCAP bag."""
    bag_dir = output_root / scenario.name
    if bag_dir.exists():
        shutil.rmtree(bag_dir)

    rng = np.random.default_rng(scenario.seed)
    ref_track = _build_reference_track()

    dt = 0.02  # 50 Hz physics & odom/tf/imu
    total_steps = round(scenario.duration_s / dt)
    delay_ms = int(scenario.params.get("delay_ms", 0))
    delay_ticks = max(0, round((delay_ms / 1000.0) / dt))

    cmd_queue: deque[tuple[float, float]] = deque([(1.15, 0.0)] * max(1, delay_ticks + 1))

    # True robot pose in map frame, and wheel odom dead-reckoned pose in odom frame
    x_true, y_true, yaw_true = 0.0, 0.0, 0.0
    x_odom, y_odom, yaw_odom = 0.0, 0.0, 0.0
    vx_true, wz_true = 0.0, 0.0
    vx_prev = 0.0
    track_idx = 0
    active_tf_jump = (0.0, 0.0, 0.0)
    last_recovery_step = -1

    omit_topics = set(scenario.params.get("omit_topics", []))
    zero_stamp_topics = set(scenario.params.get("zero_stamp_topics", []))

    with Writer(bag_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        c_odom = (
            None
            if "/odom" in omit_topics
            else writer.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=TYPESTORE)
        )
        c_amcl = (
            None
            if "/amcl_pose" in omit_topics
            else writer.add_connection(
                "/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", typestore=TYPESTORE
            )
        )
        c_tf = (
            None
            if "/tf" in omit_topics
            else writer.add_connection("/tf", "tf2_msgs/msg/TFMessage", typestore=TYPESTORE)
        )
        c_plan = (
            None
            if "/plan" in omit_topics
            else writer.add_connection("/plan", "nav_msgs/msg/Path", typestore=TYPESTORE)
        )
        c_cmd = (
            None
            if "/cmd_vel" in omit_topics
            else writer.add_connection("/cmd_vel", "geometry_msgs/msg/Twist", typestore=TYPESTORE)
        )
        c_imu = (
            None
            if "/imu/data" in omit_topics
            else writer.add_connection("/imu/data", "sensor_msgs/msg/Imu", typestore=TYPESTORE)
        )
        c_scan = (
            None
            if "/scan" in omit_topics
            else writer.add_connection("/scan", "sensor_msgs/msg/LaserScan", typestore=TYPESTORE)
        )
        c_diag = (
            None
            if "/diagnostics" in omit_topics
            else writer.add_connection(
                "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", typestore=TYPESTORE
            )
        )

        v_cmd, w_cmd = 1.2, 0.0
        for step in range(total_steps):
            t_s = step * dt
            log_ns = BASE_STAMP_NS + round(t_s * 1_000_000_000)
            in_fault = (
                scenario.onset_s is not None
                and scenario.end_s is not None
                and scenario.onset_s <= t_s <= scenario.end_s
            )

            slip_active = in_fault and scenario.fault_type == "wheel_odometry_turning_slip"
            delay_active = in_fault and scenario.fault_type == "control_loop_delay_oscillation"
            if scenario.fault_type == "mixed_slip_and_control_delay":
                slip_active = float(scenario.params["slip_start_s"]) <= t_s <= float(
                    scenario.params["slip_end_s"]
                )
                delay_active = float(scenario.params["delay_start_s"]) <= t_s <= float(
                    scenario.params["delay_end_s"]
                )

            # Localization estimate observed by planner/controller from prior state + TF jump
            if scenario.fault_type == "localization_tf_jump" and t_s >= (scenario.onset_s or 999.0):
                active_tf_jump = (
                    float(scenario.params["step_dx_m"]),
                    float(scenario.params["step_dy_m"]),
                    float(scenario.params["step_dyaw_rad"]),
                )

            x_est, y_est, yaw_est = _compose_2d(active_tf_jump, (x_true, y_true, yaw_true))
            track_idx = _closest_track_index(ref_track, x_est, y_est, track_idx)

            lookahead_idx = min(len(ref_track) - 1, track_idx + 18)
            tx, ty, tyaw = ref_track[lookahead_idx]
            dx_map = tx - x_est
            dy_map = ty - y_est
            cos_y, sin_y = math.cos(yaw_est), math.sin(yaw_est)
            cte_local = -sin_y * dx_map + cos_y * dy_map
            head_err = normalize_angle(tyaw - yaw_est)

            k_lat = 1.15
            k_yaw = 1.9
            v_target = 1.20
            if scenario.params.get("sharp_turn_speed_profile"):
                v_target = 0.65 if (scenario.onset_s or 22.0) <= t_s <= (scenario.end_s or 36.0) else 1.45

            if delay_active:
                k_lat = float(scenario.params["ctrl_lat_gain"])
                k_yaw = float(scenario.params["ctrl_yaw_gain"])

            if scenario.fault_type == "ghost_obstacle_recovery_brake":
                onset = scenario.onset_s or 30.0
                end_s = scenario.end_s or 42.0
                cmd_onset = onset + float(scenario.params.get("cmd_brake_lag_s", 0.20))
                if cmd_onset <= t_s <= end_s:
                    v_cmd = 0.0
                    w_cmd = (
                        0.0
                        if t_s < (cmd_onset + 2.0)
                        else 0.15 * math.sin(2.0 * (t_s - (cmd_onset + 2.0)))
                    )
                else:
                    v_cmd = v_target * max(0.45, math.cos(min(abs(head_err), 1.2)))
                    w_cmd = float(np.clip(k_yaw * head_err + k_lat * cte_local, -1.6, 1.6))
            else:
                v_cmd = v_target * max(0.45, math.cos(min(abs(head_err), 1.2)))
                w_cmd = float(np.clip(k_yaw * head_err + k_lat * cte_local, -1.6, 1.6))

            if delay_active:
                cmd_queue.append((v_cmd, w_cmd))
                v_applied, w_applied = cmd_queue.popleft()
            else:
                v_applied, w_applied = v_cmd, w_cmd
                cmd_queue.clear()
                cmd_queue.extend([(v_cmd, w_cmd)] * max(1, delay_ticks + 1))

            # First-order actuator dynamics
            vx_true += (v_applied - vx_true) * min(1.0, dt / 0.12)
            wz_true += (w_applied - wz_true) * min(1.0, dt / 0.10)
            ax_true = (vx_true - vx_prev) / dt
            vx_prev = vx_true

            # Integrate true pose in map frame
            x_true += vx_true * math.cos(yaw_true) * dt
            y_true += vx_true * math.sin(yaw_true) * dt
            yaw_true = normalize_angle(yaw_true + wz_true * dt)

            # Integrate wheel odometry in odom frame (with slip fault if active)
            vx_odom = vx_true
            wz_odom = wz_true
            if slip_active and abs(wz_true) > 0.06:
                vx_odom *= float(scenario.params["linear_scale_factor"])
                wz_odom *= float(scenario.params["yaw_scale_factor"])
            x_odom += vx_odom * math.cos(yaw_odom) * dt
            y_odom += vx_odom * math.sin(yaw_odom) * dt
            yaw_odom = normalize_angle(yaw_odom + wz_odom * dt)

            # Update localized map->base estimate and compute exact map->odom TF
            x_est, y_est, yaw_est = _compose_2d(active_tf_jump, (x_true, y_true, yaw_true))
            map_to_odom = _relative_map_to_odom((x_est, y_est, yaw_est), (x_odom, y_odom, yaw_odom))

            # 50 Hz: /odom, /tf, /imu/data
            if c_odom is not None:
                writer.write(
                    c_odom,
                    log_ns,
                    _serialize_odom(
                        0 if "/odom" in zero_stamp_topics else log_ns,
                        "odom",
                        "base_link",
                        x_odom,
                        y_odom,
                        yaw_odom,
                        vx_odom + float(rng.normal(0.0, 0.005)),
                        wz_odom + float(rng.normal(0.0, 0.003)),
                    ),
                )
            if c_tf is not None:
                writer.write(
                    c_tf,
                    log_ns,
                    _serialize_tf(
                        0 if "/tf" in zero_stamp_topics else log_ns,
                        map_to_odom,
                        (x_odom, y_odom, yaw_odom),
                    ),
                )
            if c_imu is not None:
                ay_imu = vx_true * wz_true + float(rng.normal(0.0, 0.02))
                writer.write(
                    c_imu,
                    log_ns,
                    _serialize_imu(
                        0 if "/imu/data" in zero_stamp_topics else log_ns,
                        wz_true + float(rng.normal(0.0, 0.004)),
                        ax_true + float(rng.normal(0.0, 0.02)),
                        ay_imu,
                    ),
                )

            # 25 Hz: /cmd_vel (every 2 ticks = 40ms)
            if c_cmd is not None and step % 2 == 0:
                writer.write(c_cmd, log_ns, _serialize_cmd_vel(v_cmd, w_cmd))

            # 10 Hz: /amcl_pose and /scan (every 5 ticks = 100ms)
            if step % 5 == 0:
                cov_xy = 0.02
                cov_yaw = 0.01
                amcl_jitter = 0.015
                if (
                    scenario.fault_type == "localization_tf_jump"
                    and scenario.onset_s is not None
                    and 0.0 <= (t_s - scenario.onset_s) < 1.5
                ):
                    cov_xy = float(scenario.params["cov_spike"])
                    cov_yaw = 0.35
                elif scenario.fault_type == "sensor_clock_drift_and_drop":
                    lag_s = float(scenario.params.get("downstream_loc_lag_s", 0.12))
                    drop_start = float(scenario.params["drop_start_s"])
                    if (drop_start + lag_s) <= t_s <= (drop_start + 1.5):
                        cov_xy = 0.65
                        cov_yaw = 0.18
                        amcl_jitter = 0.08

                if c_amcl is not None:
                    amcl_x = x_est + float(rng.normal(0.0, amcl_jitter))
                    amcl_y = y_est + float(rng.normal(0.0, amcl_jitter))
                    amcl_yaw = normalize_angle(yaw_est + float(rng.normal(0.0, 0.008)))
                    writer.write(
                        c_amcl,
                        log_ns,
                        _serialize_pose_cov(
                            0 if "/amcl_pose" in zero_stamp_topics else log_ns,
                            amcl_x,
                            amcl_y,
                            amcl_yaw,
                            cov_xy,
                            cov_yaw,
                        ),
                    )

                if c_scan is not None:
                    skip_scan = False
                    scan_stamp_ns = 0 if "/scan" in zero_stamp_topics else log_ns
                    if scenario.fault_type == "sensor_clock_drift_and_drop" and in_fault:
                        drop_start = float(scenario.params["drop_start_s"])
                        drop_end = drop_start + float(scenario.params["drop_duration_s"])
                        if drop_start <= t_s <= drop_end:
                            skip_scan = True
                        if scan_stamp_ns > 0:
                            scan_stamp_ns += int(scenario.params["header_drift_ms"]) * 1_000_000

                    if not skip_scan:
                        ghost_r = (
                            float(scenario.params["ghost_range_m"])
                            if (scenario.fault_type == "ghost_obstacle_recovery_brake" and in_fault)
                            else None
                        )
                        writer.write(c_scan, log_ns, _serialize_scan(scan_stamp_ns, rng, ghost_r))

            # 5 Hz: /plan (every 10 ticks = 200ms; staggered response for case_04)
            if c_plan is not None and step % 10 == 0:
                plan_frozen = False
                if scenario.fault_type == "ghost_obstacle_recovery_brake":
                    plan_onset = (scenario.onset_s or 30.0) + float(
                        scenario.params.get("plan_lag_s", 0.12)
                    )
                    plan_frozen = plan_onset <= t_s <= (scenario.end_s or 42.0)

                if plan_frozen:
                    waypoints = np.array([[x_est, y_est, yaw_est]], dtype=np.float64)
                else:
                    end_idx = min(len(ref_track), track_idx + 40)
                    stride = max(1, (end_idx - track_idx) // 12)
                    waypoints = ref_track[track_idx:end_idx:stride]
                    if len(waypoints) == 0:
                        waypoints = ref_track[-1:]
                writer.write(
                    c_plan,
                    log_ns,
                    _serialize_path(0 if "/plan" in zero_stamp_topics else log_ns, waypoints),
                )

            # 2 Hz: /diagnostics (every 25 ticks = 500ms)
            if c_diag is not None and step % 25 == 0:
                diag_active = False
                if scenario.fault_type == "ghost_obstacle_recovery_brake":
                    diag_onset = (scenario.onset_s or 30.0) + float(
                        scenario.params.get("diag_lag_s", 0.16)
                    )
                    diag_active = diag_onset <= t_s <= (scenario.end_s or 42.0)

                if diag_active:
                    cycle = min(3, max(1, int((t_s - (scenario.onset_s or 30.0)) // 3.0) + 1))
                    if cycle != last_recovery_step:
                        last_recovery_step = cycle
                    writer.write(
                        c_diag,
                        log_ns,
                        _serialize_diagnostics(
                            log_ns,
                            2,
                            "bt_navigator",
                            "Obstacle deadlock detected: ClearEntireCostmap & BackUp recovery triggered",
                            {"recovery_count": str(cycle), "state": "RECOVERY_ACTIVE"},
                        ),
                    )
                else:
                    writer.write(
                        c_diag,
                        log_ns,
                        _serialize_diagnostics(
                            log_ns,
                            0,
                            "bt_navigator",
                            "Navigating nominally",
                            {"recovery_count": "0", "state": "FOLLOW_PATH"},
                        ),
                    )

    return bag_dir


def generate_golden_dataset(
    output_dir: str | Path,
    scenarios: tuple[FaultScenarioConfig, ...] = DEFAULT_SCENARIOS,
    duration_override_s: float | None = None,
) -> Path:
    """Generate the complete Golden MCAP dataset and golden_truth.json manifest."""
    out_root = Path(output_dir)
    out_root.mkdir(parents=True, exist_ok=True)

    manifest: dict[str, Any] = {
        "schema_version": 2,
        "base_stamp_ns": BASE_STAMP_NS,
        "cases": [],
    }
    for cfg in scenarios:
        effective = (
            FaultScenarioConfig(
                name=cfg.name,
                fault_type=cfg.fault_type,
                root_cause_module=cfg.root_cause_module,
                description=cfg.description,
                expected_verdict=cfg.expected_verdict,
                acceptable_root_causes=cfg.acceptable_root_causes,
                forbidden_root_causes=cfg.forbidden_root_causes,
                causal_chain=dict(cfg.causal_chain),
                onset_s=(
                    min(cfg.onset_s, duration_override_s * 0.5)
                    if (cfg.onset_s is not None and duration_override_s is not None)
                    else cfg.onset_s
                ),
                end_s=(
                    min(cfg.end_s, duration_override_s * 0.9)
                    if (cfg.end_s is not None and duration_override_s is not None)
                    else cfg.end_s
                ),
                duration_s=duration_override_s if duration_override_s is not None else cfg.duration_s,
                seed=cfg.seed,
                params=dict(cfg.params),
            )
            if duration_override_s is not None
            else cfg
        )
        bag_dir = simulate_scenario(effective, out_root)
        mcap_files = sorted(p.name for p in bag_dir.glob("*.mcap"))
        entry = asdict(effective)
        entry["bag_dir"] = effective.name
        entry["mcap_file"] = f"{effective.name}/{mcap_files[0]}" if mcap_files else None
        manifest["cases"].append(entry)

    truth_path = out_root / "golden_truth.json"
    truth_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return truth_path


def main() -> None:
    default_out = Path(__file__).resolve().parents[1] / "data" / "golden"
    truth = generate_golden_dataset(default_out)
    print(f"Generated golden dataset manifest at: {truth}")


if __name__ == "__main__":
    main()
