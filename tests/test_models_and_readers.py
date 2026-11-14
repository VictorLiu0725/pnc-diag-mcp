"""Unit and integration tests for signal models, readers, and topic_map mapping."""

from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from pnc_diag.models import (
    ControlCmdSample,
    DiagnosticEvent,
    ImuSample,
    PathSample,
    PoseSample,
    euler_to_quaternion,
    normalize_angle,
    quaternion_to_euler,
)
from pnc_diag.readers import (
    BagDiscoveryError,
    SignalMapping,
    discover_bag,
    extract_dot_path,
    iter_bag_messages,
    load_topic_map,
    map_message,
    scan_bag_timing,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
TYPESTORE = get_typestore(Stores.LATEST)


def _ros_time(stamp_ns: int):
    Time = TYPESTORE.types["builtin_interfaces/msg/Time"]
    sec = stamp_ns // 1_000_000_000
    nanosec = stamp_ns % 1_000_000_000
    return Time(sec=sec, nanosec=nanosec)


def _header(stamp_ns: int, frame_id: str = "map"):
    Header = TYPESTORE.types["std_msgs/msg/Header"]
    return Header(stamp=_ros_time(stamp_ns), frame_id=frame_id)


def _make_odom_msg(
    stamp_ns: int,
    x: float,
    y: float,
    yaw: float,
    vx: float = 1.0,
    wz: float = 0.1,
    cov_xx: float = 0.04,
):
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
    cov_pose[0] = cov_xx
    cov_pose[7] = 0.09
    cov_pose[35] = 0.02
    cov_twist = np.zeros(36, dtype=np.float64)
    return Odometry(
        header=_header(stamp_ns, "odom"),
        child_frame_id="base_link",
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
            covariance=cov_twist,
        ),
    )


def _make_twist_msg(vx: float, wz: float):
    Twist = TYPESTORE.types["geometry_msgs/msg/Twist"]
    Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    return Twist(
        linear=Vector3(x=vx, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=wz),
    )


def _make_tf_msg(stamp_ns: int, parent: str, child: str, x: float, y: float, yaw: float):
    TFMessage = TYPESTORE.types["tf2_msgs/msg/TFMessage"]
    TransformStamped = TYPESTORE.types["geometry_msgs/msg/TransformStamped"]
    Transform = TYPESTORE.types["geometry_msgs/msg/Transform"]
    Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]
    qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
    return TFMessage(
        transforms=[
            TransformStamped(
                header=_header(stamp_ns, parent),
                child_frame_id=child,
                transform=Transform(
                    translation=Vector3(x=x, y=y, z=0.0),
                    rotation=Quaternion(x=qx, y=qy, z=qz, w=qw),
                ),
            )
        ]
    )


def _make_path_msg(stamp_ns: int, points: list[tuple[float, float, float]]):
    PathMsg = TYPESTORE.types["nav_msgs/msg/Path"]
    PoseStamped = TYPESTORE.types["geometry_msgs/msg/PoseStamped"]
    Pose = TYPESTORE.types["geometry_msgs/msg/Pose"]
    Point = TYPESTORE.types["geometry_msgs/msg/Point"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]
    poses = []
    for x, y, yaw in points:
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, yaw)
        poses.append(
            PoseStamped(
                header=_header(stamp_ns, "map"),
                pose=Pose(
                    position=Point(x=x, y=y, z=0.0),
                    orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
                ),
            )
        )
    return PathMsg(header=_header(stamp_ns, "map"), poses=poses)


def _make_imu_msg(stamp_ns: int, wz: float, ay: float):
    Imu = TYPESTORE.types["sensor_msgs/msg/Imu"]
    Quaternion = TYPESTORE.types["geometry_msgs/msg/Quaternion"]
    Vector3 = TYPESTORE.types["geometry_msgs/msg/Vector3"]
    cov = np.zeros(9, dtype=np.float64)
    return Imu(
        header=_header(stamp_ns, "imu_link"),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=cov,
        angular_velocity=Vector3(x=0.0, y=0.0, z=wz),
        angular_velocity_covariance=cov,
        linear_acceleration=Vector3(x=0.1, y=ay, z=9.81),
        linear_acceleration_covariance=cov,
    )


def _make_diag_msg(stamp_ns: int, name: str, status_msg: str, level: int = 2):
    DiagnosticArray = TYPESTORE.types["diagnostic_msgs/msg/DiagnosticArray"]
    DiagnosticStatus = TYPESTORE.types["diagnostic_msgs/msg/DiagnosticStatus"]
    KeyValue = TYPESTORE.types["diagnostic_msgs/msg/KeyValue"]
    return DiagnosticArray(
        header=_header(stamp_ns, ""),
        status=[
            DiagnosticStatus(
                level=level,
                name=name,
                message=status_msg,
                hardware_id="planner",
                values=[
                    KeyValue(key="recovery_count", value="3"),
                    KeyValue(key="state", value="STUCK"),
                ],
            )
        ],
    )


def _write_synthetic_bag(bag_dir: Path, storage: StoragePlugin) -> Path:
    with Writer(bag_dir, version=9, storage_plugin=storage) as writer:
        conn_odom = writer.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=TYPESTORE)
        conn_scan = writer.add_connection("/cmd_vel", "geometry_msgs/msg/Twist", typestore=TYPESTORE)
        conn_tf = writer.add_connection("/tf", "tf2_msgs/msg/TFMessage", typestore=TYPESTORE)
        conn_path = writer.add_connection("/plan", "nav_msgs/msg/Path", typestore=TYPESTORE)
        conn_imu = writer.add_connection("/imu/data", "sensor_msgs/msg/Imu", typestore=TYPESTORE)
        conn_diag = writer.add_connection(
            "/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", typestore=TYPESTORE
        )

        base_ns = 1_700_000_000_000_000_000
        # Write 25 odom messages at ~50 ms, with a 350 ms gap between index 12 and 13
        ts_ns = base_ns
        for i in range(25):
            if i == 13:
                ts_ns += 350_000_000
            elif i > 0:
                ts_ns += 50_000_000
            raw = TYPESTORE.serialize_cdr(
                _make_odom_msg(ts_ns, x=0.1 * i, y=0.05 * i, yaw=0.02 * i),
                "nav_msgs/msg/Odometry",
            )
            writer.write(conn_odom, ts_ns, raw)

        # Write cmd_vel timestamps including a duplicate and an out-of-order regression
        cmd_offsets_ms = [0, 50, 50, 150, 120, 200, 250]
        for idx, offset_ms in enumerate(cmd_offsets_ms):
            t_ns = base_ns + offset_ms * 1_000_000
            raw = TYPESTORE.serialize_cdr(
                _make_twist_msg(vx=0.5 + 0.1 * idx, wz=0.05),
                "geometry_msgs/msg/Twist",
            )
            writer.write(conn_scan, t_ns, raw)

        writer.write(
            conn_tf,
            base_ns + 100_000_000,
            TYPESTORE.serialize_cdr(
                _make_tf_msg(base_ns + 100_000_000, "map", "odom", 1.2, -0.4, 0.25),
                "tf2_msgs/msg/TFMessage",
            ),
        )
        writer.write(
            conn_path,
            base_ns + 100_000_000,
            TYPESTORE.serialize_cdr(
                _make_path_msg(base_ns + 100_000_000, [(0.0, 0.0, 0.0), (1.0, 0.2, 0.15)]),
                "nav_msgs/msg/Path",
            ),
        )
        writer.write(
            conn_imu,
            base_ns + 100_000_000,
            TYPESTORE.serialize_cdr(
                _make_imu_msg(base_ns + 100_000_000, wz=0.33, ay=0.45),
                "sensor_msgs/msg/Imu",
            ),
        )
        writer.write(
            conn_diag,
            base_ns + 200_000_000,
            TYPESTORE.serialize_cdr(
                _make_diag_msg(base_ns + 200_000_000, "bt_navigator", "ClearCostmap triggered"),
                "diagnostic_msgs/msg/DiagnosticArray",
            ),
        )
    return bag_dir


def test_geometry_round_trip() -> None:
    assert normalize_angle(3.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)

    roll_in, pitch_in, yaw_in = 0.12, -0.25, 1.35
    qx, qy, qz, qw = euler_to_quaternion(roll_in, pitch_in, yaw_in)
    roll_out, pitch_out, yaw_out = quaternion_to_euler(qx, qy, qz, qw)
    assert roll_out == pytest.approx(roll_in, abs=1e-9)
    assert pitch_out == pytest.approx(pitch_in, abs=1e-9)
    assert yaw_out == pytest.approx(yaw_in, abs=1e-9)


def test_discover_bag_errors(tmp_path: Path) -> None:
    with pytest.raises(BagDiscoveryError):
        discover_bag(tmp_path / "does_not_exist")

    empty_dir = tmp_path / "empty"
    empty_dir.mkdir()
    with pytest.raises(BagDiscoveryError):
        discover_bag(empty_dir)


@pytest.mark.parametrize("storage", [StoragePlugin.MCAP, StoragePlugin.SQLITE3])
def test_timing_scanner_and_stream_reader(tmp_path: Path, storage: StoragePlugin) -> None:
    bag_dir = _write_synthetic_bag(tmp_path / f"bag_{storage.name.lower()}", storage)

    desc = discover_bag(bag_dir)
    assert desc.storage_format == ("mcap" if storage == StoragePlugin.MCAP else "sqlite3")
    assert len(desc.files) == 1

    # Single file path discovery also succeeds
    file_desc = discover_bag(desc.files[0])
    assert file_desc.files == desc.files

    summaries = {s.topic: s for s in scan_bag_timing(bag_dir)}
    assert "/odom" in summaries
    odom = summaries["/odom"]
    assert odom.message_count == 25
    assert odom.max_gap_ms == pytest.approx(350.0, abs=1e-3)
    assert odom.median_period_ms == pytest.approx(50.0, abs=1e-3)
    assert odom.heuristic_large_gap is True
    assert odom.monotonic_violations == 0

    cmd = summaries["/cmd_vel"]
    assert cmd.message_count == 7
    assert cmd.duplicate_timestamps == 1
    assert cmd.monotonic_violations == 1

    # Topic filter in timing scanner
    filtered_summaries = scan_bag_timing(bag_dir, topics=["/odom"])
    assert [s.topic for s in filtered_summaries] == ["/odom"]

    # Stream reader with topic filter and time window
    base_ns = 1_700_000_000_000_000_000
    odom_msgs = list(
        iter_bag_messages(
            bag_dir,
            topics=["/odom"],
            start_ns=base_ns,
            stop_ns=base_ns + 200_000_000,
        )
    )
    assert 4 <= len(odom_msgs) <= 5
    assert all(m.topic == "/odom" for m in odom_msgs)
    assert all(m.header_stamp_ns == m.log_time_ns for m in odom_msgs)


def test_topic_map_loading_and_message_mapping(tmp_path: Path) -> None:
    nav2_map = load_topic_map(REPO_ROOT / "configs" / "topic_map.nav2.yaml")
    autoware_map = load_topic_map(REPO_ROOT / "configs" / "topic_map.autoware.yaml")

    assert "/odom" in nav2_map.all_topics()
    assert nav2_map.topics_by_role()["local_plan"] == "/plan"
    assert "/localization/kinematic_state" in autoware_map.all_topics()
    assert autoware_map.topics_by_role()["chassis_feedback"] == "/vehicle/status/velocity_status"

    bag_dir = _write_synthetic_bag(tmp_path / "nav2_bag", StoragePlugin.MCAP)
    mapped_types: set[type] = set()
    for raw in iter_bag_messages(bag_dir):
        for sig in nav2_map.roles_for_topic(raw.topic):
            samples = map_message(raw.log_time_ns, raw.msg, sig)
            for s in samples:
                mapped_types.add(type(s))
                if isinstance(s, DiagnosticEvent):
                    assert s.values["recovery_count"] == 3.0
                    assert s.values["state"] == "STUCK"
                elif isinstance(s, PoseSample) and sig.role == "odom_pose":
                    assert s.cov_trace == pytest.approx(0.04 + 0.09 + 0.02)

    assert {PoseSample, PathSample, ControlCmdSample, ImuSample, DiagnosticEvent}.issubset(
        mapped_types
    )


def test_extract_dot_path_and_frd_deg_conversions() -> None:
    payload = SimpleNamespace(
        header=SimpleNamespace(frame_id="body", stamp=SimpleNamespace(sec=10, nanosec=500)),
        pos=SimpleNamespace(x=3.0, y=1.5, z=-0.5),
        att=SimpleNamespace(roll=0.0, pitch=0.0, yaw=90.0),
        vel=SimpleNamespace(vx=2.0, vy=0.4, wz=30.0),
        cov=[0.1, 0.2, 0.3],
    )
    assert extract_dot_path(payload, "cov[2]") == 0.3
    assert extract_dot_path(payload, "cov[99]") is None

    frd_mapping = SignalMapping(
        role="private_pose",
        topic="/custom/pose",
        signal_type="pose",
        angle_unit="deg",
        coordinate_convention="FRD",
        fields={
            "frame_id": "header.frame_id",
            "x": "pos.x",
            "y": "pos.y",
            "z": "pos.z",
            "roll": "att.roll",
            "pitch": "att.pitch",
            "yaw": "att.yaw",
            "vx": "vel.vx",
            "vy": "vel.vy",
            "wz": "vel.wz",
        },
    )
    [pose] = map_message(999, payload, frd_mapping)
    assert isinstance(pose, PoseSample)
    assert pose.stamp_ns == 10_000_000_500
    assert pose.x == pytest.approx(3.0)
    assert pose.y == pytest.approx(-1.5)
    assert pose.z == pytest.approx(0.5)
    assert pose.yaw == pytest.approx(-math.pi / 2)
    assert pose.vy == pytest.approx(-0.4)
    assert pose.wz == pytest.approx(-30.0)

    ctrl_mapping = SignalMapping(
        role="ctrl",
        topic="/custom/ctrl",
        signal_type="control_cmd",
        angle_unit="deg",
        coordinate_convention="FRD",
        fields={"linear_vel": "vx", "steering_angle": "steer"},
    )
    [cmd] = map_message(1234, {"vx": 4.0, "steer": 15.0}, ctrl_mapping)
    assert isinstance(cmd, ControlCmdSample)
    assert cmd.linear_vel == pytest.approx(4.0)
    assert cmd.steering_angle == pytest.approx(-math.radians(15.0))
