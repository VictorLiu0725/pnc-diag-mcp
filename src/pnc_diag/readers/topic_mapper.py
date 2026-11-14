"""Semantic topic and dot-path field mapper for standard and private ROS schemas."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pnc_diag.models.geometry import normalize_angle, quaternion_to_euler
from pnc_diag.models.signals import (
    ControlCmdSample,
    DiagnosticEvent,
    ImuSample,
    LaserScanSample,
    PathPoint,
    PathSample,
    PoseSample,
    TimeDomain,
    TwistSample,
)
from pnc_diag.readers.stream_reader import extract_header_stamp_ns


@dataclass(frozen=True, slots=True)
class SignalMapping:
    """Configuration for mapping a single ROS topic onto a logical signal role."""

    role: str
    topic: str
    signal_type: str
    fields: dict[str, str] = field(default_factory=dict)
    angle_unit: str = "rad"
    coordinate_convention: str = "FLU"


@dataclass(frozen=True, slots=True)
class TopicMapConfig:
    """Complete semantic topic mapping configuration."""

    name: str
    signals: dict[str, SignalMapping]
    coordinate_convention: str = "FLU"
    angle_unit: str = "rad"

    def topics_by_role(self) -> dict[str, str]:
        return {role: sig.topic for role, sig in self.signals.items()}

    def roles_for_topic(self, topic: str) -> list[SignalMapping]:
        return [sig for sig in self.signals.values() if sig.topic == topic]

    def all_topics(self) -> set[str]:
        return {sig.topic for sig in self.signals.values()}


def load_topic_map(config_path: str | Path) -> TopicMapConfig:
    """Load a YAML topic_map configuration file."""
    path = Path(config_path)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise TypeError(f"Topic map must be a YAML mapping: {path}")

    default_conv = str(raw.get("coordinate_convention", "FLU")).upper()
    default_unit = str(raw.get("angle_unit", "rad")).lower()
    raw_signals = raw.get("signals", {})
    if not isinstance(raw_signals, dict):
        raise TypeError(f"'signals' must be a mapping in {path}")

    signals: dict[str, SignalMapping] = {}
    for role, entry in raw_signals.items():
        if not isinstance(entry, dict) or "topic" not in entry or "signal_type" not in entry:
            raise ValueError(f"Signal '{role}' requires 'topic' and 'signal_type' in {path}")
        signals[role] = SignalMapping(
            role=role,
            topic=str(entry["topic"]),
            signal_type=str(entry["signal_type"]).lower(),
            fields={str(k): str(v) for k, v in (entry.get("fields") or {}).items()},
            angle_unit=str(entry.get("angle_unit", default_unit)).lower(),
            coordinate_convention=str(entry.get("coordinate_convention", default_conv)).upper(),
        )

    return TopicMapConfig(
        name=str(raw.get("name", path.stem)),
        signals=signals,
        coordinate_convention=default_conv,
        angle_unit=default_unit,
    )


def extract_dot_path(obj: Any, path: str) -> Any:
    """Extract a nested attribute/key with optional array indexing (e.g. 'pose.covariance[0]')."""
    cur = obj
    for part in path.split("."):
        if cur is None:
            return None
        if "[" in part and part.endswith("]"):
            name, idx_str = part[:-1].split("[", 1)
            if name:
                cur = cur.get(name) if isinstance(cur, dict) else getattr(cur, name, None)
            if cur is None:
                return None
            try:
                cur = cur[int(idx_str)]
            except (IndexError, KeyError, TypeError, ValueError):
                return None
        elif isinstance(cur, dict):
            cur = cur.get(part)
        else:
            cur = getattr(cur, part, None)
    return cur


def _opt_float(obj: Any, path: str | None) -> float | None:
    if not path:
        return None
    val = extract_dot_path(obj, path)
    if val is None:
        return None
    f = float(val)
    return f if math.isfinite(f) else None


def _req_float(obj: Any, path: str | None, default: float = 0.0) -> float:
    val = _opt_float(obj, path)
    return default if val is None else val


def _to_rad(val: float, unit: str) -> float:
    return math.radians(val) if unit == "deg" else val


def resolve_message_time(
    log_time_ns: int, msg: Any
) -> tuple[int, TimeDomain, bool, int]:
    """Determine authoritative timestamp, clock domain, validity, and uncertainty (ns)."""
    header_ns = extract_header_stamp_ns(msg)
    if header_ns is not None:
        if header_ns <= 0:
            return int(log_time_ns), "missing_or_zero", False, 500_000_000
        return int(header_ns), "header_stamp", True, 5_000_000

    has_header = hasattr(msg, "header") or (isinstance(msg, dict) and "header" in msg)
    if has_header:
        return int(log_time_ns), "missing_or_zero", False, 500_000_000
    return int(log_time_ns), "log_time", True, 20_000_000


def _apply_pose_convention(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    convention: str,
) -> tuple[float, float, float, float, float, float]:
    if convention == "FRD":
        return x, -y, -z, roll, normalize_angle(-pitch), normalize_angle(-yaw)
    return x, y, z, roll, pitch, normalize_angle(yaw)


def _map_pose(
    log_time_ns: int, msg: Any, mapping: SignalMapping
) -> PoseSample:
    f = mapping.fields
    stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, msg)
    frame_id = str(extract_dot_path(msg, f.get("frame_id", "header.frame_id")) or "")
    child_frame_id = str(extract_dot_path(msg, f.get("child_frame_id", "child_frame_id")) or "")

    x = _req_float(msg, f.get("x", "pose.pose.position.x"))
    y = _req_float(msg, f.get("y", "pose.pose.position.y"))
    z = _req_float(msg, f.get("z", "pose.pose.position.z"))

    if "yaw" in f and "qx" not in f:
        roll = _to_rad(_req_float(msg, f.get("roll")), mapping.angle_unit)
        pitch = _to_rad(_req_float(msg, f.get("pitch")), mapping.angle_unit)
        yaw = _to_rad(_req_float(msg, f.get("yaw")), mapping.angle_unit)
    else:
        qx = _req_float(msg, f.get("qx", "pose.pose.orientation.x"))
        qy = _req_float(msg, f.get("qy", "pose.pose.orientation.y"))
        qz = _req_float(msg, f.get("qz", "pose.pose.orientation.z"))
        qw = _req_float(msg, f.get("qw", "pose.pose.orientation.w"), default=1.0)
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

    x, y, z, roll, pitch, yaw = _apply_pose_convention(
        x, y, z, roll, pitch, yaw, mapping.coordinate_convention
    )

    vx = _opt_float(msg, f.get("vx"))
    vy = _opt_float(msg, f.get("vy"))
    wz = _opt_float(msg, f.get("wz"))
    if mapping.coordinate_convention == "FRD":
        vy = -vy if vy is not None else None
        wz = -wz if wz is not None else None

    cov_xx = _opt_float(msg, f.get("cov_xx"))
    cov_yy = _opt_float(msg, f.get("cov_yy"))
    cov_yaw = _opt_float(msg, f.get("cov_yaw"))
    cov_trace = None
    if cov_xx is not None and cov_yy is not None:
        cov_trace = cov_xx + cov_yy + (cov_yaw or 0.0)

    return PoseSample(
        stamp_ns=stamp_ns,
        log_time_ns=log_time_ns,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        vx=vx,
        vy=vy,
        wz=wz,
        cov_xx=cov_xx,
        cov_yy=cov_yy,
        cov_yaw=cov_yaw,
        cov_trace=cov_trace,
        time_domain=time_domain,
        stamp_is_valid=stamp_is_valid,
        alignment_uncertainty_ns=unc_ns,
    )


def _map_tf(log_time_ns: int, msg: Any, mapping: SignalMapping) -> list[PoseSample]:
    transforms = extract_dot_path(msg, mapping.fields.get("transforms", "transforms")) or []
    results: list[PoseSample] = []
    for tf in transforms:
        stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, tf)
        frame_id = str(extract_dot_path(tf, "header.frame_id") or "")
        child_frame_id = str(extract_dot_path(tf, "child_frame_id") or "")
        x = _req_float(tf, "transform.translation.x")
        y = _req_float(tf, "transform.translation.y")
        z = _req_float(tf, "transform.translation.z")
        qx = _req_float(tf, "transform.rotation.x")
        qy = _req_float(tf, "transform.rotation.y")
        qz = _req_float(tf, "transform.rotation.z")
        qw = _req_float(tf, "transform.rotation.w", default=1.0)
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        x, y, z, roll, pitch, yaw = _apply_pose_convention(
            x, y, z, roll, pitch, yaw, mapping.coordinate_convention
        )
        results.append(
            PoseSample(
                stamp_ns=stamp_ns,
                log_time_ns=log_time_ns,
                frame_id=frame_id,
                child_frame_id=child_frame_id,
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                time_domain=time_domain,
                stamp_is_valid=stamp_is_valid,
                alignment_uncertainty_ns=unc_ns,
            )
        )
    return results


def _map_path(log_time_ns: int, msg: Any, mapping: SignalMapping) -> PathSample:
    f = mapping.fields
    stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, msg)
    frame_id = str(extract_dot_path(msg, f.get("frame_id", "header.frame_id")) or "")
    raw_pts = extract_dot_path(msg, f.get("points", "poses")) or []

    pts: list[PathPoint] = []
    for item in raw_pts:
        px = _req_float(item, f.get("point_x", "pose.position.x"))
        py = _req_float(item, f.get("point_y", "pose.position.y"))
        pz = _req_float(item, f.get("point_z", "pose.position.z"))
        yaw: float | None = None
        if "point_qx" in f:
            qx = _req_float(item, f["point_qx"])
            qy = _req_float(item, f.get("point_qy", ""))
            qz = _req_float(item, f.get("point_qz", ""))
            qw = _req_float(item, f.get("point_qw", ""), default=1.0)
            _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)
        v = _opt_float(item, f.get("point_v"))
        if mapping.coordinate_convention == "FRD":
            py, pz = -py, -pz
            if yaw is not None:
                yaw = normalize_angle(-yaw)
        pts.append(PathPoint(x=px, y=py, z=pz, yaw=yaw, v=v))

    return PathSample(
        stamp_ns=stamp_ns,
        log_time_ns=log_time_ns,
        frame_id=frame_id,
        points=tuple(pts),
        time_domain=time_domain,
        stamp_is_valid=stamp_is_valid,
        alignment_uncertainty_ns=unc_ns,
    )


def _map_control_cmd(log_time_ns: int, msg: Any, mapping: SignalMapping) -> ControlCmdSample:
    f = mapping.fields
    stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, msg)
    lin = _opt_float(msg, f.get("linear_vel"))
    ang = _opt_float(msg, f.get("angular_vel"))
    steer = _opt_float(msg, f.get("steering_angle"))
    accel = _opt_float(msg, f.get("acceleration"))
    if steer is not None and mapping.angle_unit == "deg":
        steer = math.radians(steer)
    if mapping.coordinate_convention == "FRD":
        ang = -ang if ang is not None else None
        steer = -steer if steer is not None else None
    return ControlCmdSample(
        stamp_ns=stamp_ns,
        log_time_ns=log_time_ns,
        linear_vel=lin,
        angular_vel=ang,
        steering_angle=steer,
        acceleration=accel,
        time_domain=time_domain,
        stamp_is_valid=stamp_is_valid,
        alignment_uncertainty_ns=unc_ns,
    )


def _map_twist(log_time_ns: int, msg: Any, mapping: SignalMapping) -> TwistSample:
    f = mapping.fields
    stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, msg)
    vx = _req_float(msg, f.get("vx", "linear.x"))
    vy = _req_float(msg, f.get("vy", "linear.y"))
    vz = _req_float(msg, f.get("vz", "linear.z"))
    wx = _req_float(msg, f.get("wx", "angular.x"))
    wy = _req_float(msg, f.get("wy", "angular.y"))
    wz = _req_float(msg, f.get("wz", "angular.z"))
    if mapping.coordinate_convention == "FRD":
        vy, vz, wy, wz = -vy, -vz, -wy, -wz
    return TwistSample(
        stamp_ns=stamp_ns,
        log_time_ns=log_time_ns,
        vx=vx,
        vy=vy,
        vz=vz,
        wx=wx,
        wy=wy,
        wz=wz,
        time_domain=time_domain,
        stamp_is_valid=stamp_is_valid,
        alignment_uncertainty_ns=unc_ns,
    )


def _map_imu(log_time_ns: int, msg: Any, mapping: SignalMapping) -> ImuSample:
    f = mapping.fields
    stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, msg)
    wx = _req_float(msg, f.get("wx", "angular_velocity.x"))
    wy = _req_float(msg, f.get("wy", "angular_velocity.y"))
    wz = _req_float(msg, f.get("wz", "angular_velocity.z"))
    ax = _req_float(msg, f.get("ax", "linear_acceleration.x"))
    ay = _req_float(msg, f.get("ay", "linear_acceleration.y"))
    az = _req_float(msg, f.get("az", "linear_acceleration.z"))
    if mapping.coordinate_convention == "FRD":
        wy, wz, ay, az = -wy, -wz, -ay, -az
    return ImuSample(
        stamp_ns=stamp_ns,
        log_time_ns=log_time_ns,
        wx=wx,
        wy=wy,
        wz=wz,
        ax=ax,
        ay=ay,
        az=az,
        time_domain=time_domain,
        stamp_is_valid=stamp_is_valid,
        alignment_uncertainty_ns=unc_ns,
    )


def _map_laser_scan(log_time_ns: int, msg: Any, mapping: SignalMapping) -> LaserScanSample:
    f = mapping.fields
    stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, msg)
    frame_id = str(extract_dot_path(msg, f.get("frame_id", "header.frame_id")) or "")
    ranges_raw = extract_dot_path(msg, f.get("ranges", "ranges"))
    range_min = float(extract_dot_path(msg, f.get("range_min", "range_min")) or 0.05)
    range_max = float(extract_dot_path(msg, f.get("range_max", "range_max")) or 100.0)
    angle_min = float(extract_dot_path(msg, f.get("angle_min", "angle_min")) or -math.pi)
    angle_inc = float(extract_dot_path(msg, f.get("angle_increment", "angle_increment")) or 0.0)

    arr = np.asarray(ranges_raw if ranges_raw is not None else [], dtype=np.float64)
    num_beams = int(arr.size)
    if num_beams == 0:
        return LaserScanSample(
            stamp_ns=stamp_ns,
            log_time_ns=log_time_ns,
            frame_id=frame_id,
            num_beams=0,
            valid_ratio=0.0,
            min_range_m=None,
            p10_range_m=None,
            median_range_m=None,
            front_min_range_m=None,
            time_domain=time_domain,
            stamp_is_valid=stamp_is_valid,
            alignment_uncertainty_ns=unc_ns,
        )

    valid_mask = np.isfinite(arr) & (arr >= range_min) & (arr <= range_max)
    valid = arr[valid_mask]
    valid_ratio = float(valid.size) / float(num_beams)

    front_min: float | None = None
    if angle_inc > 0.0:
        angles = angle_min + np.arange(num_beams, dtype=np.float64) * angle_inc
        front_valid = arr[valid_mask & (np.abs(angles) <= 0.35)]
        if front_valid.size > 0:
            front_min = float(np.min(front_valid))

    return LaserScanSample(
        stamp_ns=stamp_ns,
        log_time_ns=log_time_ns,
        frame_id=frame_id,
        num_beams=num_beams,
        valid_ratio=valid_ratio,
        min_range_m=float(np.min(valid)) if valid.size > 0 else None,
        p10_range_m=float(np.percentile(valid, 10)) if valid.size > 0 else None,
        median_range_m=float(np.median(valid)) if valid.size > 0 else None,
        front_min_range_m=front_min,
        time_domain=time_domain,
        stamp_is_valid=stamp_is_valid,
        alignment_uncertainty_ns=unc_ns,
    )


def _map_diagnostics(log_time_ns: int, msg: Any, mapping: SignalMapping) -> list[DiagnosticEvent]:
    stamp_ns, time_domain, stamp_is_valid, unc_ns = resolve_message_time(log_time_ns, msg)
    statuses = getattr(msg, "status", None)
    if statuses is None:
        f = mapping.fields
        values: dict[str, float | str] = {}
        metric_path = f.get("metric_value")
        metric_name = str(f.get("metric_name", mapping.role))
        raw_scalar = extract_dot_path(msg, metric_path) if metric_path else getattr(msg, "data", None)
        if isinstance(raw_scalar, (int, float)) and not isinstance(raw_scalar, bool):
            val = float(raw_scalar)
            if math.isfinite(val):
                values[metric_name] = val
        elif raw_scalar is not None:
            try:
                val = float(raw_scalar)
                if math.isfinite(val):
                    values[metric_name] = val
            except (TypeError, ValueError):
                pass
        return [
            DiagnosticEvent(
                stamp_ns=stamp_ns,
                log_time_ns=log_time_ns,
                source=mapping.topic,
                level=getattr(msg, "level", "INFO"),
                name=str(getattr(msg, "name", mapping.role)),
                message=str(getattr(msg, "msg", getattr(msg, "data", ""))),
                values=values,
                time_domain=time_domain,
                stamp_is_valid=stamp_is_valid,
                alignment_uncertainty_ns=unc_ns,
            )
        ]

    events: list[DiagnosticEvent] = []
    for st in statuses:
        kv_dict: dict[str, float | str] = {}
        for kv in getattr(st, "values", []) or []:
            key = str(getattr(kv, "key", ""))
            raw_val = str(getattr(kv, "value", ""))
            try:
                kv_dict[key] = float(raw_val)
            except ValueError:
                kv_dict[key] = raw_val
        raw_lvl = getattr(st, "level", 0)
        level = int.from_bytes(raw_lvl, "little") if isinstance(raw_lvl, (bytes, bytearray)) else int(raw_lvl)
        events.append(
            DiagnosticEvent(
                stamp_ns=stamp_ns,
                log_time_ns=log_time_ns,
                source=mapping.topic,
                level=level,
                name=str(getattr(st, "name", "")),
                message=str(getattr(st, "message", "")),
                values=kv_dict,
                time_domain=time_domain,
                stamp_is_valid=stamp_is_valid,
                alignment_uncertainty_ns=unc_ns,
            )
        )
    return events


def map_message(
    log_time_ns: int, msg: Any, mapping: SignalMapping
) -> list[
    PoseSample
    | TwistSample
    | PathSample
    | ControlCmdSample
    | ImuSample
    | LaserScanSample
    | DiagnosticEvent
]:
    """Convert a raw deserialized ROS/private message into standardized physical samples."""
    st = mapping.signal_type
    if st == "pose":
        return [_map_pose(log_time_ns, msg, mapping)]
    if st == "tf":
        return list(_map_tf(log_time_ns, msg, mapping))
    if st == "path":
        return [_map_path(log_time_ns, msg, mapping)]
    if st == "control_cmd":
        return [_map_control_cmd(log_time_ns, msg, mapping)]
    if st == "twist":
        return [_map_twist(log_time_ns, msg, mapping)]
    if st == "imu":
        return [_map_imu(log_time_ns, msg, mapping)]
    if st == "laser_scan":
        return [_map_laser_scan(log_time_ns, msg, mapping)]
    if st == "diagnostics":
        return list(_map_diagnostics(log_time_ns, msg, mapping))
    raise ValueError(f"Unsupported signal_type '{st}' for role '{mapping.role}'")
