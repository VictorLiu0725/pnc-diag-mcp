"""Standardized internal physical signal models decoupled from ROS schemas."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

StorageFormat = Literal["mcap", "sqlite3", "rosbag1"]
TimeDomain = Literal["header_stamp", "log_time", "fallback_log_time", "missing_or_zero"]


@dataclass(frozen=True, slots=True)
class BagDescriptor:
    """Resolved recording location, storage format, and ordered data files."""

    root_path: Path
    storage_format: StorageFormat
    files: tuple[Path, ...]
    metadata: dict[str, Any] | None = None


@dataclass(frozen=True, slots=True)
class GapInterval:
    """Detected inter-message gap exceeding nominal interval thresholds."""

    start_ns: int
    end_ns: int
    gap_ms: float


@dataclass(slots=True)
class TopicTimingSummary:
    """Zero-deserialization bounded-memory timing health summary for a single topic."""

    topic: str
    message_type: str
    message_count: int
    start_ns: int | None
    end_ns: int | None
    duration_s: float
    mean_rate_hz: float | None
    median_period_ms: float | None
    p95_period_ms: float | None
    p95_jitter_ms: float | None
    max_gap_ms: float | None
    p99_period_ms: float | None = None
    monotonic_violations: int = 0
    duplicate_timestamps: int = 0
    zero_timestamps: int = 0
    heuristic_large_gap: bool = False
    gap_intervals: tuple[GapInterval, ...] = ()


@dataclass(frozen=True, slots=True)
class PoseSample:
    """Standardized 3D pose (FLU convention, radians) with provenance and time uncertainty."""

    stamp_ns: int
    log_time_ns: int
    frame_id: str
    child_frame_id: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    vx: float | None = None
    vy: float | None = None
    wz: float | None = None
    cov_xx: float | None = None
    cov_yy: float | None = None
    cov_yaw: float | None = None
    cov_trace: float | None = None
    time_domain: TimeDomain = "header_stamp"
    stamp_is_valid: bool = True
    alignment_uncertainty_ns: int = 5_000_000


@dataclass(frozen=True, slots=True)
class TwistSample:
    """Standardized 6-DoF velocity feedback (FLU convention, m/s and rad/s)."""

    stamp_ns: int
    log_time_ns: int
    vx: float
    vy: float
    vz: float
    wx: float
    wy: float
    wz: float
    time_domain: TimeDomain = "header_stamp"
    stamp_is_valid: bool = True
    alignment_uncertainty_ns: int = 5_000_000


@dataclass(frozen=True, slots=True)
class ImuSample:
    """Standardized IMU angular velocity (rad/s) and linear acceleration (m/s^2)."""

    stamp_ns: int
    log_time_ns: int
    wx: float
    wy: float
    wz: float
    ax: float
    ay: float
    az: float
    time_domain: TimeDomain = "header_stamp"
    stamp_is_valid: bool = True
    alignment_uncertainty_ns: int = 5_000_000


@dataclass(frozen=True, slots=True)
class PathPoint:
    """Single waypoint along a planned trajectory or path."""

    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    v: float | None = None


@dataclass(frozen=True, slots=True)
class PathSample:
    """Standardized planned path or trajectory at a single planning cycle."""

    stamp_ns: int
    log_time_ns: int
    frame_id: str
    points: tuple[PathPoint, ...]
    time_domain: TimeDomain = "header_stamp"
    stamp_is_valid: bool = True
    alignment_uncertainty_ns: int = 5_000_000


@dataclass(frozen=True, slots=True)
class ControlCmdSample:
    """Unified control command across Twist (/cmd_vel) and Ackermann/Autoware commands."""

    stamp_ns: int
    log_time_ns: int
    linear_vel: float | None = None
    angular_vel: float | None = None
    steering_angle: float | None = None
    acceleration: float | None = None
    time_domain: TimeDomain = "log_time"
    stamp_is_valid: bool = True
    alignment_uncertainty_ns: int = 20_000_000


@dataclass(frozen=True, slots=True)
class LaserScanSample:
    """Lightweight range summary of a 2D LaserScan without holding full point clouds."""

    stamp_ns: int
    log_time_ns: int
    frame_id: str
    num_beams: int
    valid_ratio: float
    min_range_m: float | None
    p10_range_m: float | None
    median_range_m: float | None
    front_min_range_m: float | None
    time_domain: TimeDomain = "header_stamp"
    stamp_is_valid: bool = True
    alignment_uncertainty_ns: int = 5_000_000


@dataclass(frozen=True, slots=True)
class DiagnosticEvent:
    """Normalized diagnostic, status, BT transition, or solver metric event."""

    stamp_ns: int
    log_time_ns: int
    source: str
    level: int | str
    name: str
    message: str
    values: dict[str, float | str] = field(default_factory=dict)
    time_domain: TimeDomain = "header_stamp"
    stamp_is_valid: bool = True
    alignment_uncertainty_ns: int = 5_000_000
