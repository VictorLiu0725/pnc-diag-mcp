"""Bag file discovery across MCAP, SQLite3 (.db3), ROS 1 (.bag), and ROS 2 directories."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import yaml

from pnc_diag.models.signals import BagDescriptor, StorageFormat


class BagDiscoveryError(RuntimeError):
    """Raised when a recording path is invalid or uses an unsupported format."""


def _natural_key(path: Path) -> list[object]:
    return [int(part) if part.isdigit() else part.lower() for part in re.split(r"(\d+)", path.name)]


def _load_metadata_yaml(directory: Path) -> dict[str, Any] | None:
    meta_path = directory / "metadata.yaml"
    if not meta_path.exists():
        return None
    try:
        raw = yaml.safe_load(meta_path.read_text(encoding="utf-8")) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise BagDiscoveryError(f"Failed to parse metadata.yaml: {exc}") from exc
    if not isinstance(raw, dict):
        raise BagDiscoveryError("metadata.yaml root must be a mapping")
    info = raw.get("rosbag2_bagfile_information", raw)
    if not isinstance(info, dict):
        raise BagDiscoveryError("rosbag2_bagfile_information must be a mapping")
    return info


def _resolve_metadata_files(directory: Path, info: dict[str, Any]) -> list[Path]:
    relative: list[str] = []
    raw_rel = info.get("relative_file_paths")
    if isinstance(raw_rel, list) and all(isinstance(x, str) for x in raw_rel):
        relative = raw_rel
    elif isinstance(info.get("files"), list):
        relative = [item["path"] for item in info["files"] if isinstance(item, dict) and isinstance(item.get("path"), str)]

    root = directory.resolve()
    resolved_files: list[Path] = []
    for entry in relative:
        if not entry:
            continue
        p = Path(entry)
        if p.is_absolute():
            raise BagDiscoveryError(f"metadata.yaml must use relative paths, got: {entry}")
        resolved = (root / p).resolve()
        if not resolved.is_relative_to(root):
            raise BagDiscoveryError(f"metadata.yaml path escapes bag directory: {entry}")
        if not resolved.is_file():
            raise BagDiscoveryError(f"metadata.yaml references missing file: {entry}")
        if resolved not in resolved_files:
            resolved_files.append(resolved)
    return resolved_files


def _infer_storage(files: list[Path], declared: str = "") -> StorageFormat:
    suffixes = {f.suffix.lower() for f in files}
    if suffixes == {".mcap"}:
        fmt: StorageFormat = "mcap"
    elif suffixes <= {".db3", ".sqlite3"}:
        fmt = "sqlite3"
    elif suffixes == {".bag"}:
        fmt = "rosbag1"
    else:
        raise BagDiscoveryError(f"Mixed or unsupported storage suffixes: {sorted(suffixes)}")

    if declared and declared != fmt:
        raise BagDiscoveryError(
            f"Declared storage '{declared}' does not match discovered files '{fmt}'"
        )
    return fmt


def discover_bag(path: str | Path) -> BagDescriptor:
    """Discover recording format and ordered storage files for a bag path or directory."""
    target = Path(path).expanduser().resolve()
    if not target.exists():
        raise BagDiscoveryError(f"Recording path does not exist: {target}")

    if target.is_file():
        fmt = _infer_storage([target])
        return BagDescriptor(root_path=target, storage_format=fmt, files=(target,), metadata=None)

    info = _load_metadata_yaml(target)
    declared = ""
    if info is not None:
        storage_raw = info.get("storage_identifier")
        if isinstance(storage_raw, str) and storage_raw:
            declared = storage_raw.lower()
        meta_files = _resolve_metadata_files(target, info)
        if meta_files:
            fmt = _infer_storage(meta_files, declared)
            return BagDescriptor(
                root_path=target,
                storage_format=fmt,
                files=tuple(meta_files),
                metadata=info,
            )

    mcap_files = sorted(target.glob("*.mcap"), key=_natural_key)
    db_files = sorted([*target.glob("*.db3"), *target.glob("*.sqlite3")], key=_natural_key)
    bag_files = sorted(target.glob("*.bag"), key=_natural_key)

    candidates = [grp for grp in (mcap_files, db_files, bag_files) if grp]
    if len(candidates) > 1:
        raise BagDiscoveryError(f"Directory contains mixed recording formats: {target}")
    if not candidates:
        raise BagDiscoveryError(f"No .mcap, .db3, .sqlite3, or .bag files found in {target}")

    files = candidates[0]
    fmt = _infer_storage(files, declared)
    return BagDescriptor(root_path=target, storage_format=fmt, files=tuple(files), metadata=info)

