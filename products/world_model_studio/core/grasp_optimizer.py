"""
World Model Studio — Grasp optimization for higher success rate.
Auto-scale gripper to object, better initial pose (approach from above, angled),
grip force estimation, contact detection.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False


def _parse_object_size(object_mjcf: str) -> Tuple[float, float, float]:
    """Extract approximate object bounding size (half-extents) from MJCF. Returns (rx, ry, rz) in meters."""
    rx, ry, rz = 0.03, 0.03, 0.03
    try:
        root = ET.fromstring(object_mjcf)
        for w in root.iter():
            tag = w.tag.split("}")[-1] if "}" in w.tag else w.tag
            if tag == "geom":
                size_str = w.get("size")
                geom_type = (w.get("type") or "box").lower()
                if size_str:
                    parts = [float(x) for x in size_str.split() if x]
                    if geom_type == "box" and len(parts) >= 3:
                        rx, ry, rz = parts[0], parts[1], parts[2]
                    elif geom_type == "sphere" and len(parts) >= 1:
                        rx = ry = rz = parts[0]
                    elif geom_type == "ellipsoid" and len(parts) >= 3:
                        rx, ry, rz = parts[0], parts[1], parts[2]
                    elif geom_type == "cylinder" and len(parts) >= 2:
                        rx = ry = parts[0]
                        rz = parts[1]
                    break
    except Exception:
        pass
    return (rx, ry, rz)


def _parse_gripper_scale(gripper_mjcf: str) -> float:
    """Extract nominal gripper scale (e.g. palm radius or finger length) from MJCF. Returns scale factor base (e.g. 0.01)."""
    try:
        root = ET.fromstring(gripper_mjcf)
        for w in root.iter():
            tag = w.tag.split("}")[-1] if "}" in w.tag else w.tag
            if tag == "geom" and "palm" in (w.get("name") or "").lower():
                size_str = w.get("size")
                if size_str:
                    parts = [float(x) for x in size_str.split() if x]
                    if parts:
                        return float(parts[0])
            if tag == "geom" and "finger" in (w.get("name") or "").lower():
                size_str = w.get("size")
                if size_str:
                    parts = [float(x) for x in size_str.split() if x]
                    if parts:
                        return float(parts[0]) * 3
    except Exception:
        pass
    return 0.01


def optimize_grasp_pose(gripper_mjcf: str, object_mjcf: str) -> Dict[str, Any]:
    """
    Compute recommended gripper pose (position, orientation) for approach-from-above, angled.
    Returns dict: position [x,y,z], orientation [euler or quat], approach_height.
    """
    rx, ry, rz = _parse_object_size(object_mjcf)
    obj_center_z = rz
    approach_height = obj_center_z + 0.08
    position = [0.0, 0.0, approach_height]
    orientation = [0.0, 0.0, 0.0]
    if NUMPY_AVAILABLE:
        tilt = 0.15
        orientation = [tilt, 0.0, 0.0]
    return {
        "position": position,
        "orientation": orientation,
        "approach_height": approach_height,
        "object_half_extents": [rx, ry, rz],
    }


def calculate_grip_force(gripper: Any, object_mass: float) -> float:
    """
    Estimate required grip force (N) for object mass. gripper can be MJCF string or dict with actuator info.
    Rule of thumb: force >= mass * g * safety_factor (e.g. 2.0).
    """
    g = 9.81
    safety = 2.0
    return max(0.5, float(object_mass) * g * safety)


def detect_successful_grasp(sim_state: Dict[str, Any]) -> bool:
    """
    Detect if object is successfully grasped from sim state.
    sim_state: dict with qpos, qvel (and optionally ncon, contact). Object is often first free body (qpos 0-6).
    Heuristic: object z above threshold and velocity low (not falling).
    """
    qpos = sim_state.get("qpos")
    qvel = sim_state.get("qvel")
    if qpos is None or len(qpos) < 7:
        return False
    try:
        if NUMPY_AVAILABLE:
            pos = np.asarray(qpos)
            vel = np.asarray(qvel) if qvel is not None else np.zeros_like(pos)
            object_z = float(pos[2])
            object_vel_z = float(vel[2]) if len(vel) > 2 else 0.0
            height_ok = object_z >= 0.08
            not_falling = object_vel_z >= -0.5
            return height_ok and not_falling
        object_z = float(qpos[2])
        return object_z >= 0.08
    except Exception:
        return False


def _scale_attr_values(text: str, scale: float) -> str:
    """Multiply all numeric values in size="..." or fromto="..." attributes."""
    def repl(m: re.Match) -> str:
        attr, val_str = m.group(1), m.group(2)
        parts = val_str.split()
        scaled = []
        for p in parts:
            try:
                scaled.append(str(float(p) * scale))
            except ValueError:
                scaled.append(p)
        return f'{attr}="{" ".join(scaled)}"'
    return re.sub(r'\b(size|fromto)\s*=\s*["\']([^"\']+)["\']', repl, text)


def auto_scale_gripper(gripper_mjcf: str, object_size: Tuple[float, float, float]) -> str:
    """
    Scale gripper MJCF to better match object size. object_size = (rx, ry, rz) half-extents.
    Returns scaled MJCF string. Scales size= and fromto= numbers proportionally.
    """
    rx, ry, rz = object_size
    obj_scale = (rx + ry + rz) / 3.0
    nominal = 0.03
    if obj_scale <= 0:
        return gripper_mjcf
    scale = obj_scale / nominal
    scale = max(0.5, min(2.0, scale))
    return _scale_attr_values(gripper_mjcf, scale)


def get_recommended_initial_z(object_mjcf: str) -> float:
    """Recommended gripper base Z so fingers can reach object center. Used by scene composer."""
    rx, ry, rz = _parse_object_size(object_mjcf)
    return float(rz) + 0.06
