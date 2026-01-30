"""
Enclosure generator - box, mount, housing, bracket, rack_mount.
Params: waterproof, ventilated, mounting_pattern, snap_fit_lid.
Uses shared OMEGA ID: artifact_id() for generated designs.
"""

import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

_PRODUCTS = Path(__file__).resolve().parent.parent.parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.id_generator import artifact_id

from ..intent_parser import DesignSpec

try:
    import trimesh
    import numpy as np
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False


@dataclass
class EnclosureGenerator:
    """Generate enclosure design dict and geometry from DesignSpec."""

    design_counter: int = 0

    SCALE_MM = {"small": 30, "medium": 60, "large": 120}
    MOUNTING_PATTERNS = {"M3": 3, "M4": 4, "1/4-20": 6.35}  # hole diameter mm

    def generate(self, spec: DesignSpec) -> Dict:
        self.design_counter += 1
        enc_type = spec.params.get("type", "box")
        scale_mm = self.SCALE_MM.get(spec.scale, 60)
        waterproof = spec.params.get("waterproof", False)
        ventilated = spec.params.get("ventilated", False)
        mounting_pattern = spec.params.get("mounting_pattern", "M3")
        snap_fit_lid = spec.params.get("snap_fit_lid", False)

        design_id = artifact_id(suffix=f"E-{self.design_counter:04d}")
        name = f"{enc_type.replace('_', ' ').title()} Enclosure"
        if enc_type == "rack_mount":
            size_mm = {"x": 482.6, "y": scale_mm, "z": 44.45 * 2}
        elif enc_type == "bracket":
            size_mm = {"x": scale_mm, "y": scale_mm * 0.6, "z": scale_mm * 0.15}
        elif enc_type == "mount":
            size_mm = {"x": scale_mm, "y": scale_mm * 0.8, "z": scale_mm * 0.1}
        else:
            size_mm = {"x": scale_mm, "y": scale_mm, "z": scale_mm * 0.6}

        design = {
            "id": design_id,
            "name": name,
            "domain": "enclosure",
            "type": enc_type,
            "size_mm": size_mm,
            "wall_thickness_mm": max(2, scale_mm * 0.05),
            "waterproof": waterproof,
            "ventilated": ventilated,
            "mounting_pattern": mounting_pattern,
            "snap_fit_lid": snap_fit_lid,
            "created_at": datetime.now().isoformat(),
            "source_intent": spec.raw_intent,
        }
        return design

    def generate_mesh(self, design: Dict):
        if not TRIMESH_AVAILABLE:
            return None
        enc_type = design.get("type", "box")
        size = design.get("size_mm", {"x": 60, "y": 60, "z": 36})
        t = design.get("wall_thickness_mm", 3)
        sx, sy, sz = size["x"] / 1000, size["y"] / 1000, size["z"] / 1000
        t_m = t / 1000

        if enc_type == "box":
            outer = trimesh.creation.box(extents=[sx, sy, sz])
            inner = trimesh.creation.box(
                extents=[sx - 2 * t_m, sy - 2 * t_m, sz - t_m]
            )
            inner.apply_translation([0, 0, t_m / 2])
            try:
                mesh = outer.difference(inner)
            except Exception:
                mesh = outer
            if design.get("snap_fit_lid"):
                lid_groove = trimesh.creation.box(
                    extents=[sx - 2 * t_m - 0.002, sy - 2 * t_m - 0.002, 0.003]
                )
                lid_groove.apply_translation([0, 0, sz - t_m - 0.0015])
                try:
                    mesh = mesh.difference(lid_groove)
                except Exception:
                    pass
            return mesh

        if enc_type == "mount":
            plate = trimesh.creation.box(extents=[sx, sy, sz])
            hole_r = (self.MOUNTING_PATTERNS.get(design.get("mounting_pattern", "M3"), 3) / 1000) / 2
            for dx, dy in [(-sx/4, -sy/4), (sx/4, -sy/4), (-sx/4, sy/4), (sx/4, sy/4)]:
                cyl = trimesh.creation.cylinder(radius=hole_r, height=sz + 0.001)
                cyl.apply_translation([dx, dy, -0.0005])
                try:
                    plate = plate.difference(cyl)
                except Exception:
                    pass
            return plate

        if enc_type == "bracket":
            L = sx
            w = sy
            th = sz
            a = trimesh.creation.box(extents=[L, w, th])
            b = trimesh.creation.box(extents=[w, w, th])
            b.apply_translation([L / 2 + w / 2, 0, 0])
            try:
                mesh = a.union(b)
            except Exception:
                mesh = a
            return mesh

        if enc_type == "housing":
            outer = trimesh.creation.box(extents=[sx, sy, sz])
            inner = trimesh.creation.box(
                extents=[sx - 2 * t_m, sy - 2 * t_m, sz - t_m]
            )
            inner.apply_translation([0, 0, t_m / 2])
            try:
                mesh = outer.difference(inner)
            except Exception:
                mesh = outer
            if design.get("ventilated"):
                slot_h = 0.005
                for i in range(-1, 2):
                    slot = trimesh.creation.box(extents=[sx - 2 * t_m - 0.002, slot_h, 0.008])
                    slot.apply_translation([0, i * (sy - 2 * t_m) / 3, sz - t_m - 0.004])
                    try:
                        mesh = mesh.difference(slot)
                    except Exception:
                        pass
            return mesh

        if enc_type == "rack_mount":
            outer = trimesh.creation.box(extents=[sx / 1000, sy / 1000, sz / 1000])
            return outer

        outer = trimesh.creation.box(extents=[sx, sy, sz])
        inner = trimesh.creation.box(
            extents=[sx - 2 * t_m, sy - 2 * t_m, sz - t_m]
        )
        inner.apply_translation([0, 0, t_m / 2])
        try:
            mesh = outer.difference(inner)
        except Exception:
            mesh = outer
        return mesh

    def generate_urdf(self, design: Dict, mesh_path: Optional[str] = None) -> str:
        size = design.get("size_mm", {"x": 60, "y": 60, "z": 36})
        sx, sy, sz = size["x"] / 1000, size["y"] / 1000, size["z"] / 1000
        name = design.get("id", "enclosure").replace("-", "_").replace(" ", "_")
        return f'''<?xml version="1.0"?>
<robot name="{name}">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="{sx} {sy} {sz}"/>
      </geometry>
      <material name="gray">
        <color rgba="0.6 0.6 0.65 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="{sx} {sy} {sz}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
'''
