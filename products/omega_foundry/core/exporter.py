"""
Design exporter - MJCF, URDF, STL (trimesh), JSON.
"""

import json
from pathlib import Path
from typing import Dict, Optional

from .design_engine import GeneratedDesign

try:
    import trimesh
    import numpy as np
    TRIMESH_AVAILABLE = True
except ImportError:
    TRIMESH_AVAILABLE = False


class DesignExporter:
    """Export GeneratedDesign to MJCF, URDF, STL, JSON."""

    def export(
        self,
        design: GeneratedDesign,
        output_dir: str,
        base_name: Optional[str] = None,
    ) -> Dict[str, str]:
        """
        Export design to output_dir. Returns paths keyed by format.

        Returns:
            {"mjcf": path, "urdf": path, "stl": path, "json": path} (only present formats)
        """
        out = Path(output_dir)
        out.mkdir(parents=True, exist_ok=True)
        base = base_name or design.id.replace("-", "_").replace(" ", "_")
        paths: Dict[str, str] = {}

        if design.mjcf_xml:
            mjcf_path = out / f"{base}.mjcf"
            mjcf_path.write_text(design.mjcf_xml, encoding="utf-8")
            paths["mjcf"] = str(mjcf_path)

        if design.urdf_xml:
            urdf_path = out / f"{base}.urdf"
            urdf_path.write_text(design.urdf_xml, encoding="utf-8")
            paths["urdf"] = str(urdf_path)

        if design.design_dict and design.domain == "gripper" and TRIMESH_AVAILABLE:
            stl_path = out / f"{base}.stl"
            if self._export_gripper_stl(design.design_dict, str(stl_path)):
                paths["stl"] = str(stl_path)
        elif design.mesh_path and Path(design.mesh_path).exists():
            paths["stl"] = design.mesh_path

        json_path = out / f"{base}.json"
        json_path.write_text(
            json.dumps(design.to_dict(), indent=2, default=str),
            encoding="utf-8",
        )
        paths["json"] = str(json_path)

        return paths

    def _export_gripper_stl(self, design_dict: Dict, output_path: str) -> bool:
        """Generate gripper STL from design dict using trimesh (simplified)."""
        if not TRIMESH_AVAILABLE:
            return False
        try:
            mesh = _gripper_mesh_from_design(design_dict)
            if mesh is not None:
                mesh.export(output_path)
                return True
        except Exception:
            pass
        return False


def _gripper_mesh_from_design(design: Dict):
    """Build simple gripper mesh from design dict (palm + finger capsules)."""
    if not TRIMESH_AVAILABLE:
        return None
    finger_list = design.get("finger_designs") or [{}]
    fd = finger_list[0] if finger_list else {}
    num_fingers = design.get("num_fingers", 2)
    palm_r = design.get("palm_radius_mm", 25) / 1000
    palm_h = design.get("palm_thickness_mm", 10) / 1000
    length_m = fd.get("length_mm", 50) / 1000
    width_m = fd.get("width_mm", 12) / 1000
    taper = fd.get("taper_ratio", 0.6)
    num_seg = fd.get("num_segments", 4)
    seg_len = length_m / num_seg
    base_r = width_m / 2

    meshes = []
    palm = trimesh.creation.cylinder(radius=palm_r, height=palm_h)
    meshes.append(palm)

    import math
    for i in range(num_fingers):
        angle = 2 * math.pi * i / num_fingers
        cx = palm_r * 0.65 * math.cos(angle)
        cy = palm_r * 0.65 * math.sin(angle)
        for s in range(num_seg):
            r = base_r * (1.0 - s * (1.0 - taper) / num_seg) * 0.9
            cap = trimesh.creation.capsule(height=seg_len * 0.9, radius=r)
            z = palm_h / 2 + 0.001 + s * seg_len
            cap.apply_translation([cx, cy, z])
            meshes.append(cap)

    return trimesh.util.concatenate(meshes) if meshes else None
