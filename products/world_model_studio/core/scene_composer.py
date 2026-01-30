"""
Scene composer - Compose scenes from assets.
SceneComposer: add_surface, add_gripper, add_gripper_from_file, add_object, set_camera, compose(), save().
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple, Set

from .asset_library import load_asset


def _tag(e: ET.Element) -> str:
    """Tag name without namespace."""
    return e.tag.split("}")[-1] if "}" in e.tag else e.tag


def _find_gripper_body_root(worldbody: ET.Element) -> Optional[ET.Element]:
    """
    Find the main gripper body in worldbody: body named "palm" or first body after ground/light/camera.
    Skip geom name="ground", light, camera.
    """
    for child in worldbody:
        tag = _tag(child)
        if tag == "geom" and (child.get("name") or "").lower() == "ground":
            continue
        if tag == "light" or tag == "camera":
            continue
        if tag == "body":
            if (child.get("name") or "").lower() == "palm":
                return child
            return child
    return None


def _geom_names_in_element(e: ET.Element) -> Set[str]:
    """Recursively collect geom name attributes under element."""
    out: Set[str] = set()
    if _tag(e) == "geom":
        n = e.get("name")
        if n:
            out.add(n)
    for child in e:
        out |= _geom_names_in_element(child)
    return out


def _extract_gripper_from_mjcf(
    gripper_xml: str,
) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[str], Optional[str], Set[str]]:
    """
    Parse gripper MJCF with ElementTree.
    Returns (body_xml, default_xml, actuator_xml, tendon_xml, sensor_xml, geom_names).
    body_xml is only the gripper body hierarchy (no ground/light). geom_names from that body.
    """
    try:
        root = ET.fromstring(gripper_xml)
    except ET.ParseError:
        return None, None, None, None, None, set()

    body_xml = None
    default_xml = None
    actuator_xml = None
    tendon_xml = None
    sensor_xml = None
    geom_names: Set[str] = set()

    for elem in root:
        tag = _tag(elem)
        if tag == "worldbody":
            body_el = _find_gripper_body_root(elem)
            if body_el is not None:
                geom_names = _geom_names_in_element(body_el)
                body_el.set("pos", body_el.get("pos", "0 0 0"))
                body_xml = ET.tostring(body_el, encoding="unicode", method="xml")
            break

    for elem in root:
        tag = _tag(elem)
        if tag == "default":
            default_xml = ET.tostring(elem, encoding="unicode", method="xml")
        elif tag == "actuator":
            actuator_xml = ET.tostring(elem, encoding="unicode", method="xml")
        elif tag == "tendon":
            tendon_xml = ET.tostring(elem, encoding="unicode", method="xml")
        elif tag == "sensor":
            sensor_xml = ET.tostring(elem, encoding="unicode", method="xml")

    return body_xml, default_xml, actuator_xml, tendon_xml, sensor_xml, geom_names


def _set_body_name_and_pos(body_xml: str, name: str, pos: List[float]) -> str:
    """Replace root body name and pos in body_xml. Name e.g. gripper_test."""
    pos_str = f"{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}"
    body_xml = re.sub(r'\bname\s*=\s*["\'][^"\']*["\']', f'name="{name}"', body_xml, count=1)
    if re.search(r'\bpos\s*=', body_xml):
        body_xml = re.sub(r'pos\s*=\s*["\'][^"\']*["\']', f'pos="{pos_str}"', body_xml, count=1)
    else:
        body_xml = body_xml.replace("<body ", f'<body pos="{pos_str}" ', 1)
    return body_xml


def _geom_names_in_snippet(snippet: str) -> Set[str]:
    """Extract geom name= from a snippet (surface or object body)."""
    out: Set[str] = set()
    for m in re.finditer(r'<geom\s[^>]*\bname\s*=\s*["\']([^"\']+)["\']', snippet, re.IGNORECASE):
        out.add(m.group(1))
    return out


class SceneComposer:
    """
    Compose MJCF scenes from assets: surfaces, grippers, objects.
    compose() returns combined MJCF; save() writes to file.
    """

    def __init__(self):
        self._surfaces: List[Tuple[str, List[float]]] = []
        self._grippers: List[Tuple[str, List[float]]] = []
        self._gripper_files: List[Tuple[str, List[float], bool]] = []
        self._objects: List[Tuple[str, List[float], float]] = []
        self._camera_pos: Optional[List[float]] = None
        self._camera_target: Optional[List[float]] = None

    def add_surface(self, name: str, pos: Optional[List[float]] = None) -> "SceneComposer":
        """Add surface (table, bin, shelf, conveyor) at pos [x,y,z]. Default [0,0,0]."""
        self._surfaces.append((name, list(pos) if pos else [0.0, 0.0, 0.0]))
        return self

    def add_gripper(self, name: str, pos: Optional[List[float]] = None) -> "SceneComposer":
        """Add gripper from asset library by name. Default pos [0,0,0.3]."""
        self._grippers.append((name, list(pos) if pos else [0.0, 0.0, 0.3]))
        return self

    def add_gripper_from_file(self, file_path: str, pos: Optional[List[float]] = None, mobile: bool = True) -> "SceneComposer":
        """Add gripper from MJCF/XML file path. Default pos [0,0,0.10] so palm/fingers reach egg center (~0.05) after lower. When mobile=True (default), wrap in body with 3 slide joints (x,y,z) and position actuators."""
        self._gripper_files.append((file_path, list(pos) if pos else [0.0, 0.0, 0.10], bool(mobile)))
        return self

    def add_object(self, name: str, pos: Optional[List[float]] = None, mass: float = 0.1) -> "SceneComposer":
        """Add target object at pos with mass. Default pos [0,0,0.05]."""
        self._objects.append((name, list(pos) if pos else [0.0, 0.0, 0.05], float(mass)))
        return self

    def set_camera(self, pos: Optional[List[float]] = None, target: Optional[List[float]] = None) -> "SceneComposer":
        """Set viewer camera pos and target. Default pos [0.8,0.6,0.5], target [0,0,0.1]."""
        self._camera_pos = list(pos) if pos else [0.8, 0.6, 0.5]
        self._camera_target = list(target) if target else [0.0, 0.0, 0.1]
        return self

    def compose(self) -> str:
        """Return combined MJCF string: worldbody with surfaces, grippers, objects; lighting; camera; contact only if geoms exist."""
        lines = [
            '<?xml version="1.0" encoding="utf-8"?>',
            '<mujoco model="composed_scene">',
            '  <compiler angle="radian" autolimits="true"/>',
            '  <option timestep="0.002" gravity="0 0 -9.81"/>',
            '  <worldbody>',
            '    <geom name="ground" type="plane" size="1 1 0.1" condim="3"/>',
            '    <light name="main" pos="0.5 0.5 1.2" dir="0 0 -1" directional="true"/>',
        ]
        scene_geom_names: Set[str] = set()

        cam_pos = self._camera_pos or [0.8, 0.6, 0.5]
        lines.append(f'    <camera name="main" pos="{cam_pos[0]:.3f} {cam_pos[1]:.3f} {cam_pos[2]:.3f}" fovy="60" mode="fixed"/>')

        for name, pos in self._surfaces:
            snippet = load_asset("surfaces", name)
            scene_geom_names |= _geom_names_in_snippet(snippet)
            if "<body " in snippet:
                snippet = snippet.strip()
                if snippet.startswith("<body "):
                    idx = snippet.find(">")
                    head = snippet[:idx]
                    rest = snippet[idx + 1:]
                    if 'pos="' not in head:
                        head = head.replace("<body ", f'<body pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" ', 1)
                    else:
                        head = head.replace('pos="0 0 0"', f'pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}"', 1)
                    snippet = head + ">" + rest
                lines.append("    " + snippet.replace("\n", "\n    "))
            else:
                lines.append(f'    <body name="surface_{name}" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}">')
                lines.append("      " + snippet.replace("\n", "\n      "))
                lines.append("    </body>")

        default_blocks: List[str] = []
        actuator_blocks: List[str] = []
        mobile_actuator_blocks: List[str] = []
        tendon_blocks: List[str] = []
        sensor_blocks: List[str] = []

        def emit_gripper(gripper_xml: str, name: str, pos: List[float], mobile: bool = False) -> None:
            body_xml, def_xml, act_xml, ten_xml, sen_xml, gripper_geoms = _extract_gripper_from_mjcf(gripper_xml)
            if body_xml:
                scene_geom_names.update(gripper_geoms)
                base_name = f"gripper_{name}_base"
                jx, jy, jz = f"gripper_{name}_x", f"gripper_{name}_y", f"gripper_{name}_z"
                if mobile:
                    lines.append(f'    <body name="{base_name}" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}">')
                    lines.append(f'      <joint name="{jx}" type="slide" axis="1 0 0" range="-0.2 0.2"/>')
                    lines.append(f'      <joint name="{jy}" type="slide" axis="0 1 0" range="-0.2 0.2"/>')
                    lines.append(f'      <joint name="{jz}" type="slide" axis="0 0 1" range="-0.1 0.3"/>')
                    body_xml = _set_body_name_and_pos(body_xml, f"gripper_{name}", [0.0, 0.0, 0.0])
                    for ln in body_xml.strip().split("\n"):
                        lines.append("      " + ln)
                    lines.append("    </body>")
                    mobile_actuator_blocks.append(f'    <position name="gripper_{name}_x_act" joint="{jx}" kp="100"/>')
                    mobile_actuator_blocks.append(f'    <position name="gripper_{name}_y_act" joint="{jy}" kp="100"/>')
                    mobile_actuator_blocks.append(f'    <position name="gripper_{name}_z_act" joint="{jz}" kp="100"/>')
                else:
                    body_xml = _set_body_name_and_pos(body_xml, f"gripper_{name}", pos)
                    for ln in body_xml.strip().split("\n"):
                        lines.append("    " + ln)
                if def_xml:
                    default_blocks.append(def_xml)
                if act_xml:
                    actuator_blocks.append(act_xml)
                if ten_xml:
                    tendon_blocks.append(ten_xml)
                if sen_xml:
                    sensor_blocks.append(sen_xml)
            else:
                lines.append(f'    <!-- gripper "{name}": no palm/first body found in MJCF -->')

        for entry in self._gripper_files:
            file_path = entry[0]
            pos = entry[1]
            mobile = entry[2] if len(entry) >= 3 else True
            path = Path(file_path)
            if not path.exists():
                lines.append(f'    <!-- gripper file not found: {file_path} -->')
                continue
            name = path.stem
            gripper_xml = path.read_text(encoding="utf-8", errors="replace")
            emit_gripper(gripper_xml, name, pos, mobile=mobile)

        for name, pos in self._grippers:
            try:
                gripper_xml = load_asset("grippers", name)
            except FileNotFoundError:
                lines.append(f'    <!-- gripper "{name}" not found -->')
                continue
            emit_gripper(gripper_xml, name, pos)

        for name, pos, mass in self._objects:
            snippet = load_asset("objects", name)
            scene_geom_names |= _geom_names_in_snippet(snippet)
            snippet = snippet.strip()
            if "<body " in snippet and "mass=" in snippet:
                snippet = snippet.replace('mass="0.1"', f'mass="{mass:.3f}"', 1)
                snippet = snippet.replace('mass="0.08"', f'mass="{mass:.3f}"', 1)
                snippet = snippet.replace('mass="0.12"', f'mass="{mass:.3f}"', 1)
            if 'pos="0 0 0.05"' in snippet:
                snippet = snippet.replace('pos="0 0 0.05"', f'pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}"', 1)
            lines.append("    " + snippet)

        lines.append("  </worldbody>")

        if default_blocks:
            lines.append("  <default>")
            for blk in default_blocks:
                ln_list = blk.strip().split("\n")
                start, end = 0, len(ln_list)
                if ln_list and re.match(r"^\s*<default\s*>\s*$", ln_list[0]):
                    start = 1
                if ln_list and re.match(r"^\s*</default>\s*$", ln_list[-1]):
                    end = len(ln_list) - 1
                for ln in ln_list[start:end]:
                    if ln.strip():
                        lines.append("    " + ln.strip())
            lines.append("  </default>")
        if tendon_blocks:
            lines.append("  <tendon>")
            for blk in tendon_blocks:
                inner = re.sub(r"^\s*<tendon[^>]*>\s*", "", blk.strip())
                inner = re.sub(r"\s*</tendon>\s*$", "", inner)
                for ln in inner.split("\n"):
                    if ln.strip():
                        lines.append("    " + ln.strip())
            lines.append("  </tendon>")
        if mobile_actuator_blocks or actuator_blocks:
            lines.append("  <actuator>")
            for ln in mobile_actuator_blocks:
                lines.append(ln)
            for blk in actuator_blocks:
                inner = re.sub(r"^\s*<actuator[^>]*>\s*", "", blk.strip())
                inner = re.sub(r"\s*</actuator>\s*$", "", inner)
                for ln in inner.split("\n"):
                    if ln.strip():
                        lines.append("    " + ln.strip())
            lines.append("  </actuator>")
        if sensor_blocks:
            lines.append("  <sensor>")
            for blk in sensor_blocks:
                inner = re.sub(r"^\s*<sensor[^>]*>\s*", "", blk.strip())
                inner = re.sub(r"\s*</sensor>\s*$", "", inner)
                for ln in inner.split("\n"):
                    if ln.strip():
                        lines.append("    " + ln.strip())
            lines.append("  </sensor>")

        lines.append("  <contact>")
        gripper_geoms_for_contact = {g for g in scene_geom_names if "palm" in g.lower() or "finger" in g.lower() or g == "palm_geom"}
        object_geom_names = {"box_geom", "sphere_geom", "cylinder_geom", "egg_geom", "mug_geom"}
        object_geoms_for_contact = scene_geom_names & object_geom_names
        for g1 in gripper_geoms_for_contact:
            for g2 in object_geoms_for_contact:
                if g1 in scene_geom_names and g2 in scene_geom_names:
                    lines.append(f'    <pair name="gripper_object_{g1}_{g2}" geom1="{g1}" geom2="{g2}" condim="3"/>')
        lines.append("  </contact>")
        lines.append("</mujoco>")
        return "\n".join(lines)

    def save(self, filepath: str) -> str:
        """Save composed MJCF to file. Returns path."""
        path = Path(filepath)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.compose(), encoding="utf-8")
        return str(path)
