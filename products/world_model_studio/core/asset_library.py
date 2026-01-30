"""
Asset library - Manage grippers, objects, surfaces.
list_assets(category), load_asset(category, name), import_from_foundry(mjcf_path).
Built-in objects: box, sphere, cylinder, egg, mug.
Built-in surfaces: table, bin, shelf, conveyor.
"""

from pathlib import Path
from typing import Dict, List, Optional

_ASSETS_ROOT = Path(__file__).resolve().parent.parent / "assets"
_OBJECTS = ["box", "sphere", "cylinder", "egg", "mug"]
_SURFACES = ["table", "bin", "shelf", "conveyor"]


def list_assets(category: str) -> List[str]:
    """
    List available assets for category: "grippers", "objects", "surfaces".
    Returns names (built-ins + files in assets/{category}/).
    """
    cat = (category or "").lower().strip()
    if cat == "grippers":
        base = _ASSETS_ROOT / "grippers"
        names = [p.stem for p in base.glob("*.mjcf")] + [p.stem for p in base.glob("*.xml")]
        return sorted(set(names))
    if cat == "objects":
        base = _ASSETS_ROOT / "objects"
        file_names = [p.stem for p in base.glob("*.mjcf")] + [p.stem for p in base.glob("*.xml")]
        return sorted(set(_OBJECTS + file_names))
    if cat == "surfaces":
        base = _ASSETS_ROOT / "surfaces"
        file_names = [p.stem for p in base.glob("*.mjcf")] + [p.stem for p in base.glob("*.xml")]
        return sorted(set(_SURFACES + file_names))
    return []


def load_asset(category: str, name: str) -> str:
    """
    Load asset as MJCF string.
    category: "grippers", "objects", "surfaces".
    For built-in objects/surfaces returns generated MJCF snippet (body/geom only).
    """
    cat = (category or "").lower().strip()
    nm = (name or "").strip().lower()

    if cat == "grippers":
        base = _ASSETS_ROOT / "grippers"
        for ext in (".mjcf", ".xml"):
            p = base / f"{nm}{ext}"
            if p.exists():
                return p.read_text(encoding="utf-8", errors="replace")
        raise FileNotFoundError(f"Gripper not found: {name}")

    if cat == "objects":
        return _object_mjcf(nm)
    if cat == "surfaces":
        return _surface_mjcf(nm)

    raise ValueError(f"Unknown category: {category}")


def _object_mjcf(name: str) -> str:
    """Built-in object geom/body snippets (to embed in worldbody)."""
    if name == "box":
        return '<body name="object_box" pos="0 0 0.05"><freejoint name="box_joint"/><geom name="box_geom" type="box" size="0.03 0.03 0.03" mass="0.1" rgba="0.8 0.4 0.2 1"/></body>'
    if name == "sphere":
        return '<body name="object_sphere" pos="0 0 0.05"><freejoint name="sphere_joint"/><geom name="sphere_geom" type="sphere" size="0.03" mass="0.1" rgba="0.2 0.6 0.9 1"/></body>'
    if name == "cylinder":
        return '<body name="object_cylinder" pos="0 0 0.05"><freejoint name="cyl_joint"/><geom name="cylinder_geom" type="cylinder" size="0.025 0.04" mass="0.1" rgba="0.5 0.7 0.3 1"/></body>'
    if name == "egg":
        return '<body name="object_egg" pos="0 0 0.05"><freejoint name="egg_joint"/><geom name="egg_geom" type="ellipsoid" size="0.02 0.025 0.03" mass="0.08" rgba="0.95 0.95 0.9 1" friction="1.5 0.005 0.0001" condim="4"/></body>'
    if name == "mug":
        return '<body name="object_mug" pos="0 0 0.05"><freejoint name="mug_joint"/><geom name="mug_geom" type="cylinder" size="0.03 0.05" mass="0.12" rgba="0.6 0.3 0.2 1"/></body>'
    base = _ASSETS_ROOT / "objects"
    for ext in (".mjcf", ".xml"):
        p = base / f"{name}{ext}"
        if p.exists():
            return p.read_text(encoding="utf-8", errors="replace")
    raise FileNotFoundError(f"Object not found: {name}")


def _surface_mjcf(name: str) -> str:
    """Built-in surface body snippets (table, bin, shelf, conveyor)."""
    if name == "table":
        return '<body name="surface_table" pos="0 0 0"><geom name="table_geom" type="box" size="0.4 0.3 0.02" pos="0 0 0.02" rgba="0.5 0.45 0.4 1" condim="3" friction="1.2 0.005 0.0001"/></body>'
    if name == "bin":
        return '<body name="surface_bin" pos="0 0 0"><geom name="bin_geom" type="box" size="0.15 0.15 0.08" pos="0 0 0.04" rgba="0.4 0.35 0.5 1" condim="3"/></body>'
    if name == "shelf":
        return '<body name="surface_shelf" pos="0 0 0"><geom name="shelf_geom" type="box" size="0.35 0.25 0.015" pos="0 0 0.1" rgba="0.45 0.4 0.35 1" condim="3"/></body>'
    if name == "conveyor":
        return '<body name="surface_conveyor" pos="0 0 0"><geom name="conveyor_geom" type="box" size="0.5 0.2 0.02" pos="0 0 0.02" rgba="0.55 0.5 0.45 1" condim="3"/></body>'
    base = _ASSETS_ROOT / "surfaces"
    for ext in (".mjcf", ".xml"):
        p = base / f"{name}{ext}"
        if p.exists():
            return p.read_text(encoding="utf-8", errors="replace")
    raise FileNotFoundError(f"Surface not found: {name}")


_FOUNDRY_OUTPUTS = Path(__file__).resolve().parent.parent.parent / "omega_foundry" / "outputs"


def list_foundry_designs() -> List[Dict[str, str]]:
    """
    List recent designs from omega_foundry/outputs.
    Returns list of {id, path, name} for design.mjcf in subdirs and .mjcf/.xml at root.
    """
    out: List[Dict[str, str]] = []
    if not _FOUNDRY_OUTPUTS.exists():
        return out
    for d in sorted(_FOUNDRY_OUTPUTS.iterdir(), key=lambda p: p.stat().st_mtime if p.exists() else 0, reverse=True):
        if d.is_dir():
            for f in ("design.mjcf", "design.xml"):
                p = d / f
                if p.exists():
                    out.append({"id": d.name, "path": str(p), "name": d.name})
                    break
    for f in sorted(_FOUNDRY_OUTPUTS.glob("*.mjcf")) + sorted(_FOUNDRY_OUTPUTS.glob("*.xml")):
        if f.is_file() and f.name not in ("index.json",):
            out.append({"id": f.stem, "path": str(f), "name": f.stem})
    return out[:30]


def import_from_foundry(mjcf_path: str) -> str:
    """
    Copy MJCF from omega_foundry output path to assets/grippers/.
    Returns path to copied file in assets/grippers.
    """
    src = Path(mjcf_path)
    if not src.exists():
        raise FileNotFoundError(f"Foundry file not found: {mjcf_path}")
    dest_dir = _ASSETS_ROOT / "grippers"
    dest_dir.mkdir(parents=True, exist_ok=True)
    stem = src.stem
    dest = dest_dir / f"{stem}.mjcf"
    content = src.read_text(encoding="utf-8", errors="replace")
    dest.write_text(content, encoding="utf-8")
    return str(dest)
