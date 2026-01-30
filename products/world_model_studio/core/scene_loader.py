"""
World Model Studio — Pre-built scene templates.
list_scenes(), load_scene(name), get_scene_objects(name).
Scenes: pick_and_place, bin_picking, conveyor, assembly, inspection.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

_SCENES_ROOT = Path(__file__).resolve().parent.parent / "scenes"
_SCENES_INDEX = _SCENES_ROOT / "index.json"


def list_scenes() -> List[Dict[str, Any]]:
    """
    List available scene templates. Returns list of {name, description, task_type, difficulty, compatible_grippers}.
    """
    out: List[Dict[str, Any]] = []
    if _SCENES_INDEX.exists():
        try:
            data = json.loads(_SCENES_INDEX.read_text(encoding="utf-8"))
            out = data.get("scenes", [])
        except Exception:
            pass
    for f in sorted(_SCENES_ROOT.glob("*.xml")) + sorted(_SCENES_ROOT.glob("*.mjcf")):
        if f.name.startswith(".") or f.name == "index.json":
            continue
        name = f.stem
        if not any(s.get("name") == name or s.get("id") == name for s in out):
            out.append({
                "name": name,
                "id": name,
                "description": f"Scene: {name}",
                "task_type": "pick" if "pick" in name.lower() else "general",
                "difficulty": "medium",
                "compatible_grippers": ["two_finger", "three_finger"],
            })
    return out


def load_scene(name: str) -> Optional[Dict[str, Any]]:
    """
    Load scene by name. Returns dict: mjcf (str), task (dict), config (dict with difficulty, success_criteria).
    """
    path = _SCENES_ROOT / f"{name}.xml"
    if not path.exists():
        path = _SCENES_ROOT / f"{name}.mjcf"
    if not path.exists():
        for s in list_scenes():
            if s.get("name") == name or s.get("id") == name:
                file_name = s.get("file", f"{name}.xml")
                path = _SCENES_ROOT / file_name
                if path.exists():
                    break
                path = _SCENES_ROOT / f"{name}.xml"
                if path.exists():
                    break
                return None
        return None
    mjcf = path.read_text(encoding="utf-8", errors="replace")
    config_path = _SCENES_ROOT / f"{path.stem}.json"
    task = {}
    config = {"difficulty": "medium", "success_criteria": {"min_height": 0.1}}
    if config_path.exists():
        try:
            data = json.loads(config_path.read_text(encoding="utf-8"))
            task = data.get("task", task)
            config = data.get("config", config)
        except Exception:
            pass
    return {"mjcf": mjcf, "task": task, "config": config, "path": str(path)}


def get_scene_objects(name: str) -> List[str]:
    """Return list of object body/geom names in the scene (e.g. object_box, object_egg)."""
    scene = load_scene(name)
    if not scene or not scene.get("mjcf"):
        return []
    import re
    mjcf = scene["mjcf"]
    objects = []
    for m in re.finditer(r'<body\s+[^>]*\bname\s*=\s*["\']([^"\']+)["\']', mjcf):
        n = m.group(1)
        if "object" in n.lower() or "geom" in n.lower():
            objects.append(n)
    for m in re.finditer(r'<geom\s+[^>]*\bname\s*=\s*["\']([^"\']+)["\']', mjcf):
        n = m.group(1)
        if "box" in n or "egg" in n or "sphere" in n or "cylinder" in n:
            objects.append(n)
    return list(dict.fromkeys(objects))
