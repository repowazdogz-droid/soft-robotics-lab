"""
OMEGA Foundry — Design template loader: list, load, instantiate.
Templates live in products/omega_foundry/templates/{category}/{name}.json.
"""

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

_FOUNDRY_ROOT = Path(__file__).resolve().parent.parent
_TEMPLATES_DIR = _FOUNDRY_ROOT / "templates"


def list_templates(category: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    List available templates. If category is None, list all; else grippers, mechanisms, or enclosures.
    Returns list of {category, name, description, parameters}.
    """
    out: List[Dict[str, Any]] = []
    categories = [category] if category else ["grippers", "mechanisms", "enclosures"]
    for cat in categories:
        dir_path = _TEMPLATES_DIR / cat
        if not dir_path.is_dir():
            continue
        for f in sorted(dir_path.glob("*.json")):
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                out.append({
                    "category": cat,
                    "name": data.get("name", f.stem),
                    "description": data.get("description", ""),
                    "parameters": data.get("parameters", {}),
                    "preview_image": data.get("preview_image"),
                })
            except Exception:
                continue
    return out


def load_template(category: str, name: str) -> Optional[Dict[str, Any]]:
    """
    Load template by category and name. Name can be file stem (e.g. two_finger_pinch) or display name.
    Returns full template dict or None.
    """
    dir_path = _TEMPLATES_DIR / category
    if not dir_path.is_dir():
        return None
    # Try exact filename first
    path = dir_path / f"{name}.json"
    if not path.exists():
        for f in dir_path.glob("*.json"):
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                if data.get("name", f.stem) == name or f.stem == name:
                    return data
            except Exception:
                continue
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def instantiate_template(template: Dict[str, Any], params: Optional[Dict[str, Any]] = None) -> str:
    """
    Instantiate template with given params: build DesignSpec, run DesignEngine.generate, return MJCF string.
    Template must have spec_defaults (domain, scale, material, params). User params override spec_defaults.params.
    """
    params = dict(params or {})
    spec_defaults = template.get("spec_defaults") or {}
    domain = spec_defaults.get("domain", "gripper")
    scale = spec_defaults.get("scale", "medium")
    material = spec_defaults.get("material")
    template_params = dict(spec_defaults.get("params") or {})
    template_params.update(params)

    from .intent_parser import DesignSpec
    from .design_engine import DesignEngine

    spec = DesignSpec(
        domain=domain,
        scale=scale,
        material=material,
        params=template_params,
        target_object=spec_defaults.get("target_object"),
        raw_intent=template.get("description", ""),
    )
    engine = DesignEngine()
    design = engine.generate(spec)
    return design.mjcf_xml or design.urdf_xml or ""


def get_template_by_id(category: str, file_stem: str) -> Optional[Dict[str, Any]]:
    """Load template by category and file stem (e.g. two_finger_pinch)."""
    path = _TEMPLATES_DIR / category / f"{file_stem}.json"
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
