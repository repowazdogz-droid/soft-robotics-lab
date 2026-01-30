"""
OMEGA Decision Brief — Domain-aware decision models.
"""

import json
from pathlib import Path
from typing import Dict, Any, Optional

_DOMAINS_DIR = Path(__file__).resolve().parent


def load_domain_model(domain: str) -> Dict[str, Any]:
    """
    Load pre-built decision model for domain.
    domain: "robotics" | "synthetic_biology" | "research" | "business"
    Returns dict with decision_types, stakeholders, risks, time_horizons, trade_offs.
    """
    domain = (domain or "").strip().lower().replace(" ", "_")
    path = _DOMAINS_DIR / f"{domain}.json"
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def list_domains() -> list:
    """Return list of available domain ids."""
    out = []
    for p in _DOMAINS_DIR.glob("*.json"):
        if p.name.startswith("."):
            continue
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            out.append({"id": d.get("id", p.stem), "name": d.get("name", p.stem)})
        except Exception:
            out.append({"id": p.stem, "name": p.stem})
    return out
