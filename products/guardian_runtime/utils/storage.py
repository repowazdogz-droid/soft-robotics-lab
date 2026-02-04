"""
Local audit bundle storage.
"""
import json
from pathlib import Path
from typing import Any, Dict, Optional

from ..schemas.bundle import AuditBundle


def save_bundle(bundle: AuditBundle, directory: str, filename: Optional[str] = None) -> Path:
    """Save audit bundle to a JSON file. Returns path to file."""
    path = Path(directory)
    path.mkdir(parents=True, exist_ok=True)
    if filename is None:
        filename = f"audit_{bundle.session_id}.json"
    filepath = path / filename
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(bundle.json())
    return filepath


def load_bundle(path: str) -> Dict[str, Any]:
    """Load audit bundle from JSON file. Returns dict (bundle is read-only after load)."""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
