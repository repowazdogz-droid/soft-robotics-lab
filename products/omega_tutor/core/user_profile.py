"""
OMEGA Tutor — User profile (age/role) for level-adaptive teaching.
Stored in data/user_profile.json. User never sees "levels", only age or Professional/Researcher.
"""

import json
from pathlib import Path
from typing import Optional, Dict, Any

_DATA_DIR = Path(__file__).resolve().parent.parent / "data"
PROFILE_PATH = _DATA_DIR / "user_profile.json"


def _ensure_data_dir():
    _DATA_DIR.mkdir(parents=True, exist_ok=True)


def load_profile() -> Dict[str, Any]:
    """Load profile from JSON. Returns dict with age, name, role, created_at."""
    if not PROFILE_PATH.exists():
        return {}
    try:
        data = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def save_profile(profile: Dict[str, Any]) -> None:
    """Save profile to JSON."""
    _ensure_data_dir()
    PROFILE_PATH.write_text(json.dumps(profile, indent=2), encoding="utf-8")


def has_profile() -> bool:
    """True if profile exists and has age or role set."""
    p = load_profile()
    return p.get("age") is not None or p.get("role") in ("professional", "researcher")


def get_level(age: int) -> str:
    """
    Map age to internal level. User never sees this string.
    - 5-7 → little
    - 8-11 → kid
    - 12-15 → teen
    - 16-18 → sixth_form
    - 19-25 → undergrad
    - 26-64 → adult
    - 65+ → senior
    """
    if age is None or age < 5:
        return "kid"
    if age <= 7:
        return "little"
    if age <= 11:
        return "kid"
    if age <= 15:
        return "teen"
    if age <= 18:
        return "sixth_form"
    if age <= 25:
        return "undergrad"
    if age <= 64:
        return "adult"
    return "senior"


def get_level_from_profile(profile: Dict[str, Any]) -> str:
    """Resolve level from profile: role (professional/researcher) or age."""
    role = (profile.get("role") or "").strip().lower()
    if role == "professional":
        return "expert"
    if role == "researcher":
        return "researcher"
    age = profile.get("age")
    if age is not None:
        try:
            return get_level(int(age))
        except (TypeError, ValueError):
            pass
    return "adult"


def get_current_level() -> str:
    """Load profile and return current teaching level."""
    profile = load_profile()
    return get_level_from_profile(profile)


def get_display_label(profile: Optional[Dict[str, Any]] = None) -> str:
    """Label for sidebar: 'X years old' or 'Professional' or 'Researcher'."""
    p = profile if profile is not None else load_profile()
    role = (p.get("role") or "").strip().lower()
    if role == "professional":
        return "Professional"
    if role == "researcher":
        return "Researcher"
    age = p.get("age")
    if age is not None:
        try:
            return f"{int(age)} years old"
        except (TypeError, ValueError):
            pass
    return "—"
