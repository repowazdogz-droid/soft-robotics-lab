"""
OMEGA Tutor — User profile (persona) for level-adaptive teaching.
Stored in data/user_profile.json. User picks persona: Kid, Student, Professional, Researcher.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Dict, Any

_DATA_DIR = Path(__file__).resolve().parent.parent / "data"
PROFILE_PATH = _DATA_DIR / "user_profile.json"

PERSONA_CONFIG = {
    "kid": {
        "icon": "🧒",
        "label": "Kid Mode",
        "tagline": "Fun explanations with examples",
        "quote": "Explain like I'm 10",
        "level": "kid",
    },
    "student": {
        "icon": "🎓",
        "label": "Student Mode",
        "tagline": "Clear explanations for school/uni",
        "quote": "Help me understand and remember",
        "level": "undergrad",
    },
    "professional": {
        "icon": "💼",
        "label": "Professional Mode",
        "tagline": "Efficient, practical, no fluff",
        "quote": "Give me what I need to know",
        "level": "expert",
    },
    "researcher": {
        "icon": "🔬",
        "label": "Researcher Mode",
        "tagline": "Full depth, cutting edge, OMEGA-MAX",
        "quote": "Show me everything",
        "level": "researcher",
    },
}

VALID_PERSONAS = frozenset(PERSONA_CONFIG)


@dataclass
class UserProfile:
    persona: str = "student"
    created_at: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {"persona": self.persona, "created_at": self.created_at}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "UserProfile":
        return cls(persona=d.get("persona", "student"), created_at=d.get("created_at"))


def _ensure_data_dir():
    _DATA_DIR.mkdir(parents=True, exist_ok=True)


def load_profile() -> Dict[str, Any]:
    """Load profile from JSON. Returns dict with persona, created_at (age/role optional for migration)."""
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


def _migrate_old_profile(profile: Dict[str, Any]) -> str:
    """
    Map old age/role to persona.
    age <= 12 or (role None and age <= 12) → kid; age <= 25 → student;
    role professional → professional; role researcher → researcher; else → student.
    """
    role = (profile.get("role") or "").strip().lower()
    if role == "professional":
        return "professional"
    if role == "researcher":
        return "researcher"
    age = profile.get("age")
    if age is not None:
        try:
            a = int(age)
            if a <= 12:
                return "kid"
            if a <= 25:
                return "student"
        except (TypeError, ValueError):
            pass
    return "student"


def _ensure_persona(profile: Dict[str, Any]) -> Dict[str, Any]:
    """If profile has no persona but has age/role, set persona from migration and persist."""
    if profile.get("persona") in VALID_PERSONAS:
        return profile
    if profile.get("mode") in ("simple", "direct", "technical", "research"):
        persona = {"simple": "kid", "direct": "student", "technical": "professional", "research": "researcher"}.get(profile["mode"], "student")
        updated = {"persona": persona, "name": profile.get("name"), "created_at": profile.get("created_at")}
        save_profile(updated)
        return updated
    persona = _migrate_old_profile(profile)
    created = profile.get("created_at")
    updated = {"persona": persona, "name": profile.get("name"), "created_at": created}
    save_profile(updated)
    return updated


def has_profile() -> bool:
    """True if profile exists and has a persona set (or can be migrated from age/role/mode)."""
    p = load_profile()
    if p.get("persona") in VALID_PERSONAS:
        return True
    if p.get("mode") in ("simple", "direct", "technical", "research"):
        return True
    if p.get("age") is not None or (p.get("role") or "").strip().lower() in ("professional", "researcher"):
        return True
    return False


def get_level(profile: Optional[Dict[str, Any]] = None) -> str:
    """Return internal level from profile persona (kid, undergrad, expert, researcher)."""
    p = profile if profile is not None else load_profile()
    p = _ensure_persona(p) if p else {}
    persona = p.get("persona") or "student"
    return PERSONA_CONFIG.get(persona, PERSONA_CONFIG["student"]).get("level", "undergrad")


def get_current_level() -> str:
    """Load profile and return current teaching level."""
    return get_level(load_profile())


def get_display_label(profile: Optional[Dict[str, Any]] = None) -> str:
    """Sidebar label: e.g. '🧒 Kid Mode'."""
    p = profile if profile is not None else load_profile()
    p = _ensure_persona(p) if p else {}
    persona = p.get("persona") or "student"
    config = PERSONA_CONFIG.get(persona, PERSONA_CONFIG["student"])
    return f"{config.get('icon', '')} {config.get('label', 'Learning')}"
