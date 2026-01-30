"""
OMEGA Tutor — One-step level adjustment for Simpler/Deeper.
Moves toward little (simpler) or researcher (deeper). Profile level unchanged.
"""

from typing import Optional

# Order: simpler (left) → deeper (right)
LEVEL_ORDER = [
    "little",
    "kid",
    "teen",
    "sixth_form",
    "undergrad",
    "adult",
    "senior",
    "expert",
    "researcher",
]


def adjust_level(current_level: str, direction: str) -> Optional[str]:
    """
    Return the next level in the given direction, or None if at limit.
    - direction "simpler" → one step toward little
    - direction "deeper" → one step toward researcher
    """
    direction = (direction or "").lower().strip()
    current = (current_level or "adult").lower().strip()
    if current not in LEVEL_ORDER:
        current = "adult"
    idx = LEVEL_ORDER.index(current)
    if direction == "simpler":
        if idx <= 0:
            return None
        return LEVEL_ORDER[idx - 1]
    if direction == "deeper":
        if idx >= len(LEVEL_ORDER) - 1:
            return None
        return LEVEL_ORDER[idx + 1]
    return current
