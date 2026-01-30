"""
OMEGA Tutor — Knowledge decay (forgetting curve).
Confidence decays over time; used for review priority.
"""

from datetime import datetime, date
from typing import List, Dict, Any

# Half-life ~7 days: 0.5 = e^(-0.1 * 7)
DEFAULT_DECAY_RATE = 0.1


def calculate_confidence(initial: float, days_since: float, decay_rate: float = DEFAULT_DECAY_RATE) -> float:
    """Exponential decay: confidence = initial * e^(-decay_rate * days)."""
    if days_since <= 0:
        return initial
    import math
    return max(0.0, min(1.0, initial * math.exp(-decay_rate * days_since)))


def _days_since(iso_date: str) -> float:
    """Days since given ISO date string (date or datetime)."""
    if not iso_date:
        return 999.0
    try:
        if "T" in iso_date:
            d = datetime.fromisoformat(iso_date.replace("Z", "+00:00")).date()
        else:
            d = date.fromisoformat(iso_date[:10])
        return (date.today() - d).total_seconds() / 86400.0
    except Exception:
        return 999.0


def get_review_priority(topics: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Sort by: low (decayed) confidence first, then longest since review."""
    today = date.today()
    scored = []
    for t in topics:
        name = t.get("name", t.get("topic", ""))
        conf = float(t.get("confidence", 1.0))
        last = t.get("last_reviewed") or t.get("first_learned") or ""
        days = _days_since(last)
        decayed = calculate_confidence(conf, days)
        scored.append({
            **t,
            "name": name,
            "decayed_confidence": round(decayed, 3),
            "days_since": round(days, 1),
            "priority_score": (1.0 - decayed) * 10 + days / 30,
        })
    scored.sort(key=lambda x: (-x["priority_score"], x["days_since"]))
    return scored
