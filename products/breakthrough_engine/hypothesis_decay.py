"""
Hypothesis Decay - Confidence decays without evidence

Hypotheses that sit untested become stale.
- Confidence decays over time without new evidence
- Stale hypotheses flagged for review or kill
- "Zombie hypothesis" detection
"""
import math
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional

DEFAULT_HALF_LIFE_DAYS = 90
STALE_THRESHOLD_DAYS = 60
ZOMBIE_THRESHOLD = 0.2


@dataclass
class DecayResult:
    hypothesis_id: str
    original_confidence: float
    decayed_confidence: float
    days_since_evidence: int
    is_stale: bool
    is_zombie: bool
    recommendation: str


def calculate_decay(
    confidence: float,
    last_evidence_date: Optional[str],
    half_life_days: int = DEFAULT_HALF_LIFE_DAYS,
) -> float:
    """
    Calculate decayed confidence using exponential decay.
    C(t) = C0 * (0.5)^(t/half_life)
    """
    if not last_evidence_date:
        return confidence
    try:
        last_date = datetime.fromisoformat(last_evidence_date.replace("Z", "+00:00"))
        now = datetime.now(last_date.tzinfo) if last_date.tzinfo else datetime.now()
        days_elapsed = (now - last_date).days
        if days_elapsed <= 0:
            return confidence
        decay_factor = math.pow(0.5, days_elapsed / half_life_days)
        return confidence * decay_factor
    except Exception:
        return confidence


def assess_hypothesis_health(
    hypothesis_id: str,
    confidence: float,
    last_evidence_date: Optional[str],
    last_activity_date: Optional[str],
    half_life_days: int = DEFAULT_HALF_LIFE_DAYS,
) -> DecayResult:
    """Assess the health of a hypothesis."""
    decayed = calculate_decay(confidence, last_evidence_date, half_life_days)

    days_since = 0
    if last_evidence_date:
        try:
            last_date = datetime.fromisoformat(last_evidence_date.replace("Z", "+00:00"))
            now = datetime.now(last_date.tzinfo) if last_date.tzinfo else datetime.now()
            days_since = (now - last_date).days
        except Exception:
            pass

    is_stale = False
    if last_activity_date:
        try:
            activity_date = datetime.fromisoformat(last_activity_date.replace("Z", "+00:00"))
            now = datetime.now(activity_date.tzinfo) if activity_date.tzinfo else datetime.now()
            days_inactive = (now - activity_date).days
            is_stale = days_inactive > STALE_THRESHOLD_DAYS
        except Exception:
            pass

    is_zombie = decayed < ZOMBIE_THRESHOLD

    if is_zombie:
        recommendation = "KILL: Confidence too low. Either falsify formally or archive."
    elif is_stale and decayed < 0.5:
        recommendation = "REVIEW: Stale and decaying. Decide: test, pivot, or kill."
    elif is_stale:
        recommendation = "ATTENTION: No recent activity. Add evidence or update status."
    elif decayed < confidence * 0.7:
        recommendation = "DECAYING: Confidence dropping. Gather evidence soon."
    else:
        recommendation = "HEALTHY: Active and confident."

    return DecayResult(
        hypothesis_id=hypothesis_id,
        original_confidence=confidence,
        decayed_confidence=decayed,
        days_since_evidence=days_since,
        is_stale=is_stale,
        is_zombie=is_zombie,
        recommendation=recommendation,
    )


def assess_all_hypotheses(hypotheses: List[Dict]) -> List[DecayResult]:
    """Assess all hypotheses and return sorted by urgency."""
    results = []
    for h in hypotheses:
        result = assess_hypothesis_health(
            hypothesis_id=h.get("id", ""),
            confidence=h.get("confidence", 0.5),
            last_evidence_date=h.get("last_evidence_date") or h.get("updated_at"),
            last_activity_date=h.get("updated_at"),
        )
        results.append(result)

    results.sort(
        key=lambda r: (
            -int(r.is_zombie),
            -int(r.is_stale),
            r.decayed_confidence - r.original_confidence,
        )
    )
    return results


def get_decay_summary(results: List[DecayResult]) -> Dict:
    """Generate summary statistics."""
    zombies = [r for r in results if r.is_zombie]
    stale = [r for r in results if r.is_stale and not r.is_zombie]
    decaying = [
        r
        for r in results
        if r.decayed_confidence < r.original_confidence * 0.9 and not r.is_zombie and not r.is_stale
    ]
    healthy = [r for r in results if r.recommendation == "HEALTHY: Active and confident."]
    return {
        "total": len(results),
        "zombies": len(zombies),
        "stale": len(stale),
        "decaying": len(decaying),
        "healthy": len(healthy),
        "zombie_ids": [r.hypothesis_id for r in zombies],
        "stale_ids": [r.hypothesis_id for r in stale],
        "action_required": len(zombies) + len(stale),
    }
