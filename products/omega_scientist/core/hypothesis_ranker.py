"""
Hypothesis Ranker - Score and rank discovered hypotheses

Combines outputs from all discovery modes:
- Contradiction resolutions
- Cross-domain syntheses
- Failure fixes
- Revival candidates

Ranks by: novelty × feasibility × translation potential × impact
"""
import hashlib
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any, Dict, List, Optional


class HypothesisSource(Enum):
    CONTRADICTION = "contradiction"
    CROSS_DOMAIN = "cross_domain"
    FAILURE_FIX = "failure_fix"
    REVIVAL = "revival"
    MANUAL = "manual"


@dataclass
class RankedHypothesis:
    id: str
    claim: str
    source: HypothesisSource
    source_detail: str

    novelty_score: float
    feasibility_score: float
    translation_score: float
    impact_score: float

    overall_score: float
    rank: int = 0

    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    evidence: List[str] = field(default_factory=list)
    next_steps: List[str] = field(default_factory=list)
    falsification_cost: str = "medium"

    related_papers: List[str] = field(default_factory=list)
    tags: List[str] = field(default_factory=list)


def generate_hypothesis_id() -> str:
    """Generate unique hypothesis ID."""
    return f"H-{hashlib.md5(str(time.time()).encode()).hexdigest()[:8].upper()}"


def score_novelty(hypothesis: dict, source: HypothesisSource) -> float:
    """Score how novel/unexpected the hypothesis is."""
    base = 0.5
    if source == HypothesisSource.CROSS_DOMAIN:
        base += 0.2
    if source == HypothesisSource.REVIVAL:
        base += 0.1
    claim = hypothesis.get("claim", "").lower()
    if any(w in claim for w in ["novel", "new", "first", "unexpected", "surprising"]):
        base += 0.1
    return min(base, 1.0)


def score_feasibility(hypothesis: dict, source: HypothesisSource) -> float:
    """Score technical feasibility (SRFC-like)."""
    base = 0.5
    if source == HypothesisSource.FAILURE_FIX:
        base -= 0.1
    if source == HypothesisSource.CONTRADICTION:
        base -= 0.05
    source_data = hypothesis.get("source_data", {})
    stage = source_data.get("stage_reached", "")
    if stage in ["prototype", "preclinical", "clinical_trial"]:
        base += 0.2
    return min(max(base, 0.1), 1.0)


def score_translation(hypothesis: dict, source: HypothesisSource) -> float:
    """Score translation potential (VRFC-like)."""
    # Try to use real VRFC if available
    try:
        import sys
        from pathlib import Path
        _products = Path(__file__).resolve().parent.parent.parent
        if str(_products) not in sys.path:
            sys.path.insert(0, str(_products))
        from enterprise.decision_brief.decision_brief import _compute_validation_trinity

        claim = hypothesis.get("claim", "")
        if claim:
            _, _, _, _, vrfc, _ = _compute_validation_trinity(
                query=claim,
                params={},
                complete=False,
                domains=["general"],
            )
            if vrfc == "GREEN":
                return 0.8
            if vrfc == "AMBER":
                return 0.5
            return 0.2  # RED
    except Exception:
        pass

    # Heuristic fallback
    base = 0.5
    if source == HypothesisSource.REVIVAL:
        revival_potential = hypothesis.get("source_data", {}).get("revival_potential", 0.5)
        base = revival_potential
    claim = hypothesis.get("claim", "").lower()
    if any(w in claim for w in ["clinical", "therapeutic", "treatment", "patient"]):
        base += 0.1
    if any(w in claim for w in ["commercial", "market", "product"]):
        base += 0.1
    return min(max(base, 0.1), 1.0)


def score_impact(hypothesis: dict, source: HypothesisSource) -> float:
    """Score potential impact if hypothesis is true."""
    base = 0.5
    if source == HypothesisSource.CONTRADICTION:
        severity = hypothesis.get("source_data", {}).get("severity", 0.5)
        base = 0.4 + (severity * 0.4)
    if source == HypothesisSource.CROSS_DOMAIN:
        strength = hypothesis.get("source_data", {}).get("strength", 0.5)
        base = 0.5 + (strength * 0.3)
    claim = hypothesis.get("claim", "").lower()
    if any(w in claim for w in ["significant", "major", "breakthrough", "transformative"]):
        base += 0.15
    return min(max(base, 0.1), 1.0)


def calculate_overall_score(
    novelty: float, feasibility: float, translation: float, impact: float
) -> float:
    """Calculate weighted overall score."""
    weights = {
        "novelty": 0.15,
        "feasibility": 0.35,
        "translation": 0.30,
        "impact": 0.20,
    }
    return (
        novelty * weights["novelty"]
        + feasibility * weights["feasibility"]
        + translation * weights["translation"]
        + impact * weights["impact"]
    )


def estimate_falsification_cost(hypothesis: dict, source: HypothesisSource) -> str:
    """Estimate cost to test/falsify the hypothesis."""
    source_data = hypothesis.get("source_data", {})
    if source == HypothesisSource.REVIVAL:
        stage = source_data.get("stage_reached", "discovery")
        if stage in ["clinical_trial", "approved"]:
            return "high"
        if stage in ["prototype", "preclinical"]:
            return "medium"
        return "low"
    if source == HypothesisSource.CROSS_DOMAIN:
        return "medium"
    if source == HypothesisSource.CONTRADICTION:
        return "medium"
    if source == HypothesisSource.FAILURE_FIX:
        return "medium"
    return "medium"


def generate_next_steps(hypothesis: dict, source: HypothesisSource) -> List[str]:
    """Generate suggested next steps for the hypothesis."""
    if source == HypothesisSource.CONTRADICTION:
        return [
            "Review both source papers in detail",
            "Identify methodological differences",
            "Design experiment to resolve contradiction",
            "Check for recent papers that may have resolved this",
        ]
    if source == HypothesisSource.CROSS_DOMAIN:
        return [
            "Literature review in both domains",
            "Identify domain experts to consult",
            "Design proof-of-concept experiment",
            "Check for existing cross-domain work",
        ]
    if source == HypothesisSource.FAILURE_FIX:
        return [
            "Analyze original failure mode in detail",
            "Design fix approach",
            "Prototype and test fix",
            "Compare to original results",
        ]
    if source == HypothesisSource.REVIVAL:
        return [
            "Check current state of blocking technology",
            "Review recent related work",
            "Assess current regulatory landscape",
            "Identify potential collaborators/funders",
        ]
    return ["Review hypothesis", "Gather evidence", "Design test"]


def rank_hypothesis(hypothesis: dict, source: HypothesisSource) -> RankedHypothesis:
    """Create a fully scored and ranked hypothesis."""
    novelty = score_novelty(hypothesis, source)
    feasibility = score_feasibility(hypothesis, source)
    translation = score_translation(hypothesis, source)
    impact = score_impact(hypothesis, source)
    overall = calculate_overall_score(novelty, feasibility, translation, impact)

    return RankedHypothesis(
        id=generate_hypothesis_id(),
        claim=hypothesis.get("claim", ""),
        source=source,
        source_detail=hypothesis.get("source_detail", ""),
        novelty_score=novelty,
        feasibility_score=feasibility,
        translation_score=translation,
        impact_score=impact,
        overall_score=overall,
        evidence=hypothesis.get("evidence", []),
        next_steps=generate_next_steps(hypothesis, source),
        falsification_cost=estimate_falsification_cost(hypothesis, source),
        related_papers=hypothesis.get("papers", []),
        tags=hypothesis.get("tags", []),
    )


def rank_all_hypotheses(hypotheses: List[dict]) -> List[RankedHypothesis]:
    """Rank all hypotheses and assign ranks."""
    ranked: List[RankedHypothesis] = []
    for h in hypotheses:
        try:
            source = HypothesisSource(h.get("source", "manual"))
        except ValueError:
            source = HypothesisSource.MANUAL
        ranked_h = rank_hypothesis(h, source)
        ranked.append(ranked_h)

    ranked.sort(key=lambda x: x.overall_score, reverse=True)
    for i, h in enumerate(ranked, 1):
        h.rank = i
    return ranked


def hypothesis_to_ledger_format(ranked: RankedHypothesis) -> dict:
    """Convert to format compatible with Hypothesis Ledger."""
    return {
        "id": ranked.id,
        "claim": ranked.claim,
        "domain": "cross-domain" if ranked.source == HypothesisSource.CROSS_DOMAIN else "general",
        "confidence": ranked.overall_score,
        "status": "ACTIVE",
        "srfc_status": (
            "GREEN" if ranked.feasibility_score > 0.6 else "AMBER" if ranked.feasibility_score > 0.3 else "RED"
        ),
        "vrfc_status": (
            "GREEN" if ranked.translation_score > 0.6 else "AMBER" if ranked.translation_score > 0.3 else "RED"
        ),
        "falsification_cost": ranked.falsification_cost,
        "next_step": ranked.next_steps[0] if ranked.next_steps else "Review hypothesis",
        "who_benefits": "",
        "who_loses": "",
        "evidence": ranked.evidence,
        "source": ranked.source.value,
        "created_at": ranked.created_at,
    }


def summarize_rankings(ranked: List[RankedHypothesis]) -> str:
    """Generate summary of ranked hypotheses."""
    lines = [
        "# Hypothesis Rankings",
        "",
        f"**Total hypotheses:** {len(ranked)}",
        "",
        "## Top 10 Hypotheses",
        "",
    ]
    for h in ranked[:10]:
        claim_preview = h.claim[:80] + ("..." if len(h.claim) > 80 else "")
        lines.append(f"### #{h.rank}: {claim_preview}")
        lines.append(f"**Source:** {h.source.value} | **Overall:** {h.overall_score:.0%}")
        lines.append(
            f"- Novelty: {h.novelty_score:.0%} | Feasibility: {h.feasibility_score:.0%} | Translation: {h.translation_score:.0%} | Impact: {h.impact_score:.0%}"
        )
        lines.append(f"- Falsification cost: {h.falsification_cost}")
        lines.append("")
    return "\n".join(lines)
