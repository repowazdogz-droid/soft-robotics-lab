"""
Ledger Integration - Auto-create hypotheses from OMEGA Scientist discoveries

Connects:
- Contradiction Miner → Hypothesis Ledger
- Cross-Domain Collider → Hypothesis Ledger
- Failure Analyzer → Hypothesis Ledger
- Translation Gap Finder → Hypothesis Ledger
"""
import sys
from pathlib import Path
from typing import List, Dict, Optional, Any

# Add breakthrough_engine directory to path (it has no top-level __init__.py)
_ROOT = Path(__file__).resolve().parent.parent.parent  # products
_BREAKTHROUGH = _ROOT / "breakthrough_engine"
if str(_BREAKTHROUGH) not in sys.path:
    sys.path.insert(0, str(_BREAKTHROUGH))

try:
    from hypothesis_ledger import HypothesisLedger
    LEDGER_AVAILABLE = True
except ImportError:
    LEDGER_AVAILABLE = False
    HypothesisLedger = None


def _claim_text(claim: Any) -> str:
    """Get text from claim (dict with 'text' or object with .text)."""
    if claim is None:
        return ""
    if isinstance(claim, dict):
        return (claim.get("text") or "")[:500]
    return (getattr(claim, "text", "") or str(claim))[:500]


def get_ledger() -> Optional[Any]:
    """Get the hypothesis ledger instance."""
    if not LEDGER_AVAILABLE:
        return None
    return HypothesisLedger()


def create_hypothesis_from_contradiction(
    contradiction: Dict,
    auto_add: bool = False,
) -> Dict:
    """
    Create a hypothesis from a contradiction.

    Args:
        contradiction: Contradiction dict from miner
        auto_add: Whether to automatically add to ledger

    Returns:
        Dict with hypothesis data (and id if added)
    """
    resolution = contradiction.get("resolution_hypothesis") or (
        f"Resolve: {contradiction.get('description', 'Unknown contradiction')}"
    )
    confidence = 0.4 + (contradiction.get("discovery_potential", 0.5) * 0.3)
    hypothesis_data = {
        "claim": resolution,
        "domain": "cross_domain",
        "confidence": min(confidence, 0.95),
        "srfc_status": "AMBER",
        "vrfc_status": "AMBER",
        "falsification_cost": "medium",
        "source": "contradiction_miner",
        "source_papers": [
            contradiction.get("paper_a", ""),
            contradiction.get("paper_b", ""),
        ],
        "next_step": "Review both source papers and design discriminating experiment",
        "evidence": [
            f"Claim A: {(_claim_text(contradiction.get('claim_a')))[:100]}",
            f"Claim B: {(_claim_text(contradiction.get('claim_b')))[:100]}",
        ],
    }

    if auto_add:
        ledger = get_ledger()
        if ledger:
            h = ledger.create(
                claim=hypothesis_data["claim"],
                domain=hypothesis_data["domain"],
                author="omega_scientist",
                srfc_status=hypothesis_data["srfc_status"],
                vrfc_status=hypothesis_data["vrfc_status"],
                falsification_cost=hypothesis_data["falsification_cost"],
                next_step=hypothesis_data["next_step"],
            )
            ledger.update_confidence(h.id, hypothesis_data["confidence"], reason="Initial from OMEGA Scientist")
            hypothesis_data["id"] = h.id
            for ev in hypothesis_data["evidence"]:
                ledger.add_evidence(h.id, ev, "supports", source="omega_scientist")

    return hypothesis_data


def create_hypothesis_from_cross_domain(
    connection: Dict,
    auto_add: bool = False,
) -> Dict:
    """Create a hypothesis from a cross-domain connection."""
    hypothesis = connection.get("hypothesis") or (
        f"Cross-domain synthesis: {connection.get('description', '')}"
    )
    domain_a = connection.get("domain_a", "unknown")
    domain_b = connection.get("domain_b", "unknown")
    if hasattr(domain_a, "value"):
        domain_a = domain_a.value
    if hasattr(domain_b, "value"):
        domain_b = domain_b.value
    confidence = 0.3 + (connection.get("strength", 0.5) * 0.3)
    hypothesis_data = {
        "claim": hypothesis,
        "domain": f"{domain_a}_{domain_b}",
        "confidence": min(confidence, 0.95),
        "srfc_status": "AMBER",
        "vrfc_status": "AMBER",
        "falsification_cost": "medium",
        "source": "cross_domain_collider",
        "source_papers": connection.get("paper_sources", []),
        "next_step": f"Literature review in both {domain_a} and {domain_b}",
        "evidence": [
            f"Connection type: {connection.get('connection_type', '')}",
            f"Concept A: {connection.get('concept_a', '')}",
            f"Concept B: {connection.get('concept_b', '')}",
        ],
    }

    if auto_add:
        ledger = get_ledger()
        if ledger:
            h = ledger.create(
                claim=hypothesis_data["claim"],
                domain=hypothesis_data["domain"],
                author="omega_scientist",
                srfc_status=hypothesis_data["srfc_status"],
                vrfc_status=hypothesis_data["vrfc_status"],
                falsification_cost=hypothesis_data["falsification_cost"],
                next_step=hypothesis_data["next_step"],
            )
            ledger.update_confidence(h.id, hypothesis_data["confidence"], reason="Initial from OMEGA Scientist")
            hypothesis_data["id"] = h.id
            for ev in hypothesis_data["evidence"]:
                ledger.add_evidence(h.id, ev, "supports", source="omega_scientist")

    return hypothesis_data


def create_hypothesis_from_failure(
    failure: Dict,
    auto_add: bool = False,
) -> Dict:
    """Create a hypothesis from a failure fix opportunity."""
    fix_hypothesis = failure.get("fix_hypothesis") or (
        f"Fix: {failure.get('what_failed', 'Unknown failure')}"
    )
    hypothesis_data = {
        "claim": fix_hypothesis,
        "domain": "failure_fix",
        "confidence": 0.35,
        "srfc_status": "AMBER",
        "vrfc_status": "AMBER",
        "falsification_cost": "medium",
        "source": "failure_analyzer",
        "source_papers": [failure.get("paper", "")],
        "next_step": "Design and test the proposed fix",
        "evidence": [
            f"Failure description: {(failure.get('description') or '')[:100]}",
            f"Why it failed: {failure.get('why_failed', 'Unknown')}",
        ],
    }

    if auto_add:
        ledger = get_ledger()
        if ledger:
            h = ledger.create(
                claim=hypothesis_data["claim"],
                domain=hypothesis_data["domain"],
                author="omega_scientist",
                srfc_status=hypothesis_data["srfc_status"],
                vrfc_status=hypothesis_data["vrfc_status"],
                falsification_cost=hypothesis_data["falsification_cost"],
                next_step=hypothesis_data["next_step"],
            )
            ledger.update_confidence(h.id, hypothesis_data["confidence"], reason="Initial from OMEGA Scientist")
            hypothesis_data["id"] = h.id
            for ev in hypothesis_data["evidence"]:
                ledger.add_evidence(h.id, ev, "supports", source="omega_scientist")

    return hypothesis_data


def create_hypothesis_from_revival(
    gap: Dict,
    auto_add: bool = False,
) -> Dict:
    """Create a hypothesis from a translation gap revival candidate."""
    discovery = gap.get("discovery", "Unknown discovery")
    if isinstance(discovery, str):
        claim = f"Revive: {discovery[:150]}"
    else:
        claim = f"Revive: {str(discovery)[:150]}"
    revival_potential = gap.get("revival_potential", 0.5)
    blocker = gap.get("blocker")
    if blocker and hasattr(blocker, "value"):
        blocker = blocker.value
    hypothesis_data = {
        "claim": claim,
        "domain": "revival",
        "confidence": min(revival_potential, 0.95),
        "srfc_status": "AMBER",
        "vrfc_status": "GREEN" if not blocker else "AMBER",
        "falsification_cost": "high",
        "source": "translation_gap_finder",
        "source_papers": [gap.get("paper", "")],
        "next_step": f"Check if blocker ({blocker or 'unknown'}) is now resolved",
        "evidence": [
            f"Original discovery: {str(discovery)[:100]}",
            f"Stage reached: {gap.get('stage_reached', 'unknown')}",
            f"Blocker: {blocker or 'unknown'} - {gap.get('blocker_detail', '')}",
        ],
    }

    if auto_add:
        ledger = get_ledger()
        if ledger:
            h = ledger.create(
                claim=hypothesis_data["claim"],
                domain=hypothesis_data["domain"],
                author="omega_scientist",
                srfc_status=hypothesis_data["srfc_status"],
                vrfc_status=hypothesis_data["vrfc_status"],
                falsification_cost=hypothesis_data["falsification_cost"],
                next_step=hypothesis_data["next_step"],
            )
            ledger.update_confidence(h.id, hypothesis_data["confidence"], reason="Initial from OMEGA Scientist")
            hypothesis_data["id"] = h.id
            for ev in hypothesis_data["evidence"]:
                ledger.add_evidence(h.id, ev, "supports", source="omega_scientist")

    return hypothesis_data


def bulk_create_from_scientist_session(
    session_state: Dict,
    auto_add: bool = False,
) -> Dict[str, Any]:
    """
    Create hypotheses from all discoveries in an OMEGA Scientist session.

    Args:
        session_state: Streamlit session state with discovery reports
        auto_add: Whether to auto-add to ledger

    Returns:
        Dict with lists of created hypotheses by source
    """
    results = {
        "contradictions": [],
        "cross_domain": [],
        "failures": [],
        "revivals": [],
        "total": 0,
    }

    if "contradiction_report" in session_state:
        report = session_state["contradiction_report"]
        for c in getattr(report, "contradictions", [])[:10]:
            d = {
                "resolution_hypothesis": getattr(c, "resolution_hypothesis", None),
                "description": getattr(c, "description", ""),
                "discovery_potential": getattr(c, "discovery_potential", 0.5),
                "paper_a": getattr(c, "paper_a", ""),
                "paper_b": getattr(c, "paper_b", ""),
                "claim_a": getattr(c, "claim_a", None),
                "claim_b": getattr(c, "claim_b", None),
            }
            if isinstance(d["claim_a"], dict):
                pass
            else:
                d["claim_a"] = {"text": getattr(d["claim_a"], "text", "")} if d["claim_a"] else {}
            if isinstance(d["claim_b"], dict):
                pass
            else:
                d["claim_b"] = {"text": getattr(d["claim_b"], "text", "")} if d["claim_b"] else {}
            h = create_hypothesis_from_contradiction(d, auto_add=auto_add)
            results["contradictions"].append(h)

    if "collision_report" in session_state:
        report = session_state["collision_report"]
        for conn in getattr(report, "novel_connections", [])[:10]:
            if getattr(conn, "hypothesis", None):
                h = create_hypothesis_from_cross_domain(
                    {
                        "hypothesis": conn.hypothesis,
                        "description": getattr(conn, "description", ""),
                        "strength": getattr(conn, "strength", 0.5),
                        "domain_a": getattr(conn, "domain_a", "unknown"),
                        "domain_b": getattr(conn, "domain_b", "unknown"),
                        "paper_sources": getattr(conn, "paper_sources", []),
                        "connection_type": getattr(conn, "connection_type", ""),
                        "concept_a": getattr(conn, "concept_a", ""),
                        "concept_b": getattr(conn, "concept_b", ""),
                    },
                    auto_add=auto_add,
                )
                results["cross_domain"].append(h)

    if "failure_report" in session_state:
        report = session_state["failure_report"]
        for f in getattr(report, "fixable_opportunities", [])[:10]:
            if getattr(f, "fix_hypothesis", None):
                h = create_hypothesis_from_failure(
                    {
                        "fix_hypothesis": f.fix_hypothesis,
                        "what_failed": getattr(f, "what_failed", ""),
                        "description": getattr(f, "description", ""),
                        "why_failed": getattr(f, "why_failed", ""),
                        "paper": getattr(f, "paper", ""),
                    },
                    auto_add=auto_add,
                )
                results["failures"].append(h)

    if "translation_report" in session_state:
        report = session_state["translation_report"]
        for gap in getattr(report, "revival_candidates", [])[:10]:
            stage = getattr(gap, "stage_reached", None)
            if stage and hasattr(stage, "value"):
                stage = stage.value
            blocker = getattr(gap, "blocker", None)
            h = create_hypothesis_from_revival(
                {
                    "discovery": getattr(gap, "discovery", ""),
                    "revival_potential": getattr(gap, "revival_potential", 0.5),
                    "paper": getattr(gap, "paper", ""),
                    "stage_reached": stage or "unknown",
                    "blocker": blocker,
                    "blocker_detail": getattr(gap, "blocker_detail", ""),
                },
                auto_add=auto_add,
            )
            results["revivals"].append(h)

    results["total"] = (
        len(results["contradictions"])
        + len(results["cross_domain"])
        + len(results["failures"])
        + len(results["revivals"])
    )
    return results
