"""
Translation Gap Finder - Find discoveries stuck in translation

Many discoveries die between paper and clinic/product.
Reasons:
- Technology wasn't ready
- Funding gaps
- Regulatory barriers
- No commercial path
- Key blocker unsolved

Some of these blockers may now be solved.
"""
import re
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional

from .claim_extractor import Claim, ClaimType


class TranslationStage(Enum):
    DISCOVERY = "discovery"
    PROOF_OF_CONCEPT = "proof_of_concept"
    PROTOTYPE = "prototype"
    PRECLINICAL = "preclinical"
    CLINICAL_TRIAL = "clinical_trial"
    APPROVED = "approved"
    COMMERCIAL = "commercial"


class BlockerType(Enum):
    TECHNICAL = "technical"
    REGULATORY = "regulatory"
    FUNDING = "funding"
    MANUFACTURING = "manufacturing"
    COMMERCIAL = "commercial"
    SCIENTIFIC = "scientific"
    SAFETY = "safety"
    EFFICACY = "efficacy"


@dataclass
class TranslationGap:
    discovery: str
    paper: str
    year: Optional[int]
    stage_reached: TranslationStage
    current_status: str
    blocker: Optional[BlockerType]
    blocker_detail: Optional[str]
    revival_potential: float
    revival_reason: Optional[str]
    vrfc_factors: Dict[str, str]


@dataclass
class TranslationReport:
    papers_analyzed: List[str]
    gaps_found: List[TranslationGap]
    revival_candidates: List[TranslationGap]
    by_blocker: Dict[str, List[TranslationGap]]
    by_stage: Dict[str, List[TranslationGap]]


def extract_publication_year(text: str) -> Optional[int]:
    """Extract publication year from paper text."""
    patterns = [
        r"©\s*(\d{4})",
        r"published[:\s]+(\d{4})",
        r"received[:\s]+\w+\s+\d+,?\s+(\d{4})",
        r"\((\d{4})\)",
    ]
    current_year = datetime.now().year
    for pattern in patterns:
        matches = re.findall(pattern, text, re.IGNORECASE)
        for match in matches:
            try:
                year = int(match)
                if 1990 <= year <= current_year:
                    return year
            except (ValueError, TypeError):
                continue
    return None


def assess_translation_stage(claims: List[Claim], text: str) -> TranslationStage:
    """Determine how far the research has translated."""
    text_lower = text.lower()
    if any(w in text_lower for w in ["fda approved", "ce marked", "commercially available", "on the market"]):
        return TranslationStage.COMMERCIAL
    if any(w in text_lower for w in ["fda approval", "regulatory approval", "cleared for"]):
        return TranslationStage.APPROVED
    if any(w in text_lower for w in ["clinical trial", "phase i", "phase ii", "phase iii", "human subjects", "patient study"]):
        return TranslationStage.CLINICAL_TRIAL
    if any(w in text_lower for w in ["animal study", "in vivo", "mouse model", "rat model", "preclinical"]):
        return TranslationStage.PRECLINICAL
    if any(w in text_lower for w in ["prototype", "demonstrator", "working system", "fabricated"]):
        return TranslationStage.PROTOTYPE
    if any(w in text_lower for w in ["proof of concept", "feasibility", "demonstrated", "showed that"]):
        return TranslationStage.PROOF_OF_CONCEPT
    return TranslationStage.DISCOVERY


def identify_blockers(text: str, claims: List[Claim]) -> tuple:
    """Identify what might be blocking translation."""
    text_lower = text.lower()
    blocker = None
    detail = None
    if any(w in text_lower for w in ["technically challenging", "engineering challenge", "not yet possible", "technology limitation"]):
        blocker = BlockerType.TECHNICAL
        detail = "Technology or engineering barriers"
    elif any(w in text_lower for w in ["difficult to manufacture", "scalability", "production challenge", "fabrication challenge"]):
        blocker = BlockerType.MANUFACTURING
        detail = "Manufacturing or scaling difficulties"
    elif any(w in text_lower for w in ["regulatory", "fda", "approval process", "clinical pathway"]):
        blocker = BlockerType.REGULATORY
        detail = "Regulatory pathway challenges"
    elif any(w in text_lower for w in ["safety concern", "adverse", "toxicity", "biocompatibility issue"]):
        blocker = BlockerType.SAFETY
        detail = "Safety or biocompatibility concerns"
    elif any(w in text_lower for w in ["limited efficacy", "not effective enough", "marginal improvement"]):
        blocker = BlockerType.EFFICACY
        detail = "Insufficient efficacy"
    elif any(w in text_lower for w in ["funding", "investment", "resource", "cost prohibitive"]):
        blocker = BlockerType.FUNDING
        detail = "Funding or resource constraints"
    return blocker, detail


def extract_vrfc_factors(text: str, claims: List[Claim]) -> Dict[str, str]:
    """Extract factors relevant to VRFC assessment."""
    factors: Dict[str, str] = {}
    text_lower = text.lower()
    if "randomized" in text_lower or "rct" in text_lower:
        factors["evidence"] = "RCT level"
    elif "clinical" in text_lower:
        factors["evidence"] = "Clinical study"
    elif "animal" in text_lower or "in vivo" in text_lower:
        factors["evidence"] = "Preclinical"
    else:
        factors["evidence"] = "Bench/in vitro"
    if "fda" in text_lower or "ce mark" in text_lower:
        factors["regulatory"] = "Regulatory pathway discussed"
    else:
        factors["regulatory"] = "No regulatory pathway mentioned"
    if any(w in text_lower for w in ["market", "commercial", "business", "cost-effective"]):
        factors["commercial"] = "Commercial aspects discussed"
    else:
        factors["commercial"] = "No commercial pathway mentioned"
    if any(w in text_lower for w in ["compared to", "alternative", "standard of care", "current approach"]):
        factors["alternatives"] = "Alternatives discussed"
    else:
        factors["alternatives"] = "No comparison to alternatives"
    return factors


def extract_main_discovery(claims: List[Claim], text: str) -> str:
    """Extract the main discovery from the paper."""
    result_claims = [c for c in claims if c.claim_type == ClaimType.RESULT]
    if result_claims:
        return result_claims[0].text[:200]
    if claims:
        return claims[0].text[:200]
    return text[:200]


def determine_current_status(stage: TranslationStage, blocker: Optional[BlockerType]) -> str:
    """Determine current status description."""
    stage_status = {
        TranslationStage.DISCOVERY: "Basic research stage",
        TranslationStage.PROOF_OF_CONCEPT: "Proof of concept demonstrated",
        TranslationStage.PROTOTYPE: "Prototype developed",
        TranslationStage.PRECLINICAL: "Preclinical studies conducted",
        TranslationStage.CLINICAL_TRIAL: "Clinical trials initiated",
        TranslationStage.APPROVED: "Regulatory approval obtained",
        TranslationStage.COMMERCIAL: "Commercially available",
    }
    status = stage_status.get(stage, "Unknown")
    if blocker:
        status += f" — blocked by {blocker.value} issues"
    return status


def assess_revival_potential(
    gap: "TranslationGap",
    current_year: Optional[int] = None,
) -> tuple:
    """Assess if a stalled discovery could be revived now."""
    if current_year is None:
        current_year = datetime.now().year
    potential = 0.5
    reason: Optional[str] = None
    if gap.year:
        age = current_year - gap.year
        if 5 <= age <= 10:
            potential += 0.2
            reason = f"Discovery is {age} years old - technology may have caught up"
        elif age > 15:
            potential -= 0.2
            reason = "May be outdated"
        elif age < 3:
            potential -= 0.1
            reason = "Still recent - blockers may persist"
    stage_bonus = {
        TranslationStage.DISCOVERY: 0,
        TranslationStage.PROOF_OF_CONCEPT: 0.05,
        TranslationStage.PROTOTYPE: 0.1,
        TranslationStage.PRECLINICAL: 0.15,
        TranslationStage.CLINICAL_TRIAL: 0.2,
    }
    potential += stage_bonus.get(gap.stage_reached, 0)
    if gap.blocker == BlockerType.TECHNICAL:
        potential += 0.15
        reason = (reason or "") + "; Technical barriers often overcome with time"
    elif gap.blocker == BlockerType.MANUFACTURING:
        potential += 0.1
        reason = (reason or "") + "; Manufacturing tech has advanced"
    elif gap.blocker == BlockerType.SCIENTIFIC:
        potential -= 0.3
        reason = (reason or "") + "; Fundamental scientific issues harder to resolve"
    elif gap.blocker == BlockerType.SAFETY:
        potential -= 0.1
        reason = (reason or "") + "; Safety concerns require careful re-evaluation"
    if reason and reason.startswith("; "):
        reason = reason[2:].strip()
    return min(max(potential, 0.0), 1.0), reason


def analyze_translation_gap(
    paper_id: str,
    text: str,
    claims: List[Claim],
    use_llm: bool = True,
) -> Optional[TranslationGap]:
    """Analyze a single paper for translation gaps."""
    year = extract_publication_year(text)
    stage = assess_translation_stage(claims, text)
    blocker, blocker_detail = identify_blockers(text, claims)
    vrfc_factors = extract_vrfc_factors(text, claims)
    discovery = extract_main_discovery(claims, text)
    current_status = determine_current_status(stage, blocker)
    gap = TranslationGap(
        discovery=discovery,
        paper=paper_id,
        year=year,
        stage_reached=stage,
        current_status=current_status,
        blocker=blocker,
        blocker_detail=blocker_detail,
        revival_potential=0.5,
        revival_reason=None,
        vrfc_factors=vrfc_factors,
    )
    gap.revival_potential, gap.revival_reason = assess_revival_potential(gap)
    return gap


def find_translation_gaps(
    papers: Dict[str, str],
    claims_by_paper: Dict[str, List[Claim]],
    use_llm: bool = True,
) -> TranslationReport:
    """
    Find translation gaps across multiple papers.
    """
    gaps: List[TranslationGap] = []
    for paper_id, text in papers.items():
        claims = claims_by_paper.get(paper_id, [])
        gap = analyze_translation_gap(paper_id, text, claims, use_llm)
        if gap:
            gaps.append(gap)
    revival_candidates = [g for g in gaps if g.revival_potential > 0.5]
    revival_candidates.sort(key=lambda g: g.revival_potential, reverse=True)
    by_blocker: Dict[str, List[TranslationGap]] = {}
    for gap in gaps:
        key = gap.blocker.value if gap.blocker else "unknown"
        if key not in by_blocker:
            by_blocker[key] = []
        by_blocker[key].append(gap)
    by_stage: Dict[str, List[TranslationGap]] = {}
    for gap in gaps:
        key = gap.stage_reached.value
        if key not in by_stage:
            by_stage[key] = []
        by_stage[key].append(gap)
    return TranslationReport(
        papers_analyzed=list(papers.keys()),
        gaps_found=gaps,
        revival_candidates=revival_candidates,
        by_blocker=by_blocker,
        by_stage=by_stage,
    )


def summarize_translation_gaps(report: TranslationReport) -> str:
    """Generate human-readable summary."""
    lines = [
        "# Translation Gap Analysis",
        "",
        f"**Papers analyzed:** {len(report.papers_analyzed)}",
        f"**Translation gaps found:** {len(report.gaps_found)}",
        f"**Revival candidates:** {len(report.revival_candidates)}",
        "",
        "## By Translation Stage",
        "",
    ]
    for stage, gaps in report.by_stage.items():
        lines.append(f"- **{stage}**: {len(gaps)} papers")
    lines.append("")
    if report.by_blocker:
        lines.append("## By Blocker Type")
        lines.append("")
        for blocker, gaps in report.by_blocker.items():
            lines.append(f"- **{blocker}**: {len(gaps)} papers")
        lines.append("")
    if report.revival_candidates:
        lines.append("## Top Revival Candidates")
        lines.append("")
        for i, gap in enumerate(report.revival_candidates[:5], 1):
            lines.append(f"### {i}. {gap.paper}")
            disc = gap.discovery[:150] + ("..." if len(gap.discovery) > 150 else "")
            lines.append(f"**Discovery:** {disc}")
            lines.append(f"**Year:** {gap.year or 'Unknown'}")
            lines.append(f"**Stage reached:** {gap.stage_reached.value}")
            lines.append(f"**Revival potential:** {gap.revival_potential:.0%}")
            if gap.revival_reason:
                lines.append(f"**Why revive:** {gap.revival_reason}")
            lines.append("")
    return "\n".join(lines)
