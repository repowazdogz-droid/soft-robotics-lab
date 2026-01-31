"""
Contradiction Miner - Find disagreements between papers

Contradictions are high-value discovery opportunities:
- Paper A says X, Paper B says not-X
- Unresolved = research gap
- Resolved = potential discovery
"""
import json
import re
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

from .claim_extractor import Claim, ClaimType


class ContradictionType(Enum):
    DIRECT = "direct"
    MAGNITUDE = "magnitude"
    MECHANISM = "mechanism"
    CONDITION = "condition"
    METHOD = "method"


@dataclass
class Contradiction:
    claim_a: Claim
    claim_b: Claim
    paper_a: str
    paper_b: str
    contradiction_type: ContradictionType
    severity: float
    description: str
    resolution_hypothesis: Optional[str]
    discovery_potential: float


@dataclass
class ContradictionReport:
    papers_analyzed: List[str]
    total_claims: int
    contradictions: List[Contradiction]
    top_opportunities: List[Contradiction]


def _parse_percent_from_stats(stats: Optional[Dict[str, Any]]) -> Optional[float]:
    """Extract a numeric percent from claim stats (e.g. '45 %' -> 45.0)."""
    if not stats:
        return None
    raw = stats.get("percent") or stats.get("percentage")
    if raw is None:
        return None
    if isinstance(raw, (int, float)):
        return float(raw)
    match = re.search(r"[\d.]+", str(raw))
    return float(match.group(0)) if match else None


def detect_pattern_contradiction(claim_a: Claim, claim_b: Claim) -> Optional[dict]:
    """
    Rule-based contradiction detection.

    Looks for:
    - Negation patterns ("X increases Y" vs "X decreases Y")
    - Magnitude differences (>50% difference in reported values)
    - Opposing conclusions
    """
    text_a = claim_a.text.lower()
    text_b = claim_b.text.lower()

    increase_words = {"increase", "enhance", "improve", "higher", "more", "greater", "upregulate", "activate"}
    decrease_words = {"decrease", "reduce", "lower", "less", "fewer", "downregulate", "inhibit", "suppress"}

    a_increases = any(w in text_a for w in increase_words)
    a_decreases = any(w in text_a for w in decrease_words)
    b_increases = any(w in text_b for w in increase_words)
    b_decreases = any(w in text_b for w in decrease_words)

    if (a_increases and b_decreases) or (a_decreases and b_increases):
        words_a = set(text_a.split())
        words_b = set(text_b.split())
        overlap = len(words_a & words_b) / max(len(words_a | words_b), 1)

        if overlap > 0.3:
            return {
                "type": ContradictionType.DIRECT,
                "severity": 0.8,
                "description": "Opposing effects: one claims increase, other claims decrease",
            }

    if claim_a.quantified and claim_b.quantified and claim_a.stats and claim_b.stats:
        pct_a = _parse_percent_from_stats(claim_a.stats)
        pct_b = _parse_percent_from_stats(claim_b.stats)

        if pct_a is not None and pct_b is not None:
            diff = abs(pct_a - pct_b)
            if diff > 30:
                return {
                    "type": ContradictionType.MAGNITUDE,
                    "severity": min(diff / 100, 1.0),
                    "description": f"Magnitude difference: {pct_a:.0f}% vs {pct_b:.0f}%",
                }

    return None


def generate_resolution_hypothesis(claim_a: Claim, claim_b: Claim) -> str:
    """Generate a hypothesis for how the contradiction might be resolved."""
    templates = [
        "The difference may be due to experimental conditions or methodology",
        "Population or sample differences could explain the discrepancy",
        "The claims may both be correct under different contexts",
        "A confounding variable may explain the apparent contradiction",
        "Measurement techniques may account for the difference",
    ]
    return templates[hash(claim_a.text + claim_b.text) % len(templates)]


def calculate_discovery_potential(claim_a: Claim, claim_b: Claim, severity: float) -> float:
    """Estimate discovery potential if contradiction is resolved."""
    confidence_factor = (claim_a.confidence + claim_b.confidence) / 2
    quantified_bonus = 0.2 if (claim_a.quantified and claim_b.quantified) else 0
    return min(severity * confidence_factor + quantified_bonus, 1.0)


def detect_llm_contradiction(claim_a: Claim, claim_b: Claim) -> Optional[dict]:
    """Use LLM to detect semantic contradictions."""
    try:
        from openai import OpenAI

        client = OpenAI(
            base_url="http://localhost:1234/v1",
            api_key="not-needed",
            timeout=30,
        )

        prompt = f"""Compare these two scientific claims and determine if they contradict each other.

Claim A: {claim_a.text}
Claim B: {claim_b.text}

Respond in JSON format:
{{
    "contradicts": true/false,
    "type": "direct|magnitude|mechanism|condition|method",
    "severity": 0.0-1.0,
    "description": "brief explanation"
}}

Only return the JSON, nothing else."""

        response = client.chat.completions.create(
            model="phi-3-mini-4k-instruct",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=200,
            temperature=0.3,
        )

        raw = response.choices[0].message.content or ""
        raw = re.sub(r"^```(?:json)?\s*", "", raw)
        raw = re.sub(r"\s*```\s*$", "", raw).strip()
        result = json.loads(raw)

        if result.get("contradicts"):
            try:
                ctype = ContradictionType(result.get("type", "direct"))
            except ValueError:
                ctype = ContradictionType.DIRECT
            sev = float(result.get("severity", 0.5))
            return {
                "contradiction_type": ctype,
                "severity": sev,
                "description": result.get("description", "LLM detected contradiction"),
                "resolution_hypothesis": None,
                "discovery_potential": min(sev * 0.8, 1.0),
            }
    except Exception:
        pass
    return None


def compare_claims(
    claim_a: Claim,
    claim_b: Claim,
    paper_a: str,
    paper_b: str,
    use_llm: bool = True,
) -> Optional[Contradiction]:
    """Compare two claims for contradiction. Returns Contradiction if they contradict, None otherwise."""
    if claim_a.claim_type != claim_b.claim_type:
        return None

    contradiction = detect_pattern_contradiction(claim_a, claim_b)
    if contradiction:
        return Contradiction(
            claim_a=claim_a,
            claim_b=claim_b,
            paper_a=paper_a,
            paper_b=paper_b,
            contradiction_type=contradiction["type"],
            severity=contradiction["severity"],
            description=contradiction["description"],
            resolution_hypothesis=generate_resolution_hypothesis(claim_a, claim_b),
            discovery_potential=calculate_discovery_potential(
                claim_a, claim_b, contradiction["severity"]
            ),
        )

    if use_llm:
        contradiction = detect_llm_contradiction(claim_a, claim_b)
        if contradiction:
            return Contradiction(
                claim_a=claim_a,
                claim_b=claim_b,
                paper_a=paper_a,
                paper_b=paper_b,
                contradiction_type=contradiction["contradiction_type"],
                severity=contradiction["severity"],
                description=contradiction["description"],
                resolution_hypothesis=contradiction.get("resolution_hypothesis"),
                discovery_potential=contradiction.get("discovery_potential", contradiction["severity"] * 0.8),
            )

    return None


def find_contradictions(
    claims_by_paper: dict,
    use_llm: bool = True,
) -> ContradictionReport:
    """
    Find contradictions across multiple papers.

    Args:
        claims_by_paper: Dict mapping paper IDs to their claims
        use_llm: Use LLM for semantic comparison

    Returns:
        ContradictionReport with all found contradictions
    """
    contradictions: List[Contradiction] = []
    paper_ids = list(claims_by_paper.keys())

    for i, paper_a in enumerate(paper_ids):
        for paper_b in paper_ids[i + 1 :]:
            claims_a = claims_by_paper[paper_a]
            claims_b = claims_by_paper[paper_b]

            for claim_a in claims_a:
                for claim_b in claims_b:
                    contradiction = compare_claims(
                        claim_a, claim_b, paper_a, paper_b, use_llm
                    )
                    if contradiction:
                        contradictions.append(contradiction)

    contradictions.sort(key=lambda c: c.discovery_potential, reverse=True)

    return ContradictionReport(
        papers_analyzed=paper_ids,
        total_claims=sum(len(c) for c in claims_by_paper.values()),
        contradictions=contradictions,
        top_opportunities=contradictions[:10],
    )


def summarize_contradictions(report: ContradictionReport) -> str:
    """Generate human-readable summary of contradictions."""
    lines = [
        "# Contradiction Analysis",
        "",
        f"**Papers analyzed:** {len(report.papers_analyzed)}",
        f"**Total claims:** {report.total_claims}",
        f"**Contradictions found:** {len(report.contradictions)}",
        "",
        "## Top Discovery Opportunities",
        "",
    ]

    for i, c in enumerate(report.top_opportunities[:5], 1):
        lines.append(f"### {i}. {c.contradiction_type.value.title()} Contradiction")
        lines.append(f"**Paper A:** {c.paper_a}")
        lines.append(f"- Claim: {c.claim_a.text[:200]}...")
        lines.append("")
        lines.append(f"**Paper B:** {c.paper_b}")
        lines.append(f"- Claim: {c.claim_b.text[:200]}...")
        lines.append("")
        lines.append(f"**Analysis:** {c.description}")
        lines.append(f"**Discovery Potential:** {c.discovery_potential:.0%}")
        if c.resolution_hypothesis:
            lines.append(f"**Possible Resolution:** {c.resolution_hypothesis}")
        lines.append("")

    return "\n".join(lines)
