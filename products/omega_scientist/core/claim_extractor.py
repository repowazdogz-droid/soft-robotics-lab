"""
Claim Extractor - Find specific claims in papers
"""
import re
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum


class ClaimType(Enum):
    RESULT = "result"
    METHOD = "method"
    COMPARISON = "comparison"
    CAUSAL = "causal"
    CORRELATION = "correlation"
    HYPOTHESIS = "hypothesis"


@dataclass
class Claim:
    text: str
    claim_type: ClaimType
    evidence: str
    confidence: float
    location: str
    quantified: bool
    stats: Optional[dict]


# Sentence-level patterns for claim detection
CLAIM_PATTERNS = [
    (re.compile(r"\bwe\s+(?:found|observed|showed|demonstrated|report(?:ed)?)\s+that\b", re.I), ClaimType.RESULT),
    (re.compile(r"\b(?:results?|data)\s+(?:show|indicate|suggest|reveal)\s+that\b", re.I), ClaimType.RESULT),
    (re.compile(r"\bwe\s+(?:used|applied|employed|developed)\s+", re.I), ClaimType.METHOD),
    (re.compile(r"\b(?:method|approach|algorithm)\s+(?:was\s+)?(?:used|applied)\b", re.I), ClaimType.METHOD),
    (re.compile(r"\b(?:outperformed?|better\s+than|higher\s+than|improved\s+over)\b", re.I), ClaimType.COMPARISON),
    (re.compile(r"\b(?:compared\s+to|compared\s+with|vs\.?)\b", re.I), ClaimType.COMPARISON),
    (re.compile(r"\b(?:causes?|led\s+to|resulted\s+in|drives?)\b", re.I), ClaimType.CAUSAL),
    (re.compile(r"\b(?:correlat(?:es?|ion)\s+with|associated\s+with)\b", re.I), ClaimType.CORRELATION),
    (re.compile(r"\b(?:we\s+)?hypothes(?:e|is)\s+that\b", re.I), ClaimType.HYPOTHESIS),
    (re.compile(r"\b(?:we\s+)?(?:propose|suggest)\s+that\b", re.I), ClaimType.HYPOTHESIS),
]

STAT_PATTERNS = {
    "p_value": re.compile(r"\bp\s*[<>=]\s*[\d.e-]+|\bp\s*=\s*[\d.e-]+", re.I),
    "confidence_interval": re.compile(r"\b(?:CI|confidence\s+interval)\s*[\[\(]\s*[\d.-]+\s*,\s*[\d.-]+\s*[\]\)]", re.I),
    "effect_size": re.compile(r"\b(?:d|r|η²|Cohen\'?s?\s+d)\s*=\s*[\d.-]+", re.I),
    "percent": re.compile(r"\b\d+(?:\.\d+)?\s*%|\b\d+(?:\.\d+)?\s*percent\b", re.I),
}


def classify_claim(text: str) -> ClaimType:
    """Classify what type of claim this is."""
    for pattern, ctype in CLAIM_PATTERNS:
        if pattern.search(text):
            return ctype
    return ClaimType.RESULT


def extract_statistics(claim_text: str) -> dict:
    """Extract p-values, confidence intervals, effect sizes."""
    out = {}
    for key, pat in STAT_PATTERNS.items():
        m = pat.search(claim_text)
        if m:
            out[key] = m.group(0).strip()
    return out


def _split_into_sentences(text: str) -> List[str]:
    """Simple sentence split on period/newline."""
    text = re.sub(r"\n+", " ", text)
    parts = re.split(r"(?<=[.!?])\s+", text)
    return [p.strip() for p in parts if len(p.strip()) > 20]


def extract_claims(paper_text: str, use_llm: bool = True) -> List[Claim]:
    """
    Extract claims from paper.
    Uses pattern matching for common claim structures.
    LLM can be wired in later for nuanced extraction.
    """
    claims: List[Claim] = []
    # Use Results and Discussion sections if we have section markers
    sections = re.split(r"\n\s*(?:Results?|Discussion|Methods?|Introduction)\s*\n", paper_text, flags=re.I)
    location = "Full text"
    for i, block in enumerate(sections):
        if re.match(r"^(?:Results?|Discussion)\b", block.strip(), re.I):
            location = "Results/Discussion"
        for sent in _split_into_sentences(block):
            for pattern, ctype in CLAIM_PATTERNS:
                if pattern.search(sent):
                    stats = extract_statistics(sent)
                    quantified = len(stats) > 0
                    confidence = 0.7 if quantified else 0.5
                    claims.append(
                        Claim(
                            text=sent[:500],
                            claim_type=ctype,
                            evidence=sent[:300],
                            confidence=confidence,
                            location=location,
                            quantified=quantified,
                            stats=stats if stats else None,
                        )
                    )
                    break
    # Deduplicate by normalized text
    seen = set()
    unique = []
    for c in claims:
        norm = c.text.strip()[:200]
        if norm not in seen:
            seen.add(norm)
            unique.append(c)
    return unique[:50]
