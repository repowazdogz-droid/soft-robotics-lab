"""
Failure Analyzer - Extract and analyze failure patterns from papers

Most papers report successes. Failures are buried or unpublished.
Mining failure patterns reveals:
- What approaches don't work
- Why they fail
- Conditions under which they fail
- Opportunities to fix them
"""
import json
import re
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

from .claim_extractor import Claim


class FailureType(Enum):
    METHOD_FAILURE = "method_failure"
    CONDITION_FAILURE = "condition_failure"
    PARTIAL_FAILURE = "partial_failure"
    COMPARISON_LOSS = "comparison_loss"
    LIMITATION = "limitation"
    NEGATIVE_RESULT = "negative_result"


@dataclass
class Failure:
    description: str
    failure_type: FailureType
    what_failed: str
    why_failed: Optional[str]
    conditions: Optional[str]
    severity: float
    source_text: str
    paper: str
    fixable: bool
    fix_hypothesis: Optional[str]


@dataclass
class FailurePattern:
    pattern_name: str
    failures: List[Failure]
    frequency: int
    common_cause: Optional[str]
    potential_solution: Optional[str]


@dataclass
class FailureReport:
    papers_analyzed: List[str]
    total_failures: int
    failures: List[Failure]
    patterns: List[FailurePattern]
    fixable_opportunities: List[Failure]


FAILURE_INDICATORS = [
    r"failed to",
    r"did not (work|succeed|achieve|show|demonstrate)",
    r"was not (able|successful|effective)",
    r"unable to",
    r"could not",
    r"no (significant |statistically |)?(difference|effect|improvement|change)",
    r"not (significant|effective|successful)",
    r"negligible",
    r"limitation(s)? (of|include|is|are)",
    r"drawback",
    r"shortcoming",
    r"weakness",
    r"worse than",
    r"inferior to",
    r"outperformed by",
    r"less effective than",
    r"only (partially|partly|somewhat)",
    r"limited (success|effectiveness|improvement)",
    r"except (when|under|in)",
    r"only works (when|if|under)",
    r"fails (when|if|under)",
]

FAILURE_REASONS = [
    r"due to",
    r"because (of)?",
    r"caused by",
    r"attributed to",
    r"result(ed|ing|s)? from",
    r"owing to",
]


def classify_failure(text: str) -> FailureType:
    """Classify the type of failure."""
    text_lower = text.lower()
    if any(w in text_lower for w in ["limitation", "drawback", "shortcoming"]):
        return FailureType.LIMITATION
    if any(w in text_lower for w in ["worse than", "inferior", "outperformed by"]):
        return FailureType.COMPARISON_LOSS
    if any(w in text_lower for w in ["only partially", "limited success"]):
        return FailureType.PARTIAL_FAILURE
    if any(w in text_lower for w in ["only works when", "fails when", "except when", "except under"]):
        return FailureType.CONDITION_FAILURE
    if any(w in text_lower for w in ["no significant", "not significant", "no effect"]):
        return FailureType.NEGATIVE_RESULT
    return FailureType.METHOD_FAILURE


def extract_what_failed(text: str) -> str:
    """Extract what specifically failed."""
    patterns = [
        r"(\w+(?:\s+\w+){0,3})\s+(?:failed|did not|was not|could not)",
        r"(?:the|this|our)\s+(\w+(?:\s+\w+){0,2})\s+(?:failed|did not)",
    ]
    for pattern in patterns:
        match = re.search(pattern, text, re.IGNORECASE)
        if match:
            return match.group(1).strip()
    return "approach"


def extract_why_failed(text: str) -> Optional[str]:
    """Extract reason for failure if stated."""
    for pattern in FAILURE_REASONS:
        match = re.search(rf"{pattern}\s+(.+?)(?:[.,;]|$)", text, re.IGNORECASE | re.DOTALL)
        if match:
            return match.group(1).strip()[:100]
    return None


def extract_conditions(text: str) -> Optional[str]:
    """Extract conditions under which failure occurs."""
    condition_patterns = [
        r"when\s+(.+?)(?:[.,;]|$)",
        r"under\s+(.+?)\s+conditions",
        r"if\s+(.+?)(?:[.,;]|$)",
        r"in\s+(?:the\s+)?case\s+of\s+(.+?)(?:[.,;]|$)",
    ]
    for pattern in condition_patterns:
        match = re.search(pattern, text, re.IGNORECASE | re.DOTALL)
        if match:
            return match.group(1).strip()[:100]
    return None


def calculate_severity(text: str) -> float:
    """Calculate severity of the failure."""
    severity = 0.5
    text_lower = text.lower()
    if any(w in text_lower for w in ["completely", "entirely", "totally", "always"]):
        severity += 0.2
    if any(w in text_lower for w in ["sometimes", "occasionally", "slightly", "marginally"]):
        severity -= 0.2
    if re.search(r"\d+%", text):
        severity += 0.1
    return max(0.1, min(1.0, severity))


def is_potentially_fixable(text: str) -> bool:
    """Determine if the failure might be fixable."""
    text_lower = text.lower()
    unfixable = ["fundamental", "inherent", "impossible", "cannot be"]
    if any(w in text_lower for w in unfixable):
        return False
    fixable = ["could be improved", "future work", "might be", "potentially", "limitation"]
    if any(w in text_lower for w in fixable):
        return True
    return True


def detect_failure_pattern(sentence: str, paper_id: str) -> Optional[Failure]:
    """Detect failure using regex patterns."""
    sentence_lower = sentence.lower()
    for pattern in FAILURE_INDICATORS:
        if re.search(pattern, sentence_lower):
            return Failure(
                description=sentence[:200],
                failure_type=classify_failure(sentence),
                what_failed=extract_what_failed(sentence),
                why_failed=extract_why_failed(sentence),
                conditions=extract_conditions(sentence),
                severity=calculate_severity(sentence),
                source_text=sentence,
                paper=paper_id,
                fixable=is_potentially_fixable(sentence),
                fix_hypothesis=None,
            )
    return None


def generate_fix_hypothesis(failure: Failure) -> str:
    """Generate a hypothesis for how to fix the failure."""
    templates = [
        f"The failure of {failure.what_failed} might be addressed by modifying the approach",
        f"Alternative methods could overcome the limitation of {failure.what_failed}",
        f"With improved {failure.what_failed}, the original goal may be achievable",
        f"The condition causing failure might be mitigated through redesign",
    ]
    if failure.why_failed:
        return f"Addressing '{failure.why_failed}' could resolve the failure"
    if failure.conditions:
        return f"Avoiding or adapting to '{failure.conditions}' might fix this"
    return templates[hash(failure.description) % len(templates)]


def extract_failures_llm(paper_text: str, paper_id: str) -> List[Failure]:
    """Use LLM to extract nuanced failures."""
    try:
        from openai import OpenAI

        client = OpenAI(
            base_url="http://localhost:1234/v1",
            api_key="not-needed",
            timeout=60,
        )
        sample = paper_text[:4000]
        prompt = f"""Analyze this scientific paper excerpt for failures, limitations, and negative results.

Text:
{sample}

Find any:
1. Methods that didn't work
2. Negative results
3. Acknowledged limitations
4. Conditions under which approaches fail

Respond with JSON array (max 5 items):
[{{
    "description": "brief description",
    "what_failed": "the thing that failed",
    "why_failed": "reason if stated, else null",
    "conditions": "conditions if any, else null",
    "severity": 0.0-1.0,
    "fixable": true/false
}}]

Only return the JSON array."""

        response = client.chat.completions.create(
            model="phi-3-mini-4k-instruct",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=500,
            temperature=0.3,
        )

        content = (response.choices[0].message.content or "").strip()
        content = re.sub(r"^```\w*\n?", "", content)
        content = re.sub(r"\n?```\s*$", "", content).strip()
        results = json.loads(content)
        if not isinstance(results, list):
            results = [results] if results else []

        failures = []
        for r in results:
            failures.append(
                Failure(
                    description=r.get("description", ""),
                    failure_type=FailureType.METHOD_FAILURE,
                    what_failed=r.get("what_failed", "approach"),
                    why_failed=r.get("why_failed"),
                    conditions=r.get("conditions"),
                    severity=float(r.get("severity", 0.5)),
                    source_text=r.get("description", ""),
                    paper=paper_id,
                    fixable=r.get("fixable", True),
                    fix_hypothesis=None,
                )
            )
        return failures
    except Exception:
        return []


def extract_failures(
    paper_text: str,
    paper_id: str,
    claims: Optional[List[Claim]] = None,
    use_llm: bool = True,
) -> List[Failure]:
    """
    Extract failures and negative results from paper text.
    """
    failures = []
    sentences = re.split(r"[.!?]+", paper_text)

    for sentence in sentences:
        sentence = sentence.strip()
        if len(sentence) < 20:
            continue
        failure = detect_failure_pattern(sentence, paper_id)
        if failure:
            failures.append(failure)

    if claims:
        for claim in claims:
            if "not" in claim.text.lower() or "fail" in claim.text.lower():
                failure = Failure(
                    description=claim.text[:200],
                    failure_type=classify_failure(claim.text),
                    what_failed=extract_what_failed(claim.text),
                    why_failed=extract_why_failed(claim.text),
                    conditions=extract_conditions(claim.text),
                    severity=0.5,
                    source_text=claim.text,
                    paper=paper_id,
                    fixable=is_potentially_fixable(claim.text),
                    fix_hypothesis=None,
                )
                failures.append(failure)

    if use_llm:
        llm_failures = extract_failures_llm(paper_text, paper_id)
        failures.extend(llm_failures)

    seen = set()
    unique_failures = []
    for f in failures:
        key = f.description[:100]
        if key not in seen:
            seen.add(key)
            unique_failures.append(f)

    for failure in unique_failures:
        if failure.fixable and not failure.fix_hypothesis:
            failure.fix_hypothesis = generate_fix_hypothesis(failure)

    return unique_failures


def find_failure_patterns(failures: List[Failure]) -> List[FailurePattern]:
    """Group failures into patterns."""
    by_subject: Dict[str, List[Failure]] = {}
    for f in failures:
        key = f.what_failed.lower().strip()
        if key not in by_subject:
            by_subject[key] = []
        by_subject[key].append(f)

    patterns = []
    for subject, group in by_subject.items():
        if len(group) >= 2:
            causes = [f.why_failed for f in group if f.why_failed]
            common_cause = causes[0] if causes else None
            patterns.append(
                FailurePattern(
                    pattern_name=f"Failures in {subject}",
                    failures=group,
                    frequency=len(group),
                    common_cause=common_cause,
                    potential_solution=group[0].fix_hypothesis if group else None,
                )
            )
    return sorted(patterns, key=lambda p: p.frequency, reverse=True)


def analyze_failures(
    papers: Dict[str, str],
    claims_by_paper: Optional[Dict[str, List[Claim]]] = None,
    use_llm: bool = True,
) -> FailureReport:
    """
    Analyze failures across multiple papers.
    """
    all_failures: List[Failure] = []
    for paper_id, text in papers.items():
        claims = (claims_by_paper.get(paper_id, []) if claims_by_paper else [])
        failures = extract_failures(text, paper_id, claims, use_llm)
        all_failures.extend(failures)

    patterns = find_failure_patterns(all_failures)
    fixable = [f for f in all_failures if f.fixable]

    return FailureReport(
        papers_analyzed=list(papers.keys()),
        total_failures=len(all_failures),
        failures=all_failures,
        patterns=patterns,
        fixable_opportunities=sorted(fixable, key=lambda f: f.severity, reverse=True),
    )


def summarize_failures(report: FailureReport) -> str:
    """Generate human-readable summary."""
    lines = [
        "# Failure Analysis",
        "",
        f"**Papers analyzed:** {len(report.papers_analyzed)}",
        f"**Total failures found:** {report.total_failures}",
        f"**Fixable opportunities:** {len(report.fixable_opportunities)}",
        "",
    ]

    if report.patterns:
        lines.append("## Failure Patterns")
        lines.append("")
        for p in report.patterns[:5]:
            lines.append(f"### {p.pattern_name} ({p.frequency} occurrences)")
            if p.common_cause:
                lines.append(f"**Common cause:** {p.common_cause}")
            if p.potential_solution:
                lines.append(f"**Potential solution:** {p.potential_solution}")
            lines.append("")

    if report.fixable_opportunities:
        lines.append("## Top Fixable Opportunities")
        lines.append("")
        for f in report.fixable_opportunities[:5]:
            lines.append(f"- **{f.what_failed}** (severity: {f.severity:.0%})")
            desc = f.description[:150] + ("..." if len(f.description) > 150 else "")
            lines.append(f"  - {desc}")
            if f.fix_hypothesis:
                lines.append(f"  - *Fix:* {f.fix_hypothesis}")
            lines.append("")

    return "\n".join(lines)
