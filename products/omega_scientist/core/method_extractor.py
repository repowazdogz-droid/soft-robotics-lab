"""
Method Extractor - Extract methodology from papers
"""
import re
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class MethodBlock:
    name: str
    description: str
    section: str
    keywords: List[str]


METHOD_HEADERS = re.compile(
    r"\b(?:methods?|materials?\s+and\s+methods?|experimental|procedure|protocol)\b",
    re.I,
)


def extract_methods(paper_text: str) -> List[MethodBlock]:
    """Extract methodology blocks from paper text."""
    blocks: List[MethodBlock] = []
    # Find Methods section
    parts = re.split(r"\n\s*(?:Methods?|Materials?\s+and\s+Methods?)\s*\n", paper_text, maxsplit=1, flags=re.I)
    if len(parts) < 2:
        return blocks
    methods_text = parts[1]
    # Split on subheadings (e.g. "2.1 Cell culture", "Statistical analysis")
    subs = re.split(r"\n\s*(?:\d+\.?\d*\s+)?([A-Z][a-z]+(?:\s+[a-z]+)*)\s*\n", methods_text)
    for i, chunk in enumerate(subs):
        chunk = chunk.strip()
        if len(chunk) < 50:
            continue
        name = "Methods" if i == 0 else (subs[i - 1] if i > 0 else "Methods")
        keywords = _extract_keywords(chunk)
        blocks.append(
            MethodBlock(
                name=name[:100],
                description=chunk[:2000],
                section="Methods",
                keywords=keywords[:20],
            )
        )
    if not blocks:
        blocks.append(
            MethodBlock(
                name="Methods",
                description=methods_text[:2000],
                section="Methods",
                keywords=_extract_keywords(methods_text),
            )
        )
    return blocks


def _extract_keywords(text: str) -> List[str]:
    """Simple keyword extraction (repeated nouns / technical terms)."""
    words = re.findall(r"\b[A-Z][a-z]+(?:\s+[a-z]+)?\b|\b[a-z]+(?:ation|ment|ence|ity)\b", text)
    from collections import Counter
    return [w for w, _ in Counter(words).most_common(15)]
