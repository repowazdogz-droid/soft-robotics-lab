"""
PDF Reader - Extract text from scientific papers
"""
import re
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Tuple

try:
    import fitz  # PyMuPDF
except ImportError:
    fitz = None


@dataclass
class PaperSection:
    title: str
    content: str
    page_start: int
    page_end: int


@dataclass
class ParsedPaper:
    title: str
    authors: List[str]
    abstract: str
    sections: List[PaperSection]
    references: List[str]
    full_text: str


# Common section headers (case-insensitive, may appear at start of line)
SECTION_HEADERS = [
    r"^\s*abstract\s*$",
    r"^\s*introduction\s*$",
    r"^\s*methods?\s*$",
    r"^\s*materials?\s*(?:and\s+methods?)?\s*$",
    r"^\s*results?\s*$",
    r"^\s*discussion\s*$",
    r"^\s*conclusion(s)?\s*$",
    r"^\s*references?\s*$",
    r"^\s*bibliography\s*$",
    r"^\s*supplementary\s*",
    r"^\s*acknowledg(e)?ments?\s*$",
    r"^\s*data\s+availability\s*$",
    r"^\s*author\s+contributions\s*$",
    r"^\s*competing\s+interests\s*$",
    r"^\s*1\.\s+\w+",  # numbered sections
    r"^\s*2\.\s+\w+",
    r"^\s*\d+\.\s+\w+",
]
SECTION_PATTERN = re.compile(
    "|".join(f"({p})" for p in SECTION_HEADERS),
    re.IGNORECASE | re.MULTILINE,
)


def _get_full_text(doc: "fitz.Document") -> str:
    """Extract all text from PDF pages."""
    parts = []
    for i in range(len(doc)):
        page = doc[i]
        parts.append(page.get_text())
    return "\n\n".join(parts)


def _guess_title_and_authors(full_text: str) -> Tuple[str, List[str]]:
    """Heuristic: first non-empty line as title, next few as authors."""
    lines = [ln.strip() for ln in full_text.splitlines() if ln.strip()]
    title = lines[0] if lines else ""
    authors: List[str] = []
    for ln in lines[1:6]:
        if re.match(r"^[\w\s\.\-]+(,\s*[\w\s\.\-]+)*$", ln) and "abstract" not in ln.lower():
            authors.append(ln)
        else:
            break
    return title, authors


def extract_sections(text: str) -> List[PaperSection]:
    """Split paper into sections (Abstract, Methods, Results, etc.)."""
    sections: List[PaperSection] = []
    # Find section boundaries
    positions: List[tuple[int, str]] = [(0, "Preamble")]
    for m in SECTION_PATTERN.finditer(text):
        positions.append((m.start(), m.group(0).strip() or "Section"))
    positions.append((len(text), "End"))

    for i in range(len(positions) - 1):
        start = positions[i][0]
        end = positions[i + 1][0]
        title = positions[i][1]
        content = text[start:end].strip()
        if not content and title == "Preamble":
            continue
        # Page numbers are approximate (we don't have page mapping here)
        sections.append(
            PaperSection(
                title=title,
                content=content,
                page_start=0,
                page_end=0,
            )
        )
    if not sections and text.strip():
        sections.append(
            PaperSection(title="Full text", content=text.strip(), page_start=0, page_end=0)
        )
    return sections


def extract_references(text: str) -> List[str]:
    """Extract reference list (numbered or bulleted lines in References section)."""
    refs: List[str] = []
    in_refs = False
    ref_num = re.compile(r"^\s*\[\d+\]\s*|^\s*\d+\.\s*")
    for line in text.splitlines():
        line_stripped = line.strip()
        if not line_stripped:
            continue
        lower = line_stripped.lower()
        if "references" in lower or "bibliography" in lower:
            in_refs = True
            continue
        if in_refs and ref_num.match(line_stripped):
            refs.append(re.sub(r"^\s*\[\d+\]\s*|^\s*\d+\.\s*", "", line_stripped))
        elif in_refs and line_stripped.endswith(".") and len(line_stripped) > 30:
            refs.append(line_stripped)
    return refs


def _extract_abstract(sections: List[PaperSection]) -> str:
    """Get abstract content from sections."""
    for s in sections:
        if s.title.lower().strip() == "abstract":
            return s.content
    return ""


def read_pdf(path: str) -> ParsedPaper:
    """Extract structured content from PDF."""
    if fitz is None:
        raise ImportError("PyMuPDF (fitz) is required. Install with: pip install PyMuPDF")
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(str(path))
    doc = fitz.open(str(path))
    try:
        full_text = _get_full_text(doc)
        title, authors = _guess_title_and_authors(full_text)
        sections = extract_sections(full_text)
        abstract = _extract_abstract(sections)
        if not abstract and sections:
            abstract = sections[0].content[:2000] if sections[0].title.lower() == "abstract" else ""
        references = extract_references(full_text)
        return ParsedPaper(
            title=title,
            authors=authors,
            abstract=abstract,
            sections=sections,
            references=references,
            full_text=full_text,
        )
    finally:
        doc.close()
