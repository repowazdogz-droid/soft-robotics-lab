"""
Paper Parser - Main entry point for parsing papers
"""
import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from .pdf_reader import read_pdf, ParsedPaper, PaperSection
from .claim_extractor import extract_claims as extract_claims_fn, Claim, ClaimType
from .data_linker import find_data_links, DatasetLink


@dataclass
class ParsedPaperOutput:
    paper: ParsedPaper
    claims: List[Claim]
    datasets: List[DatasetLink]
    metadata: dict


def _serialize_claim(c: Claim) -> dict:
    return {
        "text": c.text,
        "claim_type": c.claim_type.value if isinstance(c.claim_type, ClaimType) else str(c.claim_type),
        "evidence": c.evidence,
        "confidence": c.confidence,
        "location": c.location,
        "quantified": c.quantified,
        "stats": c.stats,
    }


def _serialize_section(s: PaperSection) -> dict:
    return {
        "title": s.title,
        "content": s.content,
        "page_start": s.page_start,
        "page_end": s.page_end,
    }


def _serialize_paper(p: ParsedPaper) -> dict:
    return {
        "title": p.title,
        "authors": p.authors,
        "abstract": p.abstract,
        "sections": [_serialize_section(s) for s in p.sections],
        "references": p.references,
        "full_text": p.full_text[:50000],
    }


def _serialize_dataset(d: DatasetLink) -> dict:
    return {
        "name": d.name,
        "url": d.url,
        "repository": d.repository,
        "accession": d.accession,
        "data_type": d.data_type,
    }


def parse_paper(
    source: str,
    do_extract_claims: bool = True,
    find_data: bool = True,
    use_llm: bool = True,
) -> ParsedPaperOutput:
    """
    Parse a scientific paper and extract structured information.

    Args:
        source: Path to PDF file
        do_extract_claims: Whether to extract claims
        find_data: Whether to find dataset links
        use_llm: Whether to use LLM for extraction (reserved for future use)
    """
    path = Path(source)
    if not path.exists():
        raise FileNotFoundError(f"PDF not found: {source}")
    paper = read_pdf(str(path))
    claims: List[Claim] = []
    if do_extract_claims:
        claims = extract_claims_fn(paper.full_text, use_llm=use_llm)
    datasets: List[DatasetLink] = []
    if find_data:
        datasets = find_data_links(paper.full_text)
    metadata = {
        "source": str(path),
        "sections_count": len(paper.sections),
        "references_count": len(paper.references),
    }
    return ParsedPaperOutput(
        paper=paper,
        claims=claims,
        datasets=datasets,
        metadata=metadata,
    )


def parse_from_doi(doi: str) -> Optional[ParsedPaperOutput]:
    """
    Download paper from DOI and parse.
    Note: Many DOIs do not provide direct PDF access; this attempts common resolvers.
    """
    import tempfile
    try:
        import requests
    except ImportError:
        return None
    doi = doi.strip()
    if not doi.startswith("10."):
        return None
    # Try to get PDF URL from DOI (unpaywall-style or doi.org)
    url = f"https://doi.org/{doi}"
    headers = {"Accept": "application/vnd.crossref.unixref+xml"}
    try:
        r = requests.get(url, headers=headers, timeout=10)
        r.raise_for_status()
    except Exception:
        pass
    # Fallback: save PDF to data/papers if user has it; for now return None and require PDF path
    return None


def save_parsed(output: ParsedPaperOutput, path: str) -> None:
    """Save parsed output as JSON."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    obj = {
        "paper": _serialize_paper(output.paper),
        "claims": [_serialize_claim(c) for c in output.claims],
        "datasets": [_serialize_dataset(d) for d in output.datasets],
        "metadata": output.metadata,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)


def load_parsed(path: str) -> ParsedPaperOutput:
    """Load previously parsed paper (paper + claims + datasets; Claims as dicts)."""
    path = Path(path)
    with open(path, encoding="utf-8") as f:
        obj = json.load(f)
    from .pdf_reader import PaperSection
    secs = [
        PaperSection(
            title=s["title"],
            content=s["content"],
            page_start=s.get("page_start", 0),
            page_end=s.get("page_end", 0),
        )
        for s in obj["paper"]["sections"]
    ]
    paper = ParsedPaper(
        title=obj["paper"]["title"],
        authors=obj["paper"]["authors"],
        abstract=obj["paper"]["abstract"],
        sections=secs,
        references=obj["paper"]["references"],
        full_text=obj["paper"].get("full_text", ""),
    )
    claims = []
    for c in obj.get("claims", []):
        ct = c.get("claim_type", "result")
        try:
            claim_type = ClaimType(ct)
        except ValueError:
            claim_type = ClaimType.RESULT
        claims.append(
            Claim(
                text=c["text"],
                claim_type=claim_type,
                evidence=c.get("evidence", ""),
                confidence=c.get("confidence", 0.5),
                location=c.get("location", ""),
                quantified=c.get("quantified", False),
                stats=c.get("stats"),
            )
        )
    datasets = [
        DatasetLink(
            name=d["name"],
            url=d["url"],
            repository=d["repository"],
            accession=d["accession"],
            data_type=d["data_type"],
        )
        for d in obj.get("datasets", [])
    ]
    return ParsedPaperOutput(
        paper=paper,
        claims=claims,
        datasets=datasets,
        metadata=obj.get("metadata", {}),
    )
