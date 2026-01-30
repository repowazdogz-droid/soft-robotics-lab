"""
Data Linker - Find public dataset links in papers
"""
import re
from dataclasses import dataclass
from typing import List, Optional

REPO_PATTERNS = {
    "GEO": (re.compile(r"GSE\d+", re.I), "https://www.ncbi.nlm.nih.gov/geo/query/acc.cgi?acc={}", "expression"),
    "SRA": (re.compile(r"(?:SRP|SRR|PRJNA)\d+", re.I), "https://www.ncbi.nlm.nih.gov/sra/{}", "sequencing"),
    "Zenodo": (re.compile(r"zenodo\.org/record/(\d+)", re.I), "https://zenodo.org/record/{}", "repository"),
    "GitHub": (re.compile(r"github\.com/([\w.-]+)/([\w.-]+)"), "https://github.com/{}/{}", "code"),
    "Figshare": (re.compile(r"figshare\.com/articles/(\d+)", re.I), "https://figshare.com/articles/{}", "repository"),
    "ArrayExpress": (re.compile(r"E-[A-Z]+-\d+", re.I), "https://www.ebi.ac.uk/arrayexpress/experiments/{}", "expression"),
    "DOI": (re.compile(r"10\.\d{4,}/[^\s]+"), None, "publication"),
}


@dataclass
class DatasetLink:
    name: str
    url: str
    repository: str
    accession: str
    data_type: str


def _build_url(repo: str, match) -> Optional[str]:
    """Build URL from pattern and match."""
    entry = REPO_PATTERNS.get(repo)
    if not entry or entry[1] is None:
        return None
    template = entry[1]
    if repo == "GitHub" and match.lastindex >= 2:
        return template.format(match.group(1), match.group(2))
    if repo == "Zenodo" and match.lastindex >= 1:
        return template.format(match.group(1))
    if repo == "Figshare" and match.lastindex >= 1:
        return template.format(match.group(1))
    return template.format(match.group(0))


def find_data_links(paper_text: str) -> List[DatasetLink]:
    """Find all dataset links in paper."""
    links: List[DatasetLink] = []
    seen = set()
    for repo, (pattern, _, data_type) in REPO_PATTERNS.items():
        for m in pattern.finditer(paper_text):
            acc = m.group(0)
            if acc in seen:
                continue
            seen.add(acc)
            url = _build_url(repo, m)
            if not url and repo == "DOI":
                url = f"https://doi.org/{acc}"
            if not url:
                url = acc
            name = f"{repo}: {acc}"
            links.append(
                DatasetLink(
                    name=name,
                    url=url,
                    repository=repo,
                    accession=acc,
                    data_type=data_type,
                )
            )
    return links


def validate_link(link: DatasetLink) -> bool:
    """Check if link is still accessible (HEAD request)."""
    try:
        import urllib.request
        req = urllib.request.Request(link.url, method="HEAD")
        req.add_header("User-Agent", "OMEGA-Scientist/1.0")
        with urllib.request.urlopen(req, timeout=5) as _:
            return True
    except Exception:
        return False


def fetch_dataset_metadata(link: DatasetLink) -> dict:
    """Get metadata about dataset without downloading."""
    meta = {
        "name": link.name,
        "url": link.url,
        "repository": link.repository,
        "accession": link.accession,
        "data_type": link.data_type,
    }
    try:
        import urllib.request
        req = urllib.request.Request(link.url, method="GET")
        req.add_header("User-Agent", "OMEGA-Scientist/1.0")
        with urllib.request.urlopen(req, timeout=10) as resp:
            meta["status_code"] = resp.status
            meta["content_type"] = resp.headers.get("Content-Type", "")
    except Exception as e:
        meta["error"] = str(e)
    return meta
