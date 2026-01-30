"""
OMEGA Scientist - Paper parsing and structured extraction.
"""
from .pdf_reader import read_pdf, ParsedPaper, PaperSection
from .claim_extractor import extract_claims, Claim, ClaimType
from .data_linker import find_data_links, DatasetLink
from .paper_parser import parse_paper, parse_from_doi, ParsedPaperOutput, save_parsed, load_parsed

__all__ = [
    "read_pdf",
    "ParsedPaper",
    "PaperSection",
    "extract_claims",
    "Claim",
    "ClaimType",
    "find_data_links",
    "DatasetLink",
    "parse_paper",
    "parse_from_doi",
    "ParsedPaperOutput",
    "save_parsed",
    "load_parsed",
]
