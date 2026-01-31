"""
OMEGA Scientist - Paper parsing and structured extraction.
"""
from .pdf_reader import read_pdf, ParsedPaper, PaperSection
from .claim_extractor import extract_claims, Claim, ClaimType
from .data_linker import find_data_links, DatasetLink
from .paper_parser import parse_paper, parse_from_doi, ParsedPaperOutput, save_parsed, load_parsed
from .contradiction_miner import (
    find_contradictions,
    ContradictionReport,
    Contradiction,
    ContradictionType,
    summarize_contradictions,
)
from .cross_domain import (
    find_cross_domain_connections,
    CollisionReport,
    DomainConnection,
    DomainType,
    summarize_collisions,
)
from .failure_analyzer import (
    analyze_failures,
    FailureReport,
    Failure,
    FailureType,
    FailurePattern,
    summarize_failures,
)
from .translation_gap import (
    find_translation_gaps,
    TranslationReport,
    TranslationGap,
    TranslationStage,
    BlockerType,
    summarize_translation_gaps,
)
from .hypothesis_ranker import (
    rank_all_hypotheses,
    rank_hypothesis,
    RankedHypothesis,
    HypothesisSource,
    hypothesis_to_ledger_format,
    summarize_rankings,
)

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
    "find_contradictions",
    "ContradictionReport",
    "Contradiction",
    "ContradictionType",
    "summarize_contradictions",
    "find_cross_domain_connections",
    "CollisionReport",
    "DomainConnection",
    "DomainType",
    "summarize_collisions",
    "analyze_failures",
    "FailureReport",
    "Failure",
    "FailureType",
    "FailurePattern",
    "summarize_failures",
    "find_translation_gaps",
    "TranslationReport",
    "TranslationGap",
    "TranslationStage",
    "BlockerType",
    "summarize_translation_gaps",
    "rank_all_hypotheses",
    "rank_hypothesis",
    "RankedHypothesis",
    "HypothesisSource",
    "hypothesis_to_ledger_format",
    "summarize_rankings",
]
