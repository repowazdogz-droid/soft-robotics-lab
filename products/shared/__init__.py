"""Shared OMEGA modules (ID generator, contracts, failures, tutor_links, etc.)."""
from .id_generator import (
    generate_id,
    run_id,
    artifact_id,
    validation_id,
    hypothesis_id,
    decision_id,
    error_id,
    bundle_id,
)
from .contracts import Contract, ContractStatus, validate_handoff
from .failures import (
    Severity,
    FailureCode,
    Failure,
    create_failure,
    handle_exception,
)
from .tutor_links import get_tutor_link, get_failure_tutor_link
from .audit import AuditBundle, BundleMetadata, create_bundle
from .demo_pack import get_demo, get_known_good, get_known_bad, get_known_edge, DEMO_DESCRIPTIONS
from .trust import TrustMetrics, TrustScoreTracker, get_trust_score, get_trust_metrics
try:
    from .substrate import (
        VectorStore,
        vector_store,
        KnowledgeGraph,
        NodeType,
        EdgeType,
        knowledge_graph,
        LineageGraph,
        lineage_graph,
    )
except ImportError:
    VectorStore = vector_store = None
    KnowledgeGraph = NodeType = EdgeType = knowledge_graph = None
    LineageGraph = lineage_graph = None

__all__ = [
    "generate_id",
    "run_id",
    "artifact_id",
    "validation_id",
    "hypothesis_id",
    "decision_id",
    "error_id",
    "bundle_id",
    "Contract",
    "ContractStatus",
    "validate_handoff",
    "Severity",
    "FailureCode",
    "Failure",
    "create_failure",
    "handle_exception",
    "get_tutor_link",
    "get_failure_tutor_link",
    "AuditBundle",
    "BundleMetadata",
    "create_bundle",
    "get_demo",
    "get_known_good",
    "get_known_bad",
    "get_known_edge",
    "DEMO_DESCRIPTIONS",
    "TrustMetrics",
    "TrustScoreTracker",
    "get_trust_score",
    "get_trust_metrics",
    "VectorStore",
    "vector_store",
    "KnowledgeGraph",
    "NodeType",
    "EdgeType",
    "knowledge_graph",
    "LineageGraph",
    "lineage_graph",
]
