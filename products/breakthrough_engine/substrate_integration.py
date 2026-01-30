"""
OMEGA Breakthrough Engine — Substrate integration (vector store, knowledge graph, lineage).
"""

from pathlib import Path
from typing import List, Dict, Any, Optional

_PRODUCTS = Path(__file__).resolve().parent.parent
if str(_PRODUCTS) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(_PRODUCTS))

_vector_store = None
_knowledge_graph = None
_lineage_graph = None
_NodeType = None
_EdgeType = None

try:
    from shared.substrate.vector_store import vector_store as _vector_store
except ImportError:
    pass
try:
    from shared.substrate.knowledge_graph import knowledge_graph as _knowledge_graph
    from shared.substrate.knowledge_graph import NodeType as _NodeType
    from shared.substrate.knowledge_graph import EdgeType as _EdgeType
except ImportError:
    pass
try:
    from shared.substrate.lineage import lineage_graph as _lineage_graph
except ImportError:
    pass


def record_to_substrate(hypothesis: Any) -> None:
    """Add hypothesis to knowledge_graph (Hypothesis node) and vector_store('hypotheses', claim, metadata)."""
    if _vector_store is not None:
        try:
            claim = getattr(hypothesis, "claim", "") or ""
            h_id = getattr(hypothesis, "id", "") or ""
            domain = getattr(hypothesis, "domain", "") or "general"
            status = getattr(hypothesis, "status", None)
            status_val = status.value if status else "active"
            _vector_store.add(
                "hypotheses",
                claim,
                metadata={"hypothesis_id": h_id, "domain": domain, "status": status_val},
            )
        except Exception:
            pass
    if _knowledge_graph is not None and _NodeType is not None:
        try:
            h_id = getattr(hypothesis, "id", "") or ""
            claim = getattr(hypothesis, "claim", "") or ""
            domain = getattr(hypothesis, "domain", "") or "general"
            _knowledge_graph.add_node(
                _NodeType.Hypothesis,
                h_id,
                properties={"claim": claim[:500], "domain": domain},
            )
        except Exception:
            pass


def find_similar_hypotheses(claim: str, n: int = 3) -> List[Dict[str, Any]]:
    """Search vector store 'hypotheses' for similar hypotheses. Returns [{id, text, metadata, score}]."""
    if _vector_store is None:
        return []
    try:
        return _vector_store.search("hypotheses", claim, n=n)
    except Exception:
        return []


def link_hypothesis_to_concepts(hyp_id: str, concepts: List[str]) -> None:
    """Link hypothesis to related concept nodes in knowledge graph (related_to edges)."""
    if _knowledge_graph is None or _EdgeType is None:
        return
    try:
        for concept_id in concepts:
            concept_id = (concept_id or "").strip()
            if not concept_id:
                continue
            if not _knowledge_graph.get_node(concept_id):
                _knowledge_graph.add_node(_NodeType.Concept, concept_id, properties={"name": concept_id})
            _knowledge_graph.add_edge(hyp_id, concept_id, _EdgeType.related_to)
    except Exception:
        pass


def get_hypothesis_lineage(hyp_id: str) -> List[Dict[str, Any]]:
    """Get lineage (parents) for hypothesis from lineage graph."""
    if _lineage_graph is None:
        return []
    try:
        return _lineage_graph.get_lineage(hyp_id)
    except Exception:
        return []


def record_lineage_derived(child_hyp_id: str, parent_hyp_id: str, parameters: Optional[Dict[str, Any]] = None) -> None:
    """Record that child hypothesis was derived from parent."""
    if _lineage_graph is None:
        return
    try:
        _lineage_graph.record(child_hyp_id, parent_hyp_id, "derived_from", parameters=parameters or {})
    except Exception:
        pass


def get_related_concepts_from_graph(hyp_id: str) -> List[Dict[str, Any]]:
    """Get related nodes (concepts) for hypothesis from knowledge graph."""
    if _knowledge_graph is None:
        return []
    try:
        return _knowledge_graph.get_related(hyp_id, edge_type=_EdgeType.related_to if _EdgeType else None, direction="both")
    except Exception:
        return []
