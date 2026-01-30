"""
OMEGA Decision Brief — Substrate integration (vector store, knowledge graph, lineage).
"""

from pathlib import Path
from typing import List, Dict, Any, Optional

# Resolve products root for shared
_ENTERPRISE = Path(__file__).resolve().parent.parent
_PRODUCTS = _ENTERPRISE.parent
if str(_PRODUCTS) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(_PRODUCTS))

_vector_store = None
_knowledge_graph = None
_lineage_graph = None
_NodeType = None

try:
    from shared.substrate.vector_store import vector_store as _vector_store
except ImportError:
    pass
try:
    from shared.substrate.knowledge_graph import knowledge_graph as _knowledge_graph
    from shared.substrate.knowledge_graph import NodeType as _NodeType
except ImportError:
    pass
try:
    from shared.substrate.lineage import lineage_graph as _lineage_graph
except ImportError:
    pass


def search_past_decisions(query: str, n: int = 5) -> List[Dict[str, Any]]:
    """Search vector store 'decisions' for similar past decisions. Returns [{id, text, metadata, score}]."""
    if _vector_store is None:
        return []
    try:
        return _vector_store.search("decisions", query, n=n)
    except Exception:
        return []


def get_related_knowledge(query: str, domain: Optional[str] = None, n: int = 5) -> List[str]:
    """Query knowledge graph for relevant concepts; return list of short descriptions."""
    if _knowledge_graph is None:
        return []
    try:
        if _NodeType is not None:
            nodes = _knowledge_graph.query(node_type=_NodeType.Concept)
        else:
            nodes = _knowledge_graph.query()
        out = []
        for node in (nodes or [])[:n]:
            nid = node.get("id", "")
            name = node.get("name") or node.get("label") or nid
            if name:
                out.append(str(name))
        return out[:n]
    except Exception:
        return []


def record_decision(decision_id: str, query: str, domain: str, brief_text: str, inputs: Optional[Dict[str, Any]] = None) -> None:
    """Store decision in vector store and lineage (decision_id → depends_on → inputs)."""
    if _vector_store is not None:
        try:
            _vector_store.add(
                "decisions",
                brief_text,
                metadata={"decision_id": decision_id, "query": query[:500], "domain": domain or "general"},
            )
        except Exception:
            pass
    if _lineage_graph is not None:
        try:
            parent = f"inputs:{query[:100]}"
            _lineage_graph.record(decision_id, parent, "decision", parameters=inputs or {"query": query, "domain": domain})
        except Exception:
            pass
