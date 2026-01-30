"""
OMEGA Foundry — Substrate integration (vector store, knowledge graph, lineage).
Store designs, search similar, record lineage (design → derived_from → template/previous).
"""

from pathlib import Path
from typing import Any, Dict, List, Optional

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
except Exception:
    pass
try:
    from shared.substrate.knowledge_graph import knowledge_graph as _knowledge_graph
    from shared.substrate.knowledge_graph import NodeType as _NodeType
    from shared.substrate.knowledge_graph import EdgeType as _EdgeType
except Exception:
    pass
try:
    from shared.substrate.lineage import lineage_graph as _lineage_graph
except Exception:
    pass


def record_design_to_substrate(
    design_id: str,
    mjcf: str,
    metadata: Optional[Dict[str, Any]] = None,
    derived_from_id: Optional[str] = None,
    derived_from_type: Optional[str] = None,
) -> None:
    """
    Store design in vector_store("designs"), add Design node to knowledge graph,
    record lineage (design → derived_from) if derived_from_id given.
    """
    meta = dict(metadata or {})
    meta["design_id"] = design_id
    if _vector_store is not None:
        try:
            text = mjcf[:50000] if mjcf else ""
            _vector_store.add("designs", text, metadata=meta)
        except Exception:
            pass
    if _knowledge_graph is not None and _NodeType is not None:
        try:
            _knowledge_graph.add_node(
                _NodeType.Design,
                design_id,
                properties={"design_id": design_id, **{k: v for k, v in meta.items() if k != "design_id"}},
            )
        except Exception:
            pass
    if _lineage_graph is not None and derived_from_id:
        try:
            _lineage_graph.record(
                design_id,
                derived_from_id,
                derived_from_type or "derived_from",
                parameters={"type": derived_from_type or "template"},
            )
        except Exception:
            pass


def find_similar_designs(description: str, n: int = 3) -> List[Dict[str, Any]]:
    """Search vector store 'designs' for similar designs. Returns [{id, text, metadata, score}]."""
    if _vector_store is None:
        return []
    try:
        return _vector_store.search("designs", description, n=n)
    except Exception:
        return []


def get_design_lineage(design_id: str) -> List[Dict[str, Any]]:
    """Get lineage (parents) for design from lineage graph."""
    if _lineage_graph is None:
        return []
    try:
        return _lineage_graph.get_lineage(design_id)
    except Exception:
        return []


def link_design_to_materials(design_id: str, material_ids: List[str]) -> None:
    """Link design to material nodes in knowledge graph (made_of or related_to)."""
    if _knowledge_graph is None or _EdgeType is None:
        return
    try:
        for mid in material_ids:
            mid = (mid or "").strip()
            if not mid:
                continue
            if not _knowledge_graph.get_node(mid):
                _knowledge_graph.add_node(_NodeType.Material, mid, properties={"name": mid})
            _knowledge_graph.add_edge(design_id, mid, _EdgeType.made_of)
    except Exception:
        pass
