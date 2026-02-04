"""
Substrate - Memory & Knowledge Layer for OMEGA Stack

Components:
- Vector Store: Semantic search
- Knowledge Graph: Entity relationships
- Lineage: Provenance tracking
- Cross Connector: Cross-product insights
- Temporal: Time-based queries
"""

try:
    from .vector_store import VectorStore, vector_store
except ImportError:
    VectorStore = None
    vector_store = None

try:
    from .knowledge_graph import KnowledgeGraph, NodeType, EdgeType, knowledge_graph
except ImportError:
    KnowledgeGraph = None
    NodeType = None
    EdgeType = None
    knowledge_graph = None

try:
    from .lineage import LineageGraph, lineage_graph
except ImportError:
    LineageGraph = None
    lineage_graph = None

try:
    from .cross_connector import (
        CrossProductConnector,
        CrossConnection,
        InsightReport,
    )
except ImportError:
    CrossProductConnector = None
    CrossConnection = None
    InsightReport = None

try:
    from .temporal import (
        TemporalQueryEngine,
        BeliefSnapshot,
        BeliefChange,
        BeliefTimeline,
    )
except ImportError:
    TemporalQueryEngine = None
    BeliefSnapshot = None
    BeliefChange = None
    BeliefTimeline = None


def get_substrate() -> dict:
    """Get a configured Substrate instance with all components."""
    components: dict = {}

    if vector_store is not None:
        components["vector_store"] = vector_store
    elif VectorStore is not None:
        try:
            components["vector_store"] = VectorStore()
        except Exception:
            pass

    if knowledge_graph is not None:
        components["knowledge_graph"] = knowledge_graph
    elif KnowledgeGraph is not None:
        try:
            components["knowledge_graph"] = KnowledgeGraph()
        except Exception:
            pass

    if lineage_graph is not None:
        components["lineage"] = lineage_graph
    elif LineageGraph is not None:
        try:
            components["lineage"] = LineageGraph()
        except Exception:
            pass

    if CrossProductConnector is not None:
        try:
            components["cross_connector"] = CrossProductConnector(
                vector_store=components.get("vector_store"),
                knowledge_graph=components.get("knowledge_graph"),
            )
        except Exception:
            pass

    if TemporalQueryEngine is not None:
        try:
            components["temporal"] = TemporalQueryEngine(
                lineage_tracker=components.get("lineage"),
            )
        except Exception:
            pass

    return components


__all__ = [
    "VectorStore",
    "vector_store",
    "KnowledgeGraph",
    "NodeType",
    "EdgeType",
    "knowledge_graph",
    "LineageGraph",
    "lineage_graph",
    "CrossProductConnector",
    "CrossConnection",
    "InsightReport",
    "TemporalQueryEngine",
    "BeliefSnapshot",
    "BeliefChange",
    "BeliefTimeline",
    "get_substrate",
]
