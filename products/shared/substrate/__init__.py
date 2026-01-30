"""OMEGA Substrate — semantic memory, knowledge graph, lineage. All local, no external services."""
from .vector_store import VectorStore, vector_store
from .knowledge_graph import KnowledgeGraph, NodeType, EdgeType, knowledge_graph
from .lineage import LineageGraph, lineage_graph

__all__ = [
    "VectorStore",
    "vector_store",
    "KnowledgeGraph",
    "NodeType",
    "EdgeType",
    "knowledge_graph",
    "LineageGraph",
    "lineage_graph",
]
