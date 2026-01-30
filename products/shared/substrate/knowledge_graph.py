"""
OMEGA Substrate — Local knowledge graph with NetworkX.
Data: products/shared/substrate/data/knowledge_graph.json
Node/edge types for robotics, biology, and shared research.
"""

import json
from pathlib import Path
from typing import Optional, Dict, Any, List
from enum import Enum

_SUBSTRATE_DIR = Path(__file__).resolve().parent
_DEFAULT_PERSIST_PATH = _SUBSTRATE_DIR / "data" / "knowledge_graph.json"


class NodeType(str, Enum):
    # Robotics
    Material = "Material"
    Design = "Design"
    Gripper = "Gripper"
    Mechanism = "Mechanism"
    Actuator = "Actuator"
    Sensor = "Sensor"
    # Biology
    Gene = "Gene"
    Protein = "Protein"
    Pathway = "Pathway"
    Organism = "Organism"
    Plasmid = "Plasmid"
    Strain = "Strain"
    # Shared
    Property = "Property"
    Concept = "Concept"
    Hypothesis = "Hypothesis"
    Experiment = "Experiment"
    Validation = "Validation"


class EdgeType(str, Enum):
    # Physical
    made_of = "made_of"
    has_property = "has_property"
    contains = "contains"
    connects_to = "connects_to"
    # Causal
    affects = "affects"
    enables = "enables"
    inhibits = "inhibits"
    requires = "requires"
    # Biological
    encodes = "encodes"
    regulates = "regulates"
    produces = "produces"
    expresses = "expresses"
    binds_to = "binds_to"
    # Research
    validates = "validates"
    contradicts = "contradicts"
    depends_on = "depends_on"
    derived_from = "derived_from"
    related_to = "related_to"


def _serialize(obj: Any) -> Any:
    """JSON-serialize enums and Paths."""
    if isinstance(obj, Enum):
        return obj.value
    if isinstance(obj, Path):
        return str(obj)
    return obj


class KnowledgeGraph:
    """Local knowledge graph with NetworkX. Auto-saves after modifications."""

    def __init__(self, persist_path: Optional[Path] = None):
        import networkx as nx
        self._nx = nx
        self.persist_path = Path(persist_path) if persist_path else _DEFAULT_PERSIST_PATH
        self._g = nx.MultiDiGraph()
        self.load()

    def load(self) -> None:
        """Load graph from JSON."""
        if self.persist_path.exists():
            try:
                data = json.loads(self.persist_path.read_text(encoding="utf-8"))
                self._g = self._nx.node_link_graph(data)
            except Exception:
                self._g = self._nx.MultiDiGraph()
        else:
            self._g = self._nx.MultiDiGraph()

    def save(self) -> None:
        """Persist graph to JSON."""
        self.persist_path.parent.mkdir(parents=True, exist_ok=True)
        data = self._nx.node_link_data(self._g)
        self.persist_path.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")

    def add_node(self, node_type: NodeType, node_id: str, properties: Optional[Dict[str, Any]] = None) -> str:
        """Add or update node. Returns node_id."""
        props = dict(properties or {})
        props["node_type"] = node_type.value
        if self._g.has_node(node_id):
            self._g.nodes[node_id].update(props)
        else:
            self._g.add_node(node_id, **props)
        self.save()
        return node_id

    def add_edge(self, from_id: str, to_id: str, edge_type: EdgeType, properties: Optional[Dict[str, Any]] = None) -> None:
        """Add edge. Creates nodes if missing (without type)."""
        if not self._g.has_node(from_id):
            self._g.add_node(from_id)
        if not self._g.has_node(to_id):
            self._g.add_node(to_id)
        props = dict(properties or {})
        props["edge_type"] = edge_type.value
        self._g.add_edge(from_id, to_id, **props)
        self.save()

    def get_node(self, node_id: str) -> Optional[Dict[str, Any]]:
        """Get node attributes or None."""
        if not self._g.has_node(node_id):
            return None
        return dict(self._g.nodes[node_id])

    def get_related(
        self, node_id: str, edge_type: Optional[EdgeType] = None, direction: str = "both"
    ) -> List[Dict[str, Any]]:
        """Get related nodes. direction: 'out', 'in', or 'both'. Filter by edge_type if set."""
        if not self._g.has_node(node_id):
            return []
        results = []
        if direction in ("out", "both"):
            for _, target, key, data in self._g.out_edges(node_id, keys=True, data=True):
                if edge_type is None or data.get("edge_type") == edge_type.value:
                    node = self.get_node(target)
                    if node:
                        results.append({"node_id": target, "node": node, "edge_type": data.get("edge_type"), "direction": "out"})
        if direction in ("in", "both"):
            for source, _, key, data in self._g.in_edges(node_id, keys=True, data=True):
                if edge_type is None or data.get("edge_type") == edge_type.value:
                    node = self.get_node(source)
                    if node:
                        results.append({"node_id": source, "node": node, "edge_type": data.get("edge_type"), "direction": "in"})
        return results

    def find_path(self, from_id: str, to_id: str) -> List[Dict[str, Any]]:
        """Shortest path as list of edges (from, to, edge_type)."""
        if not self._g.has_node(from_id) or not self._g.has_node(to_id):
            return []
        try:
            path = self._nx.shortest_path(self._g, from_id, to_id)
        except (self._nx.NetworkXNoPath, self._nx.NodeNotFound):
            return []
        edges = []
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            for _, _, data in self._g.edges(u, v, data=True):
                edges.append({"from": u, "to": v, "edge_type": data.get("edge_type")})
                break
        return edges

    def get_subgraph(self, node_id: str, depth: int = 2) -> Dict[str, Any]:
        """Nodes and edges within depth hops. Returns {nodes: [{id, ...}], edges: [{from, to, ...}]}."""
        if not self._g.has_node(node_id):
            return {"nodes": [], "edges": []}
        sub = self._nx.ego_graph(self._g, node_id, radius=depth)
        nodes = [{"id": n, **dict(sub.nodes[n])} for n in sub.nodes()]
        edges = []
        for u, v, key, data in sub.edges(keys=True, data=True):
            edges.append({"from": u, "to": v, **data})
        return {"nodes": nodes, "edges": edges}

    def query(
        self, node_type: Optional[NodeType] = None, properties: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """Find nodes by type and/or property match."""
        props = dict(properties or {})
        if node_type is not None:
            props["node_type"] = node_type.value
        results = []
        for n, data in self._g.nodes(data=True):
            if all(data.get(k) == v for k, v in props.items()):
                results.append({"id": n, **dict(data)})
        return results


knowledge_graph = KnowledgeGraph()
