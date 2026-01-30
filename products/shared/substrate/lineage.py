"""
OMEGA Substrate — Provenance / lineage tracking.
Data: products/shared/substrate/data/lineage.json
Tracks: Design, Validation, TrainingRun, Experiment, Decision, Synthesis, Model.
"""

import json
from pathlib import Path
from typing import Optional, Dict, Any, List
from datetime import datetime

_SUBSTRATE_DIR = Path(__file__).resolve().parent
_DEFAULT_PERSIST_PATH = _SUBSTRATE_DIR / "data" / "lineage.json"


class LineageGraph:
    """Tracks child–parent provenance. Auto-saves after modifications."""

    def __init__(self, persist_path: Optional[Path] = None):
        self.persist_path = Path(persist_path) if persist_path else _DEFAULT_PERSIST_PATH
        self._nodes: Dict[str, Dict[str, Any]] = {}
        self._edges: List[Dict[str, Any]] = []
        self.load()

    def load(self) -> None:
        """Load from JSON."""
        if self.persist_path.exists():
            try:
                data = json.loads(self.persist_path.read_text(encoding="utf-8"))
                self._nodes = data.get("nodes", {})
                self._edges = data.get("edges", [])
            except Exception:
                self._nodes = {}
                self._edges = []
        else:
            self._nodes = {}
            self._edges = []

    def save(self) -> None:
        """Persist to JSON."""
        self.persist_path.parent.mkdir(parents=True, exist_ok=True)
        self.persist_path.write_text(
            json.dumps({"nodes": self._nodes, "edges": self._edges}, indent=2),
            encoding="utf-8",
        )

    def record(
        self,
        child_id: str,
        parent_id: str,
        method: str,
        parameters: Optional[Dict[str, Any]] = None,
        timestamp: Optional[str] = None,
    ) -> None:
        """Record a lineage link: child_id was produced from parent_id by method."""
        ts = timestamp or datetime.now().isoformat()
        if child_id not in self._nodes:
            self._nodes[child_id] = {"id": child_id, "created": ts}
        if parent_id not in self._nodes:
            self._nodes[parent_id] = {"id": parent_id, "created": ts}
        self._edges.append({
            "child_id": child_id,
            "parent_id": parent_id,
            "method": method,
            "parameters": parameters or {},
            "timestamp": ts,
        })
        self.save()

    def get_parents(self, id: str) -> List[Dict[str, Any]]:
        """Immediate parents of id."""
        return [
            {
                "parent_id": e["parent_id"],
                "method": e["method"],
                "parameters": e.get("parameters", {}),
                "timestamp": e.get("timestamp"),
            }
            for e in self._edges
            if e["child_id"] == id
        ]

    def get_lineage(self, id: str) -> List[Dict[str, Any]]:
        """All ancestors to root (breadth-first)."""
        seen = {id}
        result = []
        frontier = [id]
        while frontier:
            current = frontier.pop(0)
            for e in self._edges:
                if e["child_id"] == current and e["parent_id"] not in seen:
                    seen.add(e["parent_id"])
                    result.append({
                        "parent_id": e["parent_id"],
                        "child_id": e["child_id"],
                        "method": e["method"],
                        "parameters": e.get("parameters", {}),
                        "timestamp": e.get("timestamp"),
                    })
                    frontier.append(e["parent_id"])
        return result

    def get_children(self, id: str) -> List[Dict[str, Any]]:
        """Immediate children of id."""
        return [
            {
                "child_id": e["child_id"],
                "method": e["method"],
                "parameters": e.get("parameters", {}),
                "timestamp": e.get("timestamp"),
            }
            for e in self._edges
            if e["parent_id"] == id
        ]

    def get_descendants(self, id: str) -> List[Dict[str, Any]]:
        """All descendants (breadth-first)."""
        seen = {id}
        result = []
        frontier = [id]
        while frontier:
            current = frontier.pop(0)
            for e in self._edges:
                if e["parent_id"] == current and e["child_id"] not in seen:
                    seen.add(e["child_id"])
                    result.append({
                        "parent_id": e["parent_id"],
                        "child_id": e["child_id"],
                        "method": e["method"],
                        "parameters": e.get("parameters", {}),
                        "timestamp": e.get("timestamp"),
                    })
                    frontier.append(e["child_id"])
        return result

    def get_full_provenance(self, id: str) -> Dict[str, Any]:
        """Complete subgraph for id: all ancestors and descendants. Returns {nodes, edges}."""
        lineage = self.get_lineage(id)
        descendants = self.get_descendants(id)
        node_ids = {id}
        for e in lineage + descendants:
            node_ids.add(e.get("parent_id"))
            node_ids.add(e.get("child_id"))
        nodes = [self._nodes[n] for n in node_ids if n in self._nodes]
        edges = lineage + descendants
        return {"nodes": nodes, "edges": edges}


lineage_graph = LineageGraph()
