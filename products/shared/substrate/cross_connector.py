"""
Cross-Product Connector - Find unexpected connections across OMEGA products

Discovers:
- Related items across different products
- Unexpected connections between concepts
- Patterns that span multiple domains
"""
from dataclasses import dataclass
from typing import List, Dict, Optional, Any, Tuple
from datetime import datetime
import json


@dataclass
class CrossConnection:
    """A connection found between items from different products."""
    id: str
    item_a_id: str
    item_a_source: str  # Product name: "hypothesis_ledger", "scientist", etc.
    item_a_content: str
    item_b_id: str
    item_b_source: str
    item_b_content: str
    connection_type: str  # "semantic", "entity", "temporal", "causal"
    strength: float  # 0-1
    reasoning: str
    discovered_at: str

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "item_a": {
                "id": self.item_a_id,
                "source": self.item_a_source,
                "content": (self.item_a_content or "")[:200]
            },
            "item_b": {
                "id": self.item_b_id,
                "source": self.item_b_source,
                "content": (self.item_b_content or "")[:200]
            },
            "connection_type": self.connection_type,
            "strength": self.strength,
            "reasoning": self.reasoning,
            "discovered_at": self.discovered_at
        }


@dataclass
class InsightReport:
    """Report of cross-product insights."""
    connections: List[CrossConnection]
    total_items_scanned: int
    products_included: List[str]
    generated_at: str
    summary: str

    def to_dict(self) -> dict:
        return {
            "connections": [c.to_dict() for c in self.connections],
            "total_items_scanned": self.total_items_scanned,
            "products_included": self.products_included,
            "generated_at": self.generated_at,
            "summary": self.summary
        }


class CrossProductConnector:
    """Find connections across OMEGA products."""

    def __init__(self, vector_store=None, knowledge_graph=None):
        """
        Initialize with substrate components.

        Args:
            vector_store: VectorStore instance for semantic search
            knowledge_graph: KnowledgeGraph instance for entity relationships
        """
        self.vector_store = vector_store
        self.knowledge_graph = knowledge_graph
        self._product_loaders: Dict[str, Any] = {}
        self._register_default_loaders()

    def _register_default_loaders(self) -> None:
        """Register default product data loaders."""
        from pathlib import Path
        _products = Path(__file__).resolve().parent.parent.parent

        def load_hypothesis_ledger() -> List[Dict]:
            try:
                import sys
                sys.path.insert(0, str(_products / "breakthrough_engine"))
                from hypothesis_ledger import HypothesisLedger

                ledger = HypothesisLedger()
                items = []
                for h in ledger.list():
                    items.append({
                        "id": h.id,
                        "source": "hypothesis_ledger",
                        "content": getattr(h, "claim", "") or "",
                        "metadata": {
                            "domain": getattr(h, "domain", ""),
                            "confidence": getattr(h, "confidence", 0),
                            "status": str(getattr(h, "status", "")),
                        },
                        "timestamp": getattr(h, "updated_at", "") or getattr(h, "created_at", ""),
                    })
                return items
            except Exception:
                return []

        def load_research_memory() -> List[Dict]:
            try:
                import sys
                sys.path.insert(0, str(_products / "soft_robotics_lab" / "research_system"))
                from research_memory import ResearchMemory

                memory = ResearchMemory("default")
                items = []
                if hasattr(memory, "get_all_notes"):
                    for note in memory.get_all_notes():
                        items.append({
                            "id": note.get("id", ""),
                            "source": "research_memory",
                            "content": note.get("content", ""),
                            "metadata": {"tags": note.get("tags", [])},
                            "timestamp": note.get("created_at", ""),
                        })
                return items
            except Exception:
                return []

        def load_parsed_papers() -> List[Dict]:
            try:
                papers_dir = _products / "omega_scientist" / "outputs" / "parsed"
                items = []
                if papers_dir.exists():
                    for f in papers_dir.glob("*.json"):
                        try:
                            data = json.loads(f.read_text(encoding="utf-8"))
                            for claim in data.get("claims", []):
                                items.append({
                                    "id": f"{f.stem}_{claim.get('id', '')}",
                                    "source": "omega_scientist",
                                    "content": claim.get("text", ""),
                                    "metadata": {
                                        "paper": data.get("paper", {}).get("title", ""),
                                        "claim_type": claim.get("type", ""),
                                    },
                                    "timestamp": "",
                                })
                        except Exception:
                            pass
                return items
            except Exception:
                return []

        def load_reality_gap() -> List[Dict]:
            try:
                gap_file = _products / "world_model_studio" / "data" / "reality_gap.json"
                items = []
                if gap_file.exists():
                    data = json.loads(gap_file.read_text(encoding="utf-8"))
                    for record in data.get("records", []):
                        gap_metrics = record.get("gap_metrics", {})
                        items.append({
                            "id": record.get("id", ""),
                            "source": "reality_gap",
                            "content": f"Gap for {record.get('gripper_id')} on {record.get('task_id')}: {gap_metrics.get('severity', 'unknown')}",
                            "metadata": gap_metrics,
                            "timestamp": record.get("created_at", ""),
                        })
                return items
            except Exception:
                return []

        self._product_loaders = {
            "hypothesis_ledger": load_hypothesis_ledger,
            "research_memory": load_research_memory,
            "omega_scientist": load_parsed_papers,
            "reality_gap": load_reality_gap,
        }

    def register_loader(self, product_name: str, loader_func: Any) -> None:
        """Register a custom product loader."""
        self._product_loaders[product_name] = loader_func

    def _load_all_items(self, products: Optional[List[str]] = None) -> List[Dict]:
        """Load items from all or specified products."""
        items = []
        products = products or list(self._product_loaders.keys())

        for product in products:
            if product in self._product_loaders:
                try:
                    product_items = self._product_loaders[product]()
                    items.extend(product_items)
                except Exception:
                    pass

        return items

    def _find_semantic_connections(
        self,
        items: List[Dict],
        min_strength: float
    ) -> List[CrossConnection]:
        """Find connections via semantic similarity using vector store (collection, query, n) API."""
        connections = []
        if not self.vector_store:
            return connections

        by_source: Dict[str, List[Dict]] = {}
        for item in items:
            source = item.get("source", "unknown")
            if source not in by_source:
                by_source[source] = []
            by_source[source].append(item)

        try:
            collections = self.vector_store.list_collections()
        except Exception:
            collections = []

        for coll in collections:
            for item_a in items[:100]:
                try:
                    results = self.vector_store.search(coll, item_a.get("content", "")[:2000], n=5)
                    for r in results:
                        other_source = (r.get("metadata") or {}).get("source", coll)
                        if other_source != item_a.get("source"):
                            score = float(r.get("score", 0))
                            if score >= min_strength:
                                conn = CrossConnection(
                                    id=f"SEM-{str(item_a.get('id', ''))[:8]}-{str(r.get('id', ''))[:8]}",
                                    item_a_id=str(item_a.get("id", "")),
                                    item_a_source=item_a.get("source", ""),
                                    item_a_content=(item_a.get("content") or "")[:500],
                                    item_b_id=str(r.get("id", "")),
                                    item_b_source=other_source,
                                    item_b_content=(r.get("text") or r.get("content") or "")[:500],
                                    connection_type="semantic",
                                    strength=score,
                                    reasoning=f"Semantic similarity: {score:.0%}",
                                    discovered_at=datetime.now().isoformat(),
                                )
                                connections.append(conn)
                except Exception:
                    pass

        return connections

    def _find_entity_connections(
        self,
        items: List[Dict],
        min_strength: float
    ) -> List[CrossConnection]:
        """Find connections via shared entities."""
        connections = []
        item_entities: Dict[str, Dict] = {}
        for item in items:
            entities = self._extract_entities(item.get("content", ""))
            item_entities[item.get("id", "")] = {"item": item, "entities": entities}

        items_list = list(item_entities.items())
        for i, (id_a, data_a) in enumerate(items_list):
            for id_b, data_b in items_list[i + 1:]:
                if data_a["item"].get("source") == data_b["item"].get("source"):
                    continue
                shared = data_a["entities"] & data_b["entities"]
                if shared:
                    denom = max(len(data_a["entities"]), len(data_b["entities"]), 1)
                    strength = len(shared) / denom
                    if strength >= min_strength:
                        conn = CrossConnection(
                            id=f"ENT-{str(id_a)[:8]}-{str(id_b)[:8]}",
                            item_a_id=id_a,
                            item_a_source=data_a["item"].get("source", ""),
                            item_a_content=(data_a["item"].get("content") or "")[:500],
                            item_b_id=id_b,
                            item_b_source=data_b["item"].get("source", ""),
                            item_b_content=(data_b["item"].get("content") or "")[:500],
                            connection_type="entity",
                            strength=strength,
                            reasoning=f"Shared entities: {', '.join(list(shared)[:5])}",
                            discovered_at=datetime.now().isoformat(),
                        )
                        connections.append(conn)
        return connections

    def _find_keyword_connections(
        self,
        items: List[Dict],
        min_strength: float
    ) -> List[CrossConnection]:
        """Find connections via keyword overlap (fallback method)."""
        connections = []
        item_keywords: Dict[str, Dict] = {}
        for item in items:
            keywords = self._extract_keywords(item.get("content", ""))
            item_keywords[item.get("id", "")] = {"item": item, "keywords": keywords}

        items_list = list(item_keywords.items())
        for i, (id_a, data_a) in enumerate(items_list):
            for id_b, data_b in items_list[i + 1:]:
                if data_a["item"].get("source") == data_b["item"].get("source"):
                    continue
                shared = data_a["keywords"] & data_b["keywords"]
                union = data_a["keywords"] | data_b["keywords"]
                if union:
                    strength = len(shared) / len(union)
                    if strength >= min_strength and len(shared) >= 2:
                        conn = CrossConnection(
                            id=f"KW-{str(id_a)[:8]}-{str(id_b)[:8]}",
                            item_a_id=id_a,
                            item_a_source=data_a["item"].get("source", ""),
                            item_a_content=(data_a["item"].get("content") or "")[:500],
                            item_b_id=id_b,
                            item_b_source=data_b["item"].get("source", ""),
                            item_b_content=(data_b["item"].get("content") or "")[:500],
                            connection_type="keyword",
                            strength=strength,
                            reasoning=f"Shared keywords: {', '.join(list(shared)[:5])}",
                            discovered_at=datetime.now().isoformat(),
                        )
                        connections.append(conn)
        return connections

    def _extract_entities(self, text: str) -> set:
        """Extract named entities from text (simple implementation)."""
        import re
        entities = set()
        caps = re.findall(r"\b[A-Z][a-z]+(?:\s+[A-Z][a-z]+)*\b", text or "")
        entities.update(c.lower() for c in caps)
        domain_terms = [
            "gripper", "actuator", "pneumatic", "tendon", "silicone",
            "mujoco", "simulation", "robot", "soft", "hypothesis",
            "experiment", "validation", "design", "failure", "success"
        ]
        text_lower = (text or "").lower()
        for term in domain_terms:
            if term in text_lower:
                entities.add(term)
        return entities

    def _extract_keywords(self, text: str) -> set:
        """Extract keywords from text."""
        import re
        words = re.findall(r"\b[a-z]{3,}\b", (text or "").lower())
        stopwords = {
            "the", "and", "for", "that", "this", "with", "from", "are",
            "was", "were", "been", "have", "has", "had", "but", "not",
            "can", "will", "would", "could", "should", "may", "might"
        }
        return set(w for w in words if w not in stopwords)

    def find_cross_connections(
        self,
        products: Optional[List[str]] = None,
        min_strength: float = 0.5,
        max_results: int = 20
    ) -> InsightReport:
        """
        Find connections across products.

        Args:
            products: List of products to include (None = all)
            min_strength: Minimum connection strength (0-1)
            max_results: Maximum connections to return

        Returns:
            InsightReport with discovered connections
        """
        items = self._load_all_items(products)
        connections: List[CrossConnection] = []

        if not items:
            return InsightReport(
                connections=[],
                total_items_scanned=0,
                products_included=products or [],
                generated_at=datetime.now().isoformat(),
                summary="No items found to analyze"
            )

        if self.vector_store:
            connections.extend(self._find_semantic_connections(items, min_strength))
        if self.knowledge_graph:
            connections.extend(self._find_entity_connections(items, min_strength))
        connections.extend(self._find_keyword_connections(items, min_strength))

        seen: set = set()
        unique_connections: List[CrossConnection] = []
        for conn in sorted(connections, key=lambda c: c.strength, reverse=True):
            key = tuple(sorted([conn.item_a_id, conn.item_b_id]))
            if key not in seen:
                seen.add(key)
                unique_connections.append(conn)

        unique_connections = unique_connections[:max_results]

        if unique_connections:
            product_pairs = set()
            for c in unique_connections:
                pair = tuple(sorted([c.item_a_source, c.item_b_source]))
                product_pairs.add(pair)
            summary = f"Found {len(unique_connections)} connections across {len(product_pairs)} product pairs. "
            top = unique_connections[0]
            summary += f"Strongest: {top.item_a_source} ↔ {top.item_b_source} ({top.strength:.0%})"
        else:
            summary = "No significant cross-product connections found."

        return InsightReport(
            connections=unique_connections,
            total_items_scanned=len(items),
            products_included=list(set(i.get("source", "") for i in items)),
            generated_at=datetime.now().isoformat(),
            summary=summary
        )

    def find_related_to(
        self,
        content: str,
        exclude_source: Optional[str] = None,
        top_k: int = 5
    ) -> List[CrossConnection]:
        """
        Find items related to given content across all products.

        Args:
            content: Content to find relations for
            exclude_source: Product source to exclude
            top_k: Number of results

        Returns:
            List of connections
        """
        items = self._load_all_items()
        if exclude_source:
            items = [i for i in items if i.get("source") != exclude_source]

        connections: List[CrossConnection] = []
        query_keywords = self._extract_keywords(content)

        for item in items:
            item_keywords = self._extract_keywords(item.get("content", ""))
            shared = query_keywords & item_keywords
            if shared and len(shared) >= 2:
                union = query_keywords | item_keywords
                strength = len(shared) / len(union)
                conn = CrossConnection(
                    id=f"REL-{str(item.get('id', ''))[:8]}",
                    item_a_id="query",
                    item_a_source="query",
                    item_a_content=(content or "")[:200],
                    item_b_id=str(item.get("id", "")),
                    item_b_source=item.get("source", ""),
                    item_b_content=(item.get("content") or "")[:200],
                    connection_type="keyword",
                    strength=strength,
                    reasoning=f"Shared: {', '.join(list(shared)[:5])}",
                    discovered_at=datetime.now().isoformat(),
                )
                connections.append(conn)

        connections.sort(key=lambda c: c.strength, reverse=True)
        return connections[:top_k]
