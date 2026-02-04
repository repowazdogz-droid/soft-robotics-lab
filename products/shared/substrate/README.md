# 🧠 Substrate

**Memory & Knowledge Layer for OMEGA Stack**

Vector store, knowledge graph, lineage tracking, audit bundles.

---

## 🚀 Quick Start

**Conceptual API (target):**

```python
from shared.substrate import Substrate

substrate = Substrate()
substrate.store(
    content="Silicone elastomers cure faster at higher temperatures",
    metadata={"domain": "materials", "source": "experiment_042"}
)
results = substrate.query("silicone curing temperature", top_k=5)
```

**In this codebase** use the concrete stores (same process, different imports):

```python
# Ensure products/shared is on sys.path, then:
from shared.substrate import vector_store, knowledge_graph, lineage_graph

# Store text in a collection (vector store)
doc_id = vector_store.add(
    "materials",
    "Silicone elastomers cure faster at higher temperatures",
    metadata={"domain": "materials", "source": "experiment_042"}
)

# Semantic search
results = vector_store.search("materials", "silicone curing temperature", n=5)
for r in results:
    print(f"{r['score']:.2f}: {r['text']}")

# Record lineage (e.g. design derived from validation)
lineage_graph.record("design_v3", "design_v2", "validation", {"score": 0.9})
```

*(Path: from repo root use `products/shared` or install shared package so `shared.substrate` resolves.)*

---

## 💡 What Problem Does This Solve?

Research knowledge is fragmented. Connections between ideas are invisible. Lineage is lost.

| Problem | Solution |
|---------|----------|
| Knowledge scattered | Central vector store |
| Connections invisible | Knowledge graph |
| No audit trail | Full lineage tracking |
| Memory fades | Persistent storage |
| Can't find old insights | Semantic search |

---

## 📦 Features

### 1. Vector Store
Semantic search across all knowledge.

- **VectorStore** (ChromaDB + sentence-transformers): `add(collection, text, metadata)`, `search(collection, query, n)`, `get(collection, id)`, `list_collections()`, `count(collection)`.
- Collections: e.g. `materials`, `literature`, `protocols`, `experiments`, `designs`, `conversations`, `concepts`.
- Automatic embedding; similarity search; metadata filtering.

```python
from shared.substrate import vector_store

doc_id = vector_store.add(
    "soft_robotics",
    "Pneumatic actuators provide high force but slow response",
    metadata={"domain": "soft_robotics", "type": "observation"}
)
results = vector_store.search("soft_robotics", "what actuator type is best for speed?", n=5)
for r in results:
    print(f"{r['score']:.2f}: {r['text']}")
```

### 2. Knowledge Graph
Explicit relationships between concepts.

- **KnowledgeGraph** (NetworkX): `add_node(node_type, node_id, properties)`, `add_edge(from_id, to_id, edge_type, properties)`, `get_related(node_id, edge_type, direction)`, `find_path(from_id, to_id)`, `get_subgraph(node_id, depth)`.
- **NodeType**: Material, Design, Gripper, Mechanism, Actuator, Sensor, Gene, Protein, Pathway, Property, Concept, Hypothesis, Experiment, Validation, etc.
- **EdgeType**: `has_property`, `is_part_of`, `contradicts`, `derived_from`, `validates`, `affects`, `enables`, etc.

```python
from shared.substrate import knowledge_graph
from shared.substrate.knowledge_graph import NodeType, EdgeType

knowledge_graph.add_node(NodeType.Material, "pneumatic_actuator", {"force": "high", "speed": "low"})
knowledge_graph.add_node(NodeType.Property, "high_force")
knowledge_graph.add_edge("pneumatic_actuator", "high_force", EdgeType.has_property)
related = knowledge_graph.get_related("pneumatic_actuator")
path = knowledge_graph.find_path("pneumatic_actuator", "surgical_gripper")
```

### 3. Lineage Tracking
Provenance for every artifact.

- **LineageGraph**: `record(child_id, parent_id, method, parameters, timestamp)`, `get_parents(id)`, `get_lineage(id)` (all ancestors), `get_children(id)`, `get_descendants(id)`, `get_full_provenance(id)`.
- Data: `products/shared/substrate/data/lineage.json`.

```python
from shared.substrate import lineage_graph

lineage_graph.record(
    "design_v3",
    "design_v2",
    "validation",
    parameters={"score": 0.92},
    timestamp="2024-01-20T14:30:00Z"
)
ancestors = lineage_graph.get_lineage("design_v3")
```

### 4. Audit Bundles
Reproducible knowledge/artifact packages.

- **AuditBundle** lives in `shared/audit/bundle.py`: zip with metadata, artifact, contract, validation, logs. Use for design/validation/training runs.
- For “audit bundle” of substrate knowledge (vector + graph + lineage subset), combine vector_store.search + knowledge_graph.get_subgraph + lineage_graph.get_full_provenance and export to JSON/zip as needed.

### 5. Cross-Product Insights
Products (Scientist, Ledger, Foundry, Reality Bridge, Tutor) each have `substrate_integration` modules that store and query the shared substrate (vector_store, knowledge_graph, lineage_graph). Cross-product discovery is done by querying shared collections and graph from multiple domains.

### 6. Temporal Queries
- **Vector store**: metadata can include `timestamp`; filter by date in app logic.
- **Lineage**: `get_lineage(id)` and edge `timestamp` give derivation chronology.
- Belief timelines (e.g. “what did we believe at date X?”) can be implemented by storing dated claims in the vector store and filtering on metadata.

---

## 🏗️ Architecture

```
products/shared/
├── substrate/
│   ├── __init__.py              # VectorStore, KnowledgeGraph, LineageGraph, singletons
│   ├── vector_store.py          # ChromaDB + embeddings (add, search, get)
│   ├── knowledge_graph.py       # NetworkX graph (add_node, add_edge, get_related, find_path)
│   ├── lineage.py              # LineageGraph (record, get_lineage, get_parents)
│   ├── data/
│   │   ├── chroma/              # ChromaDB persistence
│   │   ├── knowledge_graph.json
│   │   └── lineage.json
│   └── README.md                # This file
└── audit/
    ├── bundle.py                # AuditBundle (zip: metadata, artifact, contract, validation, logs)
    └── bundles/                # Output directory
```

---

## 🔄 Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                      OMEGA Products                             │
│   Scientist │ Ledger │ Foundry │ Reality Bridge │ Tutor        │
└─────────────────────────┬──────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                       SUBSTRATE                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ VectorStore │  │ Knowledge   │  │ LineageGraph             │  │
│  │ (ChromaDB)  │  │ Graph (NX)  │  │ (provenance)            │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
│         data/chroma    data/knowledge_graph.json   data/lineage.json
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Storage Backends

| Backend | Use Case |
|---------|----------|
| **ChromaDB (default)** | Local vector store; used in this repo |
| **SQLite** | Lineage/graph JSON files are file-based; could be swapped to SQLite |
| **PostgreSQL + pgvector** | Production, larger scale (not in repo yet) |
| **Neo4j** | Advanced graph queries (not in repo yet) |

---

## 🔌 Integration

### From OMEGA Scientist
Store parsed claims in a collection (e.g. `literature` or `concepts`) and optionally add nodes/edges to the knowledge graph.

### From Hypothesis Ledger
Record hypothesis IDs and relations: `knowledge_graph.add_node(NodeType.Hypothesis, h.id)` and `knowledge_graph.add_edge(h.id, paper_id, EdgeType.derived_from)`.

### From Reality Bridge
After validation, store a summary in the vector store and record lineage: `lineage_graph.record(validation_id, design_id, "validate", {"score": result.score})`.

---

## 🧪 Example Workflow

```python
from shared.substrate import vector_store, knowledge_graph, lineage_graph
from shared.substrate.knowledge_graph import NodeType, EdgeType

# 1. Store knowledge
vector_store.add("materials", "Ecoflex 00-30 has Shore hardness 00-30", metadata={"domain": "materials", "source": "datasheet"})
vector_store.add("design", "Lower shore hardness = more compliant grippers", metadata={"domain": "design", "source": "experiment"})

# 2. Build knowledge graph
knowledge_graph.add_node(NodeType.Material, "ecoflex_00-30", {"shore": "00-30"})
knowledge_graph.add_node(NodeType.Property, "compliance")
knowledge_graph.add_edge("ecoflex_00-30", "compliance", EdgeType.has_property)

# 3. Semantic search
results = vector_store.search("materials", "what material for soft compliant gripper?", n=5)

# 4. Lineage
lineage_graph.record("design_v2", "design_v1", "iteration", {"feedback": "softer"})
ancestors = lineage_graph.get_lineage("design_v2")

# 5. Related nodes
related = knowledge_graph.get_related("ecoflex_00-30")
path = knowledge_graph.find_path("ecoflex_00-30", "surgical_gripper")
```

---

## 📁 Data Format

### Vector store (per document)
- **id**: UUID from Chroma.
- **text**: Stored content.
- **metadata**: Flat dict (str/int/float/bool) per Chroma rules.

### Lineage edge
```json
{
  "child_id": "design_v3",
  "parent_id": "design_v2",
  "method": "validation",
  "parameters": {},
  "timestamp": "2024-01-20T14:30:00Z"
}
```

### Graph node (NetworkX)
- **id**: node_id.
- **node_type**: NodeType value.
- **properties**: arbitrary dict saved on node.

### Graph edge
- **from**, **to**: node ids.
- **edge_type**: EdgeType value.
- **properties**: optional dict.

---

## 🎯 Use Cases

1. **Knowledge retrieval**: “What did we learn about X?” → vector_store.search(collection, query).
2. **Connection discovery**: “How does A relate to B?” → knowledge_graph.find_path(a, b), get_related(a).
3. **Audit trail**: “Where did this artifact come from?” → lineage_graph.get_lineage(id).
4. **Temporal analysis**: Use lineage timestamps and/or dated metadata in vector store.
5. **Cross-product insight**: Query shared collections and graph from Scientist, Ledger, Foundry, Reality Bridge, Tutor.

---

## 📋 Requirements

- **Python**: 3.10+
- **chromadb**: vector store (used by this repo).
- **sentence-transformers** or Chroma default embedding: for embeddings.
- **networkx**: knowledge graph.

Optional: `products/shared/requirements.txt` may list these. From repo root, ensure `products/shared` (or `shared`) is on `sys.path` when importing `shared.substrate`.

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"Remember everything. Connect everything. Trace everything."*
