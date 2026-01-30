"""
OMEGA Substrate — Local vector storage with ChromaDB and sentence-transformers.
Data: products/shared/substrate/data/chroma/
Collections: materials, literature, protocols, experiments, designs, conversations, concepts.
"""

import uuid
from pathlib import Path
from typing import Optional, Dict, Any, List

_SUBSTRATE_DIR = Path(__file__).resolve().parent
_DEFAULT_PERSIST_DIR = _SUBSTRATE_DIR / "data" / "chroma"

# ChromaDB and embedding function (lazy to avoid slow import at module load)
_client = None
_embedding_fn = None


def _get_client(persist_dir: Path):
    global _client
    if _client is None:
        import chromadb
        persist_dir.mkdir(parents=True, exist_ok=True)
        _client = chromadb.PersistentClient(path=str(persist_dir))
    return _client


def _get_embedding_fn():
    global _embedding_fn
    if _embedding_fn is None:
        try:
            from chromadb.utils import embedding_functions
            _embedding_fn = embedding_functions.SentenceTransformerEmbeddingFunction(
                model_name="all-MiniLM-L6-v2"
            )
        except Exception:
            # Fallback: Chroma default (uses all-MiniLM-L6-v2 if available)
            from chromadb.utils import embedding_functions
            _embedding_fn = embedding_functions.DefaultEmbeddingFunction()
    return _embedding_fn


class VectorStore:
    """Local vector store using ChromaDB and sentence-transformers."""

    def __init__(self, persist_dir: Optional[Path] = None):
        self.persist_dir = Path(persist_dir) if persist_dir else _DEFAULT_PERSIST_DIR

    def _collection(self, collection: str):
        client = _get_client(self.persist_dir)
        return client.get_or_create_collection(
            name=collection,
            embedding_function=_get_embedding_fn(),
            metadata={"hnsw:space": "cosine"},
        )

    def add(self, collection: str, text: str, metadata: Optional[Dict[str, Any]] = None) -> str:
        """Add a document; return assigned id."""
        meta = metadata or {}
        # Chroma supports str, int, float, bool; stringify others
        safe_meta = {}
        for k, v in meta.items():
            if v is None:
                continue
            if isinstance(v, (str, int, float, bool)):
                safe_meta[k] = v
            else:
                safe_meta[k] = str(v)
        doc_id = str(uuid.uuid4())
        coll = self._collection(collection)
        coll.add(ids=[doc_id], documents=[text], metadatas=[safe_meta])
        return doc_id

    def search(self, collection: str, query: str, n: int = 5) -> List[Dict[str, Any]]:
        """Search by query; return list of {id, text, metadata, score}."""
        coll = self._collection(collection)
        out = coll.query(query_texts=[query], n_results=n, include=["documents", "metadatas", "distances"])
        ids = out["ids"][0] if out.get("ids") else []
        docs = (out["documents"][0] if out.get("documents") else []) or []
        metadatas = (out["metadatas"][0] if out.get("metadatas") else []) or []
        distances = (out["distances"][0] if out.get("distances") else []) or []
        results = []
        for i, doc_id in enumerate(ids):
            # Cosine distance: 0 = identical, 2 = opposite. Score = 1 - (d/2) for [0,1] range.
            dist = distances[i] if i < len(distances) else 0.0
            score = max(0.0, 1.0 - (dist / 2.0))
            results.append({
                "id": doc_id,
                "text": docs[i] if i < len(docs) else "",
                "metadata": metadatas[i] if i < len(metadatas) else {},
                "score": round(score, 4),
            })
        return results

    def get(self, collection: str, id: str) -> Optional[Dict[str, Any]]:
        """Get one document by id."""
        coll = self._collection(collection)
        out = coll.get(ids=[id], include=["documents", "metadatas"])
        if not out["ids"]:
            return None
        return {
            "id": out["ids"][0],
            "text": out["documents"][0] if out["documents"] else "",
            "metadata": out["metadatas"][0] if out["metadatas"] else {},
        }

    def delete(self, collection: str, id: str) -> None:
        """Remove document by id."""
        coll = self._collection(collection)
        coll.delete(ids=[id])

    def list_collections(self) -> List[str]:
        """List collection names."""
        client = _get_client(self.persist_dir)
        return [c.name for c in client.list_collections()]

    def count(self, collection: str) -> int:
        """Number of documents in collection."""
        coll = self._collection(collection)
        return coll.count()


vector_store = VectorStore()
