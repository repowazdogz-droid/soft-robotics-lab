#!/usr/bin/env python3
"""
Semantic Search Engine
======================

Embedding-based search for research memory.

Uses sentence-transformers for embeddings and FAISS for fast similarity search.
"""

import json
import pickle
import re
import numpy as np
from pathlib import Path
from typing import List, Dict, Optional
from dataclasses import dataclass


@dataclass
class SearchResult:
    """A semantic search result."""
    chunk_id: str
    doc_path: str
    section: str
    text: str
    score: float  # Similarity score 0-1
    highlight: str  # Text with query terms highlighted


class SemanticSearchEngine:
    """Semantic search using embeddings."""

    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        self.model_name = model_name
        self.model = None
        self.index = None
        self.chunk_ids: List[str] = []
        self.chunk_texts: List[str] = []
        self.chunk_metadata: List[Dict] = []
        self.embeddings = None

        self._load_model()

    def _load_model(self):
        """Load the sentence transformer model."""
        try:
            from sentence_transformers import SentenceTransformer
            self.model = SentenceTransformer(self.model_name)
            print(f"✓ Loaded embedding model: {self.model_name}")
        except ImportError:
            print("⚠ sentence-transformers not installed. Using keyword search fallback.")
            self.model = None

    def _ensure_faiss(self):
        """Ensure FAISS is available."""
        try:
            import faiss
            return faiss
        except ImportError:
            print("⚠ FAISS not installed. Using numpy fallback.")
            return None

    def build_index(self, chunks: List[Dict]):
        """
        Build search index from chunks.

        Args:
            chunks: List of dicts with 'id', 'text', 'doc_path', 'section'
        """
        if not chunks:
            return

        self.chunk_ids = [c["id"] for c in chunks]
        self.chunk_texts = [c["text"] for c in chunks]
        self.chunk_metadata = [
            {"doc_path": c.get("doc_path", ""), "section": c.get("section", "")}
            for c in chunks
        ]

        if self.model is None:
            return

        # Generate embeddings
        print(f"Generating embeddings for {len(chunks)} chunks...")
        embeddings = self.model.encode(self.chunk_texts, show_progress_bar=True)
        embeddings = embeddings.astype("float32")

        # Normalize for cosine similarity
        norms = np.linalg.norm(embeddings, axis=1, keepdims=True)
        embeddings = embeddings / norms

        # Build FAISS index
        faiss_mod = self._ensure_faiss()
        if faiss_mod:
            d = embeddings.shape[1]
            self.index = faiss_mod.IndexFlatIP(d)
            self.index.add(embeddings)
            print(f"✓ Built FAISS index with {self.index.ntotal} vectors")
        else:
            self.embeddings = embeddings
            print(f"✓ Built numpy index with {len(embeddings)} vectors")

    def search(self, query: str, top_k: int = 5) -> List[SearchResult]:
        """
        Search for chunks similar to query.

        Args:
            query: Search query
            top_k: Number of results to return

        Returns:
            List of SearchResult objects
        """
        if self.model is None:
            return self._keyword_search(query, top_k)

        # Encode query
        query_embedding = self.model.encode([query]).astype("float32")
        query_embedding = query_embedding / np.linalg.norm(query_embedding)

        # Search
        faiss_mod = self._ensure_faiss()
        if faiss_mod and self.index is not None:
            scores, indices = self.index.search(
                query_embedding, min(top_k, len(self.chunk_ids))
            )
            scores = scores[0]
            indices = indices[0]
        elif hasattr(self, "embeddings") and self.embeddings is not None:
            scores = np.dot(self.embeddings, query_embedding.T).flatten()
            indices = np.argsort(scores)[::-1][:top_k]
            scores = scores[indices]
        else:
            return self._keyword_search(query, top_k)

        # Build results
        results = []
        for score, idx in zip(scores, indices):
            if idx < 0 or idx >= len(self.chunk_ids):
                continue

            chunk_id = self.chunk_ids[idx]
            text = self.chunk_texts[idx]
            metadata = self.chunk_metadata[idx]

            highlight = self._highlight_text(text, query)

            results.append(
                SearchResult(
                    chunk_id=chunk_id,
                    doc_path=metadata["doc_path"],
                    section=metadata["section"],
                    text=text,
                    score=float(score),
                    highlight=highlight,
                )
            )

        return results

    def _keyword_search(self, query: str, top_k: int) -> List[SearchResult]:
        """Fallback keyword search."""
        query_terms = query.lower().split()

        scored_chunks = []
        for i, text in enumerate(self.chunk_texts):
            text_lower = text.lower()
            score = sum(1 for term in query_terms if term in text_lower)
            if score > 0:
                scored_chunks.append((score, i))

        scored_chunks.sort(reverse=True, key=lambda x: x[0])

        results = []
        for score, idx in scored_chunks[:top_k]:
            chunk_id = self.chunk_ids[idx]
            text = self.chunk_texts[idx]
            metadata = self.chunk_metadata[idx]

            results.append(
                SearchResult(
                    chunk_id=chunk_id,
                    doc_path=metadata["doc_path"],
                    section=metadata["section"],
                    text=text,
                    score=score / max(len(query_terms), 1),
                    highlight=self._highlight_text(text, query),
                )
            )

        return results

    def _highlight_text(self, text: str, query: str, context_chars: int = 150) -> str:
        """Create highlighted snippet around query terms."""
        query_terms = query.lower().split()
        text_lower = text.lower()

        best_pos = 0
        best_score = 0

        for term in query_terms:
            pos = text_lower.find(term)
            if pos >= 0:
                window_start = max(0, pos - context_chars)
                window_end = min(len(text), pos + context_chars)
                window = text_lower[window_start:window_end]
                score = sum(1 for t in query_terms if t in window)
                if score > best_score:
                    best_score = score
                    best_pos = pos

        start = max(0, best_pos - context_chars)
        end = min(len(text), best_pos + context_chars)

        snippet = text[start:end]

        if start > 0:
            snippet = "..." + snippet
        if end < len(text):
            snippet = snippet + "..."

        for term in query_terms:
            pattern = re.compile(re.escape(term), re.IGNORECASE)
            snippet = pattern.sub(f"**{term.upper()}**", snippet)

        return snippet

    def save(self, path: str):
        """Save index to disk."""
        path = Path(path)
        path.mkdir(parents=True, exist_ok=True)

        with open(path / "metadata.pkl", "wb") as f:
            pickle.dump(
                {
                    "chunk_ids": self.chunk_ids,
                    "chunk_texts": self.chunk_texts,
                    "chunk_metadata": self.chunk_metadata,
                },
                f,
            )

        faiss_mod = self._ensure_faiss()
        if faiss_mod and self.index is not None:
            faiss_mod.write_index(self.index, str(path / "index.faiss"))
        elif hasattr(self, "embeddings") and self.embeddings is not None:
            np.save(path / "embeddings.npy", self.embeddings)

        print(f"✓ Saved index to {path}")

    def load(self, path: str) -> bool:
        """Load index from disk."""
        path = Path(path)

        if not (path / "metadata.pkl").exists():
            return False

        with open(path / "metadata.pkl", "rb") as f:
            data = pickle.load(f)
            self.chunk_ids = data["chunk_ids"]
            self.chunk_texts = data["chunk_texts"]
            self.chunk_metadata = data["chunk_metadata"]

        faiss_mod = self._ensure_faiss()
        if faiss_mod and (path / "index.faiss").exists():
            self.index = faiss_mod.read_index(str(path / "index.faiss"))
            self.embeddings = None
        elif (path / "embeddings.npy").exists():
            self.embeddings = np.load(path / "embeddings.npy")
            self.index = None
        else:
            self.embeddings = None
            self.index = None

        print(f"✓ Loaded index from {path} ({len(self.chunk_ids)} chunks)")
        return True


if __name__ == "__main__":
    engine = SemanticSearchEngine()

    sample_chunks = [
        {
            "id": "1",
            "text": "Variable stiffness grippers can adapt to different object shapes.",
            "doc_path": "paper1.pdf",
            "section": "Abstract",
        },
        {
            "id": "2",
            "text": "Tendon-driven actuators provide precise control for soft robots.",
            "doc_path": "paper2.pdf",
            "section": "Methods",
        },
        {
            "id": "3",
            "text": "Pneumatic actuators are commonly used in soft grippers.",
            "doc_path": "paper3.pdf",
            "section": "Introduction",
        },
        {
            "id": "4",
            "text": "The compliance of soft materials allows for safe human-robot interaction.",
            "doc_path": "paper1.pdf",
            "section": "Discussion",
        },
        {
            "id": "5",
            "text": "Silicone elastomers like Ecoflex are popular choices for soft gripper fabrication.",
            "doc_path": "paper4.pdf",
            "section": "Materials",
        },
    ]

    engine.build_index(sample_chunks)

    results = engine.search("variable stiffness gripper")

    print("\nSearch results for 'variable stiffness gripper':")
    for r in results:
        print(f"\n  Score: {r.score:.3f}")
        print(f"  Doc: {r.doc_path} - {r.section}")
        print(f"  Highlight: {r.highlight}")
