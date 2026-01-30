"""
OMEGA Tutor — Semantic memory using shared substrate (vector store + knowledge graph).
Stores topics learned, level, timestamp, confidence; supports related learning and history.
"""

import json
import re
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Any

_ROOT = Path(__file__).resolve().parent.parent
_DATA = _ROOT / "data"
_HISTORY_PATH = _DATA / "learning_history.json"
_CONFIDENCE_PATH = _DATA / "topics_confidence.json"

import sys
_PRODUCTS = _ROOT.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
try:
    from shared.substrate import vector_store, knowledge_graph
    from shared.substrate.knowledge_graph import NodeType
except ImportError:
    vector_store = knowledge_graph = None
    NodeType = None


def _slug(topic: str) -> str:
    """Normalize topic to a safe node id."""
    s = re.sub(r"[^\w\s-]", "", topic.lower())
    return re.sub(r"[-\s]+", "_", s).strip("_") or "topic"


def _load_history() -> List[Dict[str, Any]]:
    if not _HISTORY_PATH.exists():
        return []
    try:
        data = json.loads(_HISTORY_PATH.read_text(encoding="utf-8"))
        return data if isinstance(data, list) else []
    except Exception:
        return []


def _save_history(history: List[Dict[str, Any]]) -> None:
    _DATA.mkdir(parents=True, exist_ok=True)
    _HISTORY_PATH.write_text(json.dumps(history[-500:], indent=2), encoding="utf-8")


def _load_confidence() -> Dict[str, float]:
    if not _CONFIDENCE_PATH.exists():
        return {}
    try:
        data = json.loads(_CONFIDENCE_PATH.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _save_confidence(conf: Dict[str, float]) -> None:
    _DATA.mkdir(parents=True, exist_ok=True)
    _CONFIDENCE_PATH.write_text(json.dumps(conf, indent=2), encoding="utf-8")


def record_learning(topic: str, level: str, content: str) -> None:
    """Store topic in vector store and knowledge graph. Append to learning history."""
    topic = (topic or "").strip() or "general"
    level = (level or "adult").strip()
    ts = datetime.now().isoformat()
    confidence = 1.0
    meta = {"topic": topic, "level": level, "timestamp": ts, "confidence": confidence}
    if vector_store:
        try:
            doc_id = vector_store.add("concepts", content[:50000], meta)
        except Exception:
            doc_id = None
    else:
        doc_id = None
    if knowledge_graph and NodeType is not None:
        try:
            nid = _slug(topic) + "_" + ts[:10].replace("-", "")
            knowledge_graph.add_node(NodeType.Concept, nid, {"topic": topic, "level": level, "timestamp": ts, "confidence": confidence})
        except Exception:
            pass
    history = _load_history()
    history.append({"topic": topic, "timestamp": ts, "level": level, "confidence": confidence, "id": doc_id})
    _save_history(history)
    conf = _load_confidence()
    conf[topic] = confidence
    _save_confidence(conf)


def get_related_learning(query: str, n: int = 3) -> List[Dict[str, Any]]:
    """Search vector store for similar past learning. Returns list of {topic, text, metadata, score}."""
    if not vector_store:
        return []
    try:
        results = vector_store.search("concepts", query, n=n)
        return [{"topic": r.get("metadata", {}).get("topic", ""), "text": r.get("text", ""), "metadata": r.get("metadata", {}), "score": r.get("score", 0)} for r in results]
    except Exception:
        return []


def has_learned(topic: str) -> bool:
    """True if topic appears in confidence map or vector store."""
    topic = (topic or "").strip()
    if not topic:
        return False
    conf = _load_confidence()
    if topic in conf:
        return True
    if vector_store:
        try:
            results = vector_store.search("concepts", topic, n=1)
            if results and results[0].get("metadata", {}).get("topic", "").lower() == topic.lower():
                return True
        except Exception:
            pass
    return False


def get_learning_history(limit: int = 20) -> List[Dict[str, Any]]:
    """Recent topics with timestamps (newest first)."""
    history = _load_history()
    return list(reversed(history[-limit:]))


def update_confidence(topic: str, delta: float) -> None:
    """Adjust confidence for topic (e.g. after quiz). Clamped to [0, 1]."""
    topic = (topic or "").strip()
    if not topic:
        return
    conf = _load_confidence()
    current = conf.get(topic, 1.0)
    conf[topic] = max(0.0, min(1.0, current + delta))
    _save_confidence(conf)
