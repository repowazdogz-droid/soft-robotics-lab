"""
OMEGA Tutor — Learning paths with prerequisites and progress.
Curricula stored in data/curricula/*.json.
"""

import json
import os
import re
import sys
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Any

_ROOT = Path(__file__).resolve().parent.parent
_CURRICULA_DIR = _ROOT / "data" / "curricula"

# Optional memory for auto-detect completion
try:
    from core import memory
except ImportError:
    memory = None

from core.curriculum_progress import is_topic_complete, mark_topic_complete, get_curriculum_stats


def _ensure_curricula_dir():
    _CURRICULA_DIR.mkdir(parents=True, exist_ok=True)


class CurriculumEngine:
    """Manages learning paths: load, progress, prerequisites, next topics, custom generation."""

    def __init__(self, curricula_dir: Optional[Path] = None):
        self.curricula_dir = Path(curricula_dir) if curricula_dir else _CURRICULA_DIR

    def load_curriculum(self, curriculum_id: str) -> Optional[Dict[str, Any]]:
        """Load curriculum by id (filename without .json)."""
        path = self.curricula_dir / f"{curriculum_id}.json"
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else None
        except Exception:
            return None

    def list_curricula(self) -> List[Dict[str, Any]]:
        """List available curricula: id, name, description, topic_count."""
        _ensure_curricula_dir()
        result = []
        for path in self.curricula_dir.glob("*.json"):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(data, dict):
                    topics = data.get("topics", [])
                    result.append({
                        "id": data.get("id", path.stem),
                        "name": data.get("name", path.stem),
                        "description": data.get("description", ""),
                        "topic_count": len(topics),
                    })
            except Exception:
                pass
        return result

    def get_progress(self, curriculum_id: str) -> Dict[str, Any]:
        """Returns {completed: [topic_ids], available: [topic], locked: [topic], percent_complete}."""
        curr = self.load_curriculum(curriculum_id)
        if not curr:
            return {"completed": [], "available": [], "locked": [], "percent_complete": 0.0}
        topics = curr.get("topics", [])
        stats = get_curriculum_stats(curriculum_id)
        completed_ids = set(stats.get("completed_topic_ids", []))
        completed = []
        available = []
        locked = []
        for t in topics:
            tid = t.get("id", "")
            name = t.get("name", tid)
            prereqs = t.get("prerequisites", [])
            if tid in completed_ids:
                completed.append({"id": tid, "name": name, **t})
            elif all(pid in completed_ids for pid in prereqs):
                available.append({"id": tid, "name": name, **t})
            else:
                locked.append({"id": tid, "name": name, "prerequisites": prereqs, **t})
        total = len(topics)
        pct = (len(completed) / total * 100.0) if total else 0.0
        return {
            "completed": completed,
            "available": available,
            "locked": locked,
            "percent_complete": round(pct, 1),
        }

    def check_prerequisites(self, curriculum_id: str, topic_id: str) -> bool:
        """True if all prerequisites for topic are completed."""
        curr = self.load_curriculum(curriculum_id)
        if not curr:
            return False
        topics = {t.get("id"): t for t in curr.get("topics", [])}
        topic = topics.get(topic_id)
        if not topic:
            return False
        prereqs = topic.get("prerequisites", [])
        stats = get_curriculum_stats(curriculum_id)
        completed = set(stats.get("completed_topic_ids", []))
        return all(p in completed for p in prereqs)

    def get_next_topics(self, curriculum_id: str, n: int = 3) -> List[Dict[str, Any]]:
        """Available topics user hasn't completed (up to n)."""
        progress = self.get_progress(curriculum_id)
        return progress.get("available", [])[:n]

    def mark_complete(self, curriculum_id: str, topic_id: str) -> None:
        """Mark topic complete. Uses memory.has_learned for optional auto-detect; we always persist."""
        mark_topic_complete(curriculum_id, topic_id, confidence=1.0)

    def generate_custom_curriculum(self, topics: List[str], api_key: Optional[str] = None) -> Dict[str, Any]:
        """Create curriculum from user's topic list using LLM; returns curriculum dict with prerequisites."""
        topics = [t.strip() for t in topics if t.strip()]
        if not topics:
            return {"id": "custom", "name": "Custom Path", "description": "Your topics", "topics": []}
        prompt = f"""Given these learning topics: {json.dumps(topics)}.
Create a learning path with prerequisites. Order topics so dependencies come first.
Return JSON only:
{{"id": "custom-1", "name": "Custom Path", "description": "One line", "topics": [{{"id": "slug", "name": "Display Name", "prerequisites": ["id1", "id2"]}}, ...]}}
Use short id slugs (e.g. materials-basics). Prerequisites must refer to ids in the same list.
"""
        key = api_key or os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
        raw = ""
        try:
            from openai import OpenAI
            client = OpenAI(base_url="http://localhost:1234/v1", api_key="lm-studio")
            models = client.models.list()
            model_id = models.data[0].id if models.data else "local-model"
            r = client.chat.completions.create(model=model_id, messages=[{"role": "user", "content": prompt}], temperature=0.3)
            raw = (r.choices[0].message.content or "").strip()
        except Exception:
            pass
        if not raw and key:
            try:
                import google.generativeai as genai
                genai.configure(api_key=key)
                model = genai.GenerativeModel("gemini-2.0-flash")
                r = model.generate_content(prompt)
                raw = (r.text or "").strip()
            except Exception:
                pass
        if not raw:
            return {
                "id": "custom-1",
                "name": "Custom Path",
                "description": "Your topics in order",
                "topics": [{"id": re.sub(r"[^a-z0-9]+", "-", t.lower()).strip("-") or f"t{i}", "name": t, "prerequisites": [] if i == 0 else [re.sub(r"[^a-z0-9]+", "-", topics[i - 1].lower()).strip("-") or f"t{i-1}"]} for i, t in enumerate(topics)],
            }
        try:
            m = re.search(r"\{[\s\S]*\}", raw)
            if m:
                data = json.loads(m.group(0))
                data["id"] = data.get("id", "custom-1")
                data["name"] = data.get("name", "Custom Path")
                data["topics"] = data.get("topics", [])
                return data
        except Exception:
            pass
        return {
            "id": "custom-1",
            "name": "Custom Path",
            "description": "Your topics",
            "topics": [{"id": f"t{i}", "name": t, "prerequisites": []} for i, t in enumerate(topics)],
        }


curriculum_engine = CurriculumEngine()
