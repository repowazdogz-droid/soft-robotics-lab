"""
OMEGA-MAX knowledge base (optional).
Saves learned topics for later reference or session continuity.
"""

import json
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Any

_DATA_ROOT = Path(__file__).resolve().parent.parent / "data"
_SESSIONS_DIR = _DATA_ROOT / "sessions"


class KnowledgeBase:
    """Optional: save learned topics and session history."""

    def __init__(self, data_root: Optional[Path] = None):
        self.sessions_dir = (data_root or _DATA_ROOT) / "sessions"
        self.sessions_dir.mkdir(parents=True, exist_ok=True)

    def save_session(self, topic: str, level: str, summary: str, messages_count: int = 0) -> str:
        """Save a learning session; returns session id."""
        sid = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        path = self.sessions_dir / f"{sid}.json"
        rec = {
            "id": sid,
            "topic": topic,
            "level": level,
            "summary": summary,
            "messages_count": messages_count,
            "saved_at": datetime.now().isoformat(),
        }
        path.write_text(json.dumps(rec, indent=2), encoding="utf-8")
        return sid

    def list_sessions(self) -> List[Dict[str, Any]]:
        """List saved sessions (most recent first)."""
        out = []
        for p in sorted(self.sessions_dir.glob("session_*.json"), reverse=True):
            try:
                rec = json.loads(p.read_text(encoding="utf-8"))
                out.append(rec)
            except Exception:
                pass
        return out
