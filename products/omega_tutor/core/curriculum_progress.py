"""
OMEGA Tutor — Curriculum-specific progress (SQLite).
Table: curriculum_progress (curriculum_id, topic_id, completed_at, confidence).
"""

import sqlite3
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Any

_ROOT = Path(__file__).resolve().parent.parent
_DATA = _ROOT / "data"
_DB_PATH = _DATA / "progress.db"


def _conn():
    _DATA.mkdir(parents=True, exist_ok=True)
    return sqlite3.connect(str(_DB_PATH), detect_types=sqlite3.PARSE_DECIMAL_TYPES)


def _init_table():
    with _conn() as c:
        c.execute("""
            CREATE TABLE IF NOT EXISTS curriculum_progress (
                curriculum_id TEXT NOT NULL,
                topic_id TEXT NOT NULL,
                completed_at TEXT NOT NULL,
                confidence REAL DEFAULT 1.0,
                PRIMARY KEY (curriculum_id, topic_id)
            )
        """)


def is_topic_complete(curriculum_id: str, topic_id: str) -> bool:
    """True if topic is marked complete for this curriculum."""
    _init_table()
    with _conn() as c:
        row = c.execute(
            "SELECT 1 FROM curriculum_progress WHERE curriculum_id = ? AND topic_id = ?",
            (curriculum_id, topic_id),
        ).fetchone()
    return row is not None


def mark_topic_complete(curriculum_id: str, topic_id: str, confidence: float = 1.0) -> None:
    """Mark topic as complete for curriculum."""
    _init_table()
    now = datetime.now().isoformat()
    with _conn() as c:
        c.execute(
            "INSERT OR REPLACE INTO curriculum_progress (curriculum_id, topic_id, completed_at, confidence) VALUES (?, ?, ?, ?)",
            (curriculum_id, topic_id, now, confidence),
        )


def get_curriculum_stats(curriculum_id: str) -> Dict[str, Any]:
    """Returns {completed_count, total_count, percent_complete, completed_topic_ids}."""
    _init_table()
    with _conn() as c:
        rows = c.execute(
            "SELECT topic_id FROM curriculum_progress WHERE curriculum_id = ?",
            (curriculum_id,),
        ).fetchall()
    completed_ids = [r[0] for r in rows]
    return {
        "completed_count": len(completed_ids),
        "completed_topic_ids": completed_ids,
        "total_count": None,
        "percent_complete": 0.0,
    }


def set_curriculum_total(curriculum_id: str, total: int) -> None:
    """Optional: store total topic count for a curriculum (we derive from JSON usually)."""
    pass
