"""
OMEGA Tutor — Progress tracking (sessions, topics, streak).
SQLite at data/progress.db.
"""

import sqlite3
from pathlib import Path
from datetime import datetime, date, timedelta
from typing import Optional, List, Dict, Any

_ROOT = Path(__file__).resolve().parent.parent
_DATA = _ROOT / "data"
_DB_PATH = _DATA / "progress.db"


def _conn():
    _DATA.mkdir(parents=True, exist_ok=True)
    return sqlite3.connect(str(_DB_PATH), detect_types=sqlite3.PARSE_DECIMAL_TYPES)


def _init_db():
    with _conn() as c:
        c.executescript("""
            CREATE TABLE IF NOT EXISTS sessions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                started_at TEXT NOT NULL,
                ended_at TEXT,
                topics_count INTEGER DEFAULT 0,
                questions_count INTEGER DEFAULT 0
            );
            CREATE TABLE IF NOT EXISTS topics (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                first_learned TEXT NOT NULL,
                last_reviewed TEXT,
                times_reviewed INTEGER DEFAULT 0,
                confidence REAL DEFAULT 1.0
            );
            CREATE TABLE IF NOT EXISTS questions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                topic TEXT NOT NULL,
                correct INTEGER NOT NULL,
                at TEXT NOT NULL
            );
            CREATE TABLE IF NOT EXISTS streak (
                date TEXT PRIMARY KEY,
                active INTEGER NOT NULL DEFAULT 1
            );
        """)


class ProgressTracker:
    """Track sessions, topics, questions, and daily streak."""

    def __init__(self, db_path: Optional[Path] = None):
        self.db_path = Path(db_path) if db_path else _DB_PATH
        _init_db()

    def start_session(self) -> int:
        """Start a new session; return session_id."""
        with _conn() as c:
            c.execute("INSERT INTO sessions (started_at) VALUES (?)", (datetime.now().isoformat(),))
            return c.lastrowid or 0

    def end_session(self, session_id: int) -> None:
        """Mark session ended and update counts if needed."""
        with _conn() as c:
            c.execute("UPDATE sessions SET ended_at = ? WHERE id = ?", (datetime.now().isoformat(), session_id))

    def record_topic(self, topic: str) -> None:
        """Record that user learned a topic (first time or review)."""
        topic = (topic or "").strip() or "general"
        now = datetime.now().isoformat()
        with _conn() as c:
            row = c.execute("SELECT id, first_learned, times_reviewed FROM topics WHERE name = ?", (topic,)).fetchone()
            if row:
                c.execute(
                    "UPDATE topics SET last_reviewed = ?, times_reviewed = times_reviewed + 1 WHERE name = ?",
                    (now, topic),
                )
            else:
                c.execute(
                    "INSERT INTO topics (name, first_learned, last_reviewed, times_reviewed, confidence) VALUES (?, ?, ?, 1, 1.0)",
                    (topic, now, now),
                )
            c.execute("INSERT INTO streak (date, active) VALUES (?, 1) ON CONFLICT(date) DO NOTHING", (date.today().isoformat(),))

    def record_question(self, topic: str, correct: bool) -> None:
        """Record a question (e.g. quiz) for a topic."""
        topic = (topic or "").strip() or "general"
        with _conn() as c:
            c.execute("INSERT INTO questions (topic, correct, at) VALUES (?, ?, ?)", (topic, 1 if correct else 0, datetime.now().isoformat()))

    def get_streak(self) -> int:
        """Consecutive days with activity (including today)."""
        with _conn() as c:
            rows = c.execute("SELECT date FROM streak ORDER BY date DESC").fetchall()
        if not rows:
            return 0
        dates = [r[0] for r in rows]
        today = date.today().isoformat()
        if today not in dates:
            return 0
        count = 0
        d = date.today()
        while d.isoformat() in dates:
            count += 1
            d -= timedelta(days=1)
        return count

    def get_weekly_stats(self) -> Dict[str, Any]:
        """Stats for current week: topics, questions, correct_rate, time_spent (minutes)."""
        week_start = (date.today() - timedelta(days=date.today().weekday())).isoformat()
        with _conn() as c:
            topics_count = c.execute(
                "SELECT COUNT(DISTINCT name) FROM topics WHERE date(first_learned) >= ? OR date(COALESCE(last_reviewed, first_learned)) >= ?",
                (week_start, week_start),
            ).fetchone()[0] or 0
            q = c.execute(
                "SELECT COUNT(*), SUM(correct) FROM questions WHERE date(at) >= ?",
                (week_start,),
            ).fetchone()
            questions_count = q[0] or 0
            correct_count = q[1] or 0
            correct_rate = (correct_count / questions_count * 100) if questions_count else 0
            sessions = c.execute(
                "SELECT started_at, ended_at FROM sessions WHERE started_at >= ?",
                (week_start + "T00:00:00",),
            ).fetchall()
        time_spent = 0
        for start, end in sessions:
            try:
                if end:
                    t0 = datetime.fromisoformat(start)
                    t1 = datetime.fromisoformat(end)
                    time_spent += (t1 - t0).total_seconds() / 60
                else:
                    time_spent += 5
            except Exception:
                time_spent += 5
        return {
            "topics": topics_count,
            "questions": questions_count,
            "correct_rate": round(correct_rate, 1),
            "time_spent": round(time_spent, 0),
        }

    def get_strongest_topics(self, n: int = 3) -> List[str]:
        """Topics with highest confidence and recent review."""
        with _conn() as c:
            rows = c.execute(
                "SELECT name FROM topics ORDER BY confidence DESC, times_reviewed DESC LIMIT ?",
                (n,),
            ).fetchall()
        return [r[0] for r in rows] if rows else []

    def get_weakest_topics(self, n: int = 3) -> List[str]:
        """Topics with low confidence or not reviewed recently."""
        with _conn() as c:
            rows = c.execute(
                """SELECT name FROM topics
                   ORDER BY confidence ASC, COALESCE(last_reviewed, first_learned) ASC
                   LIMIT ?""",
                (n,),
            ).fetchall()
        return [r[0] for r in rows] if rows else []

    def get_due_for_review(self) -> List[Dict[str, Any]]:
        """Topics that are due for review (decayed confidence or old)."""
        with _conn() as c:
            rows = c.execute(
                "SELECT name, confidence, last_reviewed, first_learned FROM topics"
            ).fetchall()
        from core.knowledge_decay import get_review_priority
        topics = [{"name": r[0], "confidence": float(r[1]), "last_reviewed": r[2], "first_learned": r[3]} for r in rows]
        return get_review_priority(topics)[:10]

    def get_recent_topics(self, limit: int = 20) -> List[Dict[str, Any]]:
        """Recent topics with last_reviewed or first_learned."""
        with _conn() as c:
            rows = c.execute(
                """SELECT name, first_learned, last_reviewed, confidence
                   FROM topics ORDER BY COALESCE(last_reviewed, first_learned) DESC LIMIT ?""",
                (limit,),
            ).fetchall()
        return [{"topic": r[0], "first_learned": r[1], "last_reviewed": r[2], "confidence": float(r[3])} for r in rows]

    def update_topic_confidence(self, topic: str, confidence: float) -> None:
        """Set confidence for a topic (0–1)."""
        topic = (topic or "").strip() or "general"
        confidence = max(0.0, min(1.0, confidence))
        with _conn() as c:
            c.execute("UPDATE topics SET confidence = ? WHERE name = ?", (confidence, topic))


progress_tracker = ProgressTracker()
