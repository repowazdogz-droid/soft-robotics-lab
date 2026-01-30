"""
Validation logging - SQLite dataset at data/validations.db.
init_db(), log_validation(), get_stats(), get_failures(), get_designs_by_score().
"""

import json
import hashlib
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

_DATA_DIR = Path(__file__).resolve().parent.parent / "data"
_DB_PATH = _DATA_DIR / "validations.db"

_SCHEMA = """
CREATE TABLE IF NOT EXISTS validations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT NOT NULL,
    design_hash TEXT NOT NULL,
    mjcf_size INTEGER,
    domain TEXT,
    passed BOOLEAN,
    score REAL,
    tests_json TEXT,
    metrics_json TEXT,
    errors_json TEXT,
    source TEXT
)
"""


def init_db() -> None:
    """Create data dir and validations table if not exist."""
    _DATA_DIR.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(_DB_PATH) as conn:
        conn.executescript(_SCHEMA)


def _hash_mjcf(mjcf_string: str) -> str:
    """SHA256 hex digest of normalized MJCF string."""
    normalized = (mjcf_string or "").strip()
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def log_validation(
    mjcf_string: Optional[str],
    result: Any,
    source: str = "api",
    domain: Optional[str] = None,
) -> str:
    """
    Hash MJCF, store validation result. Returns design_hash.
    result: ValidationResult (passed, score, tests, metrics, errors).
    """
    mjcf = (mjcf_string or "").strip()
    design_hash = _hash_mjcf(mjcf)
    timestamp = datetime.utcnow().isoformat() + "Z"
    mjcf_size = len(mjcf) if mjcf else None

    passed = getattr(result, "passed", False)
    score = float(getattr(result, "score", 0.0))
    tests = getattr(result, "tests", {})
    metrics = getattr(result, "metrics", {})
    errors = list(getattr(result, "errors", []))

    def _test_to_dict(k: str, v: Any) -> dict:
        if hasattr(v, "name") and hasattr(v, "passed"):
            return {"name": v.name, "passed": v.passed, "message": getattr(v, "message", ""), "details": getattr(v, "details", {})}
        return {"name": k, "passed": False, "message": str(v), "details": {}}
    tests_json = json.dumps({k: _test_to_dict(k, v) for k, v in tests.items()}, default=str)
    metrics_json = json.dumps(metrics, default=str)
    errors_json = json.dumps(errors, default=str)

    with sqlite3.connect(_DB_PATH) as conn:
        conn.execute(
            """
            INSERT INTO validations
            (timestamp, design_hash, mjcf_size, domain, passed, score, tests_json, metrics_json, errors_json, source)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (timestamp, design_hash, mjcf_size, domain or "", 1 if passed else 0, score, tests_json, metrics_json, errors_json, source or "api"),
        )
    return design_hash


def get_stats() -> Dict[str, Any]:
    """Return total count, pass rate, avg score."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.execute("SELECT COUNT(*) AS total FROM validations")
        total = cur.fetchone()[0]
        if total == 0:
            return {"total": 0, "pass_rate": 0.0, "avg_score": 0.0}
        cur = conn.execute("SELECT SUM(passed) AS passed, AVG(score) AS avg_score FROM validations")
        row = cur.fetchone()
        passed = row[0] or 0
        avg_score = float(row[1] or 0)
    return {
        "total": total,
        "pass_rate": passed / total if total else 0.0,
        "avg_score": round(avg_score, 4),
    }


def get_failures(limit: int = 50) -> List[Dict[str, Any]]:
    """Return recent failures with errors."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.execute(
            "SELECT id, timestamp, design_hash, mjcf_size, domain, score, errors_json, source FROM validations WHERE passed = 0 ORDER BY id DESC LIMIT ?",
            (limit,),
        )
        rows = cur.fetchall()
    out = []
    for r in rows:
        out.append({
            "id": r["id"],
            "timestamp": r["timestamp"],
            "design_hash": r["design_hash"],
            "mjcf_size": r["mjcf_size"],
            "domain": r["domain"],
            "score": r["score"],
            "errors": json.loads(r["errors_json"] or "[]"),
            "source": r["source"],
        })
    return out


def get_designs_by_score(min_score: float = 0.9, limit: int = 10) -> List[Dict[str, Any]]:
    """Return high-performing designs (top by score)."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.execute(
            "SELECT id, timestamp, design_hash, mjcf_size, domain, passed, score, source FROM validations WHERE score >= ? ORDER BY score DESC, id DESC LIMIT ?",
            (min_score, limit),
        )
        rows = cur.fetchall()
    out = []
    for r in rows:
        out.append({
            "id": r["id"],
            "timestamp": r["timestamp"],
            "design_hash": r["design_hash"],
            "mjcf_size": r["mjcf_size"],
            "domain": r["domain"],
            "passed": bool(r["passed"]),
            "score": r["score"],
            "source": r["source"],
        })
    return out


def get_leaderboard(limit: int = 10) -> List[Dict[str, Any]]:
    """Return top designs by score (alias for get_designs_by_score with min_score=0)."""
    return get_designs_by_score(min_score=0.0, limit=limit)
