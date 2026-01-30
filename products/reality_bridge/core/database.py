"""
Validation logging - SQLite dataset at data/validations.db.
init_db(), log_validation(), get_stats(), get_failures(), get_designs_by_score().
Uses shared OMEGA ID: validation_id(), error_id().
"""

import json
import hashlib
import sqlite3
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

_DATA_DIR = Path(__file__).resolve().parent.parent / "data"
_DB_PATH = _DATA_DIR / "validations.db"
_PRODUCTS = Path(__file__).resolve().parent.parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.id_generator import validation_id, error_id

_SCHEMA = """
CREATE TABLE IF NOT EXISTS validations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    validation_id TEXT,
    error_id TEXT,
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
    """Create data dir and validations table if not exist. Migrate: add validation_id, error_id, validation_time_ms, artifact_id if missing."""
    _DATA_DIR.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(_DB_PATH) as conn:
        conn.executescript(_SCHEMA)
        try:
            conn.execute("ALTER TABLE validations ADD COLUMN validation_id TEXT")
        except sqlite3.OperationalError:
            pass
        try:
            conn.execute("ALTER TABLE validations ADD COLUMN error_id TEXT")
        except sqlite3.OperationalError:
            pass
        try:
            conn.execute("ALTER TABLE validations ADD COLUMN validation_time_ms INTEGER")
        except sqlite3.OperationalError:
            pass
        try:
            conn.execute("ALTER TABLE validations ADD COLUMN artifact_id TEXT")
        except sqlite3.OperationalError:
            pass


def _hash_mjcf(mjcf_string: str) -> str:
    """SHA256 hex digest of normalized MJCF string."""
    normalized = (mjcf_string or "").strip()
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def log_validation(
    mjcf_string: Optional[str],
    result: Any,
    source: str = "api",
    domain: Optional[str] = None,
    validation_time_ms: Optional[int] = None,
    artifact_id: Optional[str] = None,
) -> tuple:
    """
    Hash MJCF, store validation result. Returns (design_hash, validation_id).
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
    vid = validation_id()
    eid = error_id("validation_failed") if not passed else None

    def _test_to_dict(k: str, v: Any) -> dict:
        if hasattr(v, "name") and hasattr(v, "passed"):
            return {"name": v.name, "passed": v.passed, "message": getattr(v, "message", ""), "details": getattr(v, "details", {})}
        return {"name": k, "passed": False, "message": str(v), "details": {}}
    tests_json = json.dumps({k: _test_to_dict(k, v) for k, v in tests.items()}, default=str)
    metrics_json = json.dumps(metrics, default=str)
    errors_json = json.dumps(errors, default=str)

    with sqlite3.connect(_DB_PATH) as conn:
        try:
            conn.execute(
                """
                INSERT INTO validations
                (validation_id, error_id, timestamp, design_hash, mjcf_size, domain, passed, score, tests_json, metrics_json, errors_json, source, validation_time_ms, artifact_id)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (vid, eid, timestamp, design_hash, mjcf_size, domain or "", 1 if passed else 0, score, tests_json, metrics_json, errors_json, source or "api", validation_time_ms, artifact_id or ""),
            )
        except sqlite3.OperationalError:
            conn.execute(
                """
                INSERT INTO validations
                (validation_id, error_id, timestamp, design_hash, mjcf_size, domain, passed, score, tests_json, metrics_json, errors_json, source)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (vid, eid, timestamp, design_hash, mjcf_size, domain or "", 1 if passed else 0, score, tests_json, metrics_json, errors_json, source or "api"),
            )
    return design_hash, vid


def get_stats() -> Dict[str, Any]:
    """Return total count, pass rate, avg score, today/this_week, avg_validation_ms, failure_distribution."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.execute("SELECT COUNT(*) AS total FROM validations")
        total = cur.fetchone()[0]
        if total == 0:
            return {
                "total": 0, "total_validations": 0, "pass_rate": 0.0, "avg_score": 0.0,
                "avg_validation_ms": None, "today": {"count": 0, "pass_rate": 0.0},
                "this_week": {"count": 0, "pass_rate": 0.0}, "failure_distribution": {},
            }
        cur = conn.execute("SELECT SUM(passed) AS passed, AVG(score) AS avg_score FROM validations")
        row = cur.fetchone()
        passed = row[0] or 0
        avg_score = float(row[1] or 0)
        avg_ms = None
        today_count = today_passed = week_count = week_passed = 0
        try:
            cur = conn.execute("SELECT AVG(validation_time_ms) FROM validations WHERE validation_time_ms IS NOT NULL")
            avg_ms_row = cur.fetchone()
            avg_ms = int(avg_ms_row[0]) if avg_ms_row and avg_ms_row[0] is not None else None
        except sqlite3.OperationalError:
            pass
        try:
            cur = conn.execute("SELECT COUNT(*) AS c, SUM(passed) AS p FROM validations WHERE date(timestamp) = date('now')")
            today_row = cur.fetchone()
            today_count = today_row[0] or 0
            today_passed = today_row[1] or 0
        except sqlite3.OperationalError:
            pass
        try:
            cur = conn.execute("SELECT COUNT(*) AS c, SUM(passed) AS p FROM validations WHERE timestamp >= datetime('now', '-7 days')")
            week_row = cur.fetchone()
            week_count = week_row[0] or 0
            week_passed = week_row[1] or 0
        except sqlite3.OperationalError:
            pass
    dist = get_failure_distribution()
    return {
        "total": total,
        "total_validations": total,
        "pass_rate": passed / total if total else 0.0,
        "avg_score": round(avg_score, 4),
        "avg_validation_ms": avg_ms,
        "today": {"count": today_count, "pass_rate": (today_passed / today_count) if today_count else 0.0},
        "this_week": {"count": week_count, "pass_rate": (week_passed / week_count) if week_count else 0.0},
        "failure_distribution": dist,
    }


def get_recent_validations(limit: int = 20) -> List[Dict[str, Any]]:
    """Return recent validations (id, validation_id, timestamp, design_hash, artifact_id, passed, score, validation_time_ms)."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        try:
            cur = conn.execute(
                "SELECT id, validation_id, timestamp, design_hash, COALESCE(artifact_id, '') AS artifact_id, passed, score, validation_time_ms FROM validations ORDER BY id DESC LIMIT ?",
                (limit,),
            )
        except sqlite3.OperationalError:
            cur = conn.execute(
                "SELECT id, validation_id, timestamp, design_hash, passed, score FROM validations ORDER BY id DESC LIMIT ?",
                (limit,),
            )
        rows = cur.fetchall()
    out = []
    for r in rows:
        d = dict(r)
        dh = d.get("design_hash") or ""
        out.append({
            "id": d["id"],
            "validation_id": d.get("validation_id"),
            "timestamp": d["timestamp"],
            "design_hash": dh[:12] + "..." if len(dh) > 12 else dh,
            "artifact_id": d.get("artifact_id") or "",
            "passed": bool(d["passed"]),
            "score": d["score"],
            "validation_time_ms": d.get("validation_time_ms"),
        })
    return out


def get_validation_history(design_id: str, limit: int = 50) -> List[Dict[str, Any]]:
    """Return all validations for a design (by artifact_id or design_hash). design_id can be artifact_id or design_hash."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        try:
            cur = conn.execute(
                "SELECT id, validation_id, timestamp, design_hash, artifact_id, passed, score, validation_time_ms, errors_json FROM validations WHERE artifact_id = ? OR design_hash = ? ORDER BY id DESC LIMIT ?",
                (design_id, design_id, limit),
            )
        except sqlite3.OperationalError:
            cur = conn.execute(
                "SELECT id, validation_id, timestamp, design_hash, passed, score, errors_json FROM validations WHERE design_hash = ? ORDER BY id DESC LIMIT ?",
                (design_id, limit),
            )
        rows = cur.fetchall()
    out = []
    for r in rows:
        d = dict(r)
        out.append({
            "id": d["id"],
            "validation_id": d.get("validation_id"),
            "timestamp": d["timestamp"],
            "passed": bool(d["passed"]),
            "score": d["score"],
            "validation_time_ms": d.get("validation_time_ms"),
            "errors": json.loads(d.get("errors_json") or "[]"),
        })
    return out


def get_failure_distribution(limit_days: Optional[int] = None) -> Dict[str, float]:
    """Return fraction of failures per failure type (inferred from errors). If limit_days, only recent."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        if limit_days:
            cur = conn.execute(
                "SELECT errors_json FROM validations WHERE passed = 0 AND timestamp >= datetime('now', ?)",
                (f"-{limit_days} days",),
            )
        else:
            cur = conn.execute("SELECT errors_json FROM validations WHERE passed = 0")
        rows = cur.fetchall()
    counts: Dict[str, int] = {}
    for r in rows:
        errs = json.loads(r["errors_json"] or "[]")
        err_str = " ".join(str(e).upper() for e in errs)
        if "NAN" in err_str or "INF" in err_str or "UNSTABLE" in err_str or "STABILITY" in err_str:
            counts["PHYSICS_INSTABILITY"] = counts.get("PHYSICS_INSTABILITY", 0) + 1
        elif "MASS" in err_str or "STIFFNESS" in err_str:
            counts["MATERIAL_OUT_OF_RANGE"] = counts.get("MATERIAL_OUT_OF_RANGE", 0) + 1
        elif "COLLISION" in err_str or "INTERSECT" in err_str:
            counts["GEOMETRY_SELF_INTERSECTION"] = counts.get("GEOMETRY_SELF_INTERSECTION", 0) + 1
        elif "GRASP" in err_str:
            counts["GRASP_FAILURE"] = counts.get("GRASP_FAILURE", 0) + 1
        else:
            counts["UNKNOWN"] = counts.get("UNKNOWN", 0) + 1
    total = sum(counts.values()) or 1
    return {k: round(v / total, 2) for k, v in counts.items()}


def get_failures(limit: int = 50) -> List[Dict[str, Any]]:
    """Return recent failures with errors."""
    with sqlite3.connect(_DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.execute(
            "SELECT id, validation_id, error_id, timestamp, design_hash, mjcf_size, domain, score, errors_json, source FROM validations WHERE passed = 0 ORDER BY id DESC LIMIT ?",
            (limit,),
        )
        rows = cur.fetchall()
    out = []
    for r in rows:
        out.append({
            "id": r["id"],
            "validation_id": r["validation_id"],
            "error_id": r["error_id"],
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
