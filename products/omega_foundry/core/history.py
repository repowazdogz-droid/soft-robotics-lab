"""
OMEGA Foundry — Design history and versioning.
SQLite: data/design_history.db. Tables: designs, versions.
Track iterations: design_id, version, mjcf, params, timestamp, notes. Diff and restore.
"""

import json
import sqlite3
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_FOUNDRY_ROOT = Path(__file__).resolve().parent.parent
_DATA_DIR = _FOUNDRY_ROOT / "data"
_DB_PATH = _DATA_DIR / "design_history.db"


def _get_conn():
    _DATA_DIR.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(_DB_PATH))
    conn.row_factory = sqlite3.Row
    return conn


def _init_db(conn: sqlite3.Connection) -> None:
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS designs (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            current_version INTEGER NOT NULL DEFAULT 1
        );
        CREATE TABLE IF NOT EXISTS versions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            design_id TEXT NOT NULL,
            version INTEGER NOT NULL,
            mjcf TEXT,
            params_json TEXT,
            created_at TEXT NOT NULL,
            notes TEXT,
            FOREIGN KEY (design_id) REFERENCES designs(id)
        );
        CREATE INDEX IF NOT EXISTS idx_versions_design_id ON versions(design_id);
    """)
    conn.commit()


def save_version(
    design_id: str,
    name: str,
    mjcf: str,
    params: Optional[Dict[str, Any]] = None,
    notes: Optional[str] = None,
) -> int:
    """
    Save a new version for design. Creates design row if new. Returns version number.
    """
    conn = _get_conn()
    try:
        _init_db(conn)
        cursor = conn.cursor()
        cursor.execute("SELECT current_version FROM designs WHERE id = ?", (design_id,))
        row = cursor.fetchone()
        if row:
            version = row["current_version"] + 1
            cursor.execute("UPDATE designs SET name = ?, current_version = ? WHERE id = ?", (name, version, design_id))
        else:
            version = 1
            cursor.execute("INSERT INTO designs (id, name, current_version) VALUES (?, ?, ?)", (design_id, name, version))

        from datetime import datetime
        created_at = datetime.utcnow().isoformat() + "Z"
        params_json = json.dumps(params, default=str) if params else None
        cursor.execute(
            "INSERT INTO versions (design_id, version, mjcf, params_json, created_at, notes) VALUES (?, ?, ?, ?, ?, ?)",
            (design_id, version, mjcf, params_json, created_at, notes or ""),
        )
        conn.commit()
        return version
    finally:
        conn.close()


def list_versions(design_id: str) -> List[Dict[str, Any]]:
    """List all versions for a design, newest first."""
    conn = _get_conn()
    try:
        _init_db(conn)
        cursor = conn.cursor()
        cursor.execute(
            "SELECT id, design_id, version, created_at, notes FROM versions WHERE design_id = ? ORDER BY version DESC",
            (design_id,),
        )
        return [dict(row) for row in cursor.fetchall()]
    finally:
        conn.close()


def get_version(design_id: str, version: Optional[int] = None) -> Optional[Dict[str, Any]]:
    """Get a specific version (or current if version is None). Returns mjcf, params_json, created_at, notes."""
    conn = _get_conn()
    try:
        _init_db(conn)
        cursor = conn.cursor()
        if version is None:
            cursor.execute("SELECT current_version FROM designs WHERE id = ?", (design_id,))
            row = cursor.fetchone()
            version = row["current_version"] if row else 1
        cursor.execute(
            "SELECT design_id, version, mjcf, params_json, created_at, notes FROM versions WHERE design_id = ? AND version = ?",
            (design_id, version),
        )
        row = cursor.fetchone()
        if not row:
            return None
        out = dict(row)
        if out.get("params_json"):
            try:
                out["params"] = json.loads(out["params_json"])
            except Exception:
                out["params"] = {}
        return out
    finally:
        conn.close()


def diff_versions(design_id: str, version_a: int, version_b: int) -> Tuple[str, str, str]:
    """
    Compare two versions. Returns (mjcf_a, mjcf_b, diff_summary).
    diff_summary is a short text description of changes (e.g. line count diff).
    """
    va = get_version(design_id, version_a)
    vb = get_version(design_id, version_b)
    if not va or not vb:
        return ("", "", "Version not found")
    mjcf_a = va.get("mjcf") or ""
    mjcf_b = vb.get("mjcf") or ""
    lines_a = len(mjcf_a.splitlines())
    lines_b = len(mjcf_b.splitlines())
    summary = f"v{version_a}: {lines_a} lines → v{version_b}: {lines_b} lines"
    if mjcf_a != mjcf_b:
        summary += " (content changed)"
    return (mjcf_a, mjcf_b, summary)


def restore_version(design_id: str, version: int) -> Optional[str]:
    """Return MJCF content of the given version (for restore/download). Does not change current_version."""
    v = get_version(design_id, version)
    return v.get("mjcf") if v else None


def get_current_version_number(design_id: str) -> int:
    """Return current version number for design, or 0 if not found."""
    conn = _get_conn()
    try:
        _init_db(conn)
        cursor = conn.cursor()
        cursor.execute("SELECT current_version FROM designs WHERE id = ?", (design_id,))
        row = cursor.fetchone()
        return row["current_version"] if row else 0
    finally:
        conn.close()


def list_designs() -> List[Dict[str, Any]]:
    """List all designs (id, name, current_version)."""
    conn = _get_conn()
    try:
        _init_db(conn)
        cursor = conn.cursor()
        cursor.execute("SELECT id, name, current_version FROM designs ORDER BY id")
        return [dict(row) for row in cursor.fetchall()]
    finally:
        conn.close()
