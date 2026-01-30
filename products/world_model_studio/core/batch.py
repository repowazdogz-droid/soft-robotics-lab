"""
World Model Studio — Batch optimization jobs.
Parameter sweep, run simulations, log to SQLite, find best parameters.
"""

import json
import sqlite3
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

_WMS_ROOT = Path(__file__).resolve().parent.parent
_DATA_DIR = _WMS_ROOT / "data"
_DB_PATH = _DATA_DIR / "batch_jobs.db"

_PRODUCTS = _WMS_ROOT.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.id_generator import run_id


def _get_conn() -> sqlite3.Connection:
    _DATA_DIR.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(_DB_PATH))
    conn.row_factory = sqlite3.Row
    return conn


def _init_db(conn: sqlite3.Connection) -> None:
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS jobs (
            id TEXT PRIMARY KEY,
            task_type TEXT NOT NULL,
            status TEXT NOT NULL,
            parameters_schema TEXT,
            created_at TEXT NOT NULL,
            completed_at TEXT
        );
        CREATE TABLE IF NOT EXISTS trials (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            job_id TEXT NOT NULL,
            parameters_json TEXT NOT NULL,
            result_json TEXT,
            success INTEGER NOT NULL,
            score REAL,
            created_at TEXT,
            FOREIGN KEY (job_id) REFERENCES jobs(id)
        );
        CREATE INDEX IF NOT EXISTS idx_trials_job ON trials(job_id);
    """)
    conn.commit()


def create_batch_job(
    task_type: str,
    parameters: Dict[str, Any],
    n_trials: int,
) -> str:
    """
    Create a batch job. parameters = dict of param_name -> list of values (sweep) or single value.
    n_trials = number of trials to run. Returns job_id.
    """
    job_id = run_id()
    conn = _get_conn()
    try:
        _init_db(conn)
        created = datetime.utcnow().isoformat() + "Z"
        conn.execute(
            "INSERT INTO jobs (id, task_type, status, parameters_schema, created_at) VALUES (?, ?, ?, ?, ?)",
            (job_id, task_type, "pending", json.dumps({"n_trials": n_trials, "sweep": parameters}), created),
        )
        conn.commit()
        return job_id
    finally:
        conn.close()


def run_batch_job(
    job_id: str,
    run_fn: Any,
    scene_path: str,
    task: Any,
    n_trials: Optional[int] = None,
) -> None:
    """
    Run all trials for a job. run_fn(sim, task, trial_params) -> {success, score, ...}.
    run_fn is called once per trial with parameters from parameter sweep or random samples.
    """
    conn = _get_conn()
    try:
        _init_db(conn)
        cur = conn.execute("SELECT task_type, parameters_schema FROM jobs WHERE id = ?", (job_id,))
        row = cur.fetchone()
        if not row:
            return
        schema = json.loads(row["parameters_schema"] or "{}")
        sweep = schema.get("sweep", {})
        n = n_trials or schema.get("n_trials", 10)
        conn.execute("UPDATE jobs SET status = ? WHERE id = ?", ("running", job_id))
        conn.commit()

        from .simulator import Simulator
        sim = Simulator()
        sim.load_scene(path=scene_path)
        sim.set_task(task)
        created = datetime.utcnow().isoformat() + "Z"

        for i in range(n):
            trial_params: Dict[str, Any] = {}
            for k, v in sweep.items():
                if isinstance(v, list):
                    trial_params[k] = v[i % len(v)] if v else None
                else:
                    trial_params[k] = v
            try:
                result = run_fn(sim, task, trial_params)
                success = 1 if result.get("success", False) else 0
                score = float(result.get("score", result.get("total_reward", 0.0)))
            except Exception as e:
                result = {"error": str(e)}
                success = 0
                score = 0.0
            conn.execute(
                "INSERT INTO trials (job_id, parameters_json, result_json, success, score, created_at) VALUES (?, ?, ?, ?, ?, ?)",
                (job_id, json.dumps(trial_params), json.dumps(result, default=str), success, score, created),
            )
        conn.execute("UPDATE jobs SET status = ?, completed_at = ? WHERE id = ?", ("completed", datetime.utcnow().isoformat() + "Z", job_id))
        conn.commit()
    except Exception:
        try:
            conn.execute("UPDATE jobs SET status = ? WHERE id = ?", ("failed", job_id))
            conn.commit()
        except Exception:
            pass
    finally:
        conn.close()


def get_batch_results(job_id: str) -> Dict[str, Any]:
    """Return job metadata and all trials (parameters_json, result_json, success, score)."""
    conn = _get_conn()
    try:
        _init_db(conn)
        cur = conn.execute("SELECT id, task_type, status, parameters_schema, created_at, completed_at FROM jobs WHERE id = ?", (job_id,))
        row = cur.fetchone()
        if not row:
            return {"job_id": job_id, "found": False}
        job = dict(row)
        job["parameters_schema"] = json.loads(job["parameters_schema"]) if job.get("parameters_schema") else {}
        cur = conn.execute("SELECT id, job_id, parameters_json, result_json, success, score, created_at FROM trials WHERE job_id = ? ORDER BY id", (job_id,))
        trials = []
        for r in cur.fetchall():
            d = dict(r)
            d["parameters"] = json.loads(d["parameters_json"]) if d.get("parameters_json") else {}
            d["result"] = json.loads(d["result_json"]) if d.get("result_json") else {}
            trials.append(d)
        return {"job_id": job_id, "found": True, "job": job, "trials": trials}
    finally:
        conn.close()


def find_best_parameters(job_id: str) -> Dict[str, Any]:
    """Return best trial parameters (highest score) and its score."""
    results = get_batch_results(job_id)
    if not results.get("found") or not results.get("trials"):
        return {"job_id": job_id, "best_parameters": {}, "best_score": None}
    trials = results["trials"]
    best = max(trials, key=lambda t: float(t.get("score") or 0))
    return {
        "job_id": job_id,
        "best_parameters": best.get("parameters", {}),
        "best_score": best.get("score"),
        "best_trial_id": best.get("id"),
    }


def list_jobs(limit: int = 50) -> List[Dict[str, Any]]:
    """List recent jobs (id, task_type, status, created_at)."""
    conn = _get_conn()
    try:
        _init_db(conn)
        cur = conn.execute("SELECT id, task_type, status, created_at, completed_at FROM jobs ORDER BY created_at DESC LIMIT ?", (limit,))
        return [dict(r) for r in cur.fetchall()]
    finally:
        conn.close()
