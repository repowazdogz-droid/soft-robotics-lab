"""
System Health Check

Run this to verify Lab OS infrastructure is healthy.
"""

import json
import os
import sqlite3
from datetime import datetime
from pathlib import Path

import requests

# Paths
BASE_DIR = Path(__file__).parent.parent
DB_PATH = BASE_DIR / "db" / "omega.db"
ARTIFACTS_DIR = BASE_DIR / "artifacts"
REGISTRY_PATH = BASE_DIR / "registry" / "experiment_index.json"
SURFACE_PATH = BASE_DIR / "registry" / "discovery_surface.json"
# Lab OS URL (env for cross-machine; default local 18002)
LAB_OS_URL = os.environ.get("LAB_OS_URL", "http://localhost:18002")


def check_lab_os() -> dict:
    """Check if Lab OS API is reachable"""
    try:
        r = requests.get(f"{LAB_OS_URL}/health", timeout=5)
        return {"status": "ok", "response": r.json()}
    except requests.exceptions.ConnectionError:
        return {"status": "unreachable", "error": "Connection refused"}
    except Exception as e:
        return {"status": "error", "error": str(e)}


def check_database() -> dict:
    """Check if database is accessible and has correct tables"""
    if not DB_PATH.exists():
        return {"status": "missing", "path": str(DB_PATH)}

    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()

        # Check tables exist
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
        tables = [row[0] for row in cursor.fetchall()]

        expected = ["hypotheses", "evidence", "runs"]
        missing = [t for t in expected if t not in tables]

        # Count rows
        counts = {}
        for table in expected:
            if table in tables:
                cursor.execute(f"SELECT COUNT(*) FROM {table}")
                counts[table] = cursor.fetchone()[0]

        conn.close()

        if missing:
            return {
                "status": "incomplete",
                "missing_tables": missing,
                "counts": counts,
            }

        return {"status": "ok", "tables": tables, "counts": counts}

    except Exception as e:
        return {"status": "error", "error": str(e)}


def check_artifacts_dir() -> dict:
    """Check artifacts directory"""
    if not ARTIFACTS_DIR.exists():
        return {"status": "missing", "path": str(ARTIFACTS_DIR)}

    # Count run directories
    run_dirs = [
        d
        for d in ARTIFACTS_DIR.iterdir()
        if d.is_dir() and d.name.startswith("R-")
    ]

    # Check write permissions
    try:
        test_file = ARTIFACTS_DIR / ".write_test"
        test_file.write_text("test")
        test_file.unlink()
        writable = True
    except Exception:
        writable = False

    return {
        "status": "ok" if writable else "read_only",
        "path": str(ARTIFACTS_DIR),
        "run_count": len(run_dirs),
        "writable": writable,
    }


def check_registry() -> dict:
    """Check experiment registry"""
    if not REGISTRY_PATH.exists():
        return {"status": "missing", "path": str(REGISTRY_PATH)}

    try:
        with open(REGISTRY_PATH) as f:
            registry = json.load(f)

        return {
            "status": "ok",
            "version": registry.get("version"),
            "run_count": len(registry.get("runs", [])),
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}


def check_digests_dir() -> dict:
    """Check digests directory"""
    digest_dir = BASE_DIR / "digests"
    if not digest_dir.exists():
        return {"status": "missing", "path": str(digest_dir)}

    digests = list(digest_dir.glob("digest_*.json"))
    return {
        "status": "ok",
        "path": str(digest_dir),
        "digest_count": len(digests)
    }


def check_discovery_surface() -> dict:
    """Check discovery surface"""
    if not SURFACE_PATH.exists():
        return {"status": "missing", "path": str(SURFACE_PATH)}

    try:
        with open(SURFACE_PATH) as f:
            surface = json.load(f)

        points = surface.get("points", [])
        novelty_count = sum(1 for p in points if p.get("novelty_flag"))

        return {
            "status": "ok",
            "version": surface.get("version"),
            "total_points": len(points),
            "novelty_flags": novelty_count,
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}


def run_health_check() -> dict:
    """Run full system health check"""
    print("\n" + "=" * 60)
    print("OMEGA LAB SYSTEM HEALTH CHECK")
    print(f"Time: {datetime.now().isoformat()}")
    print("=" * 60 + "\n")

    results = {
        "timestamp": datetime.now().isoformat(),
        "checks": {},
    }

    # Lab OS
    print("Checking Lab OS API...")
    lab_os = check_lab_os()
    results["checks"]["lab_os"] = lab_os
    print(f"  → {lab_os['status']}")

    # Database
    print("Checking database...")
    db = check_database()
    results["checks"]["database"] = db
    print(f"  → {db['status']}")
    if db.get("counts"):
        for table, count in db["counts"].items():
            print(f"     {table}: {count} rows")

    # Artifacts
    print("Checking artifacts directory...")
    artifacts = check_artifacts_dir()
    results["checks"]["artifacts"] = artifacts
    print(f"  → {artifacts['status']} ({artifacts.get('run_count', 0)} runs)")

    # Registry
    print("Checking experiment registry...")
    registry = check_registry()
    results["checks"]["registry"] = registry
    print(f"  → {registry['status']}")

    # Discovery Surface
    print("Checking discovery surface...")
    surface = check_discovery_surface()
    results["checks"]["discovery_surface"] = surface
    print(f"  → {surface['status']} ({surface.get('total_points', 0)} points)")

    # Digests
    print("Checking digests directory...")
    digests = check_digests_dir()
    results["checks"]["digests"] = digests
    print(f"  → {digests['status']} ({digests.get('digest_count', 0)} digests)")

    # Overall status
    all_ok = all(
        c.get("status") == "ok" for c in results["checks"].values()
    )

    results["overall"] = "healthy" if all_ok else "degraded"

    print("\n" + "=" * 60)
    print(f"OVERALL STATUS: {results['overall'].upper()}")
    print("=" * 60 + "\n")

    return results


def rebuild_test() -> bool:
    """
    Test: Can we rebuild the lab from artifacts alone?

    This verifies the core principle:
    'If the Mac Mini died tonight, could I rebuild from artifacts?'
    """
    print("\n" + "=" * 60)
    print("ARTIFACT REBUILD TEST")
    print("=" * 60 + "\n")

    if not ARTIFACTS_DIR.exists():
        print("✗ Artifacts directory missing")
        return False

    run_dirs = [
        d
        for d in ARTIFACTS_DIR.iterdir()
        if d.is_dir() and d.name.startswith("R-")
    ]

    if not run_dirs:
        print("✗ No run artifacts found")
        return False

    valid_runs = 0
    invalid_runs = 0

    for run_dir in run_dirs:
        run_json = run_dir / "run.json"
        if run_json.exists():
            try:
                with open(run_json) as f:
                    bundle = json.load(f)

                # Check required fields
                required = ["run_id", "hypothesis_id", "metrics", "outcome"]
                if all(k in bundle for k in required):
                    valid_runs += 1
                else:
                    invalid_runs += 1
                    print(f"  ⚠ {run_dir.name}: missing required fields")
            except Exception:
                invalid_runs += 1
                print(f"  ✗ {run_dir.name}: invalid JSON")
        else:
            invalid_runs += 1
            print(f"  ✗ {run_dir.name}: no run.json")

    print(f"\nValid runs: {valid_runs}")
    print(f"Invalid runs: {invalid_runs}")

    if invalid_runs == 0:
        print("\n✓ REBUILD TEST PASSED")
        print("  All artifacts are self-contained and valid.")
        print("  Lab could be rebuilt from artifacts alone.")
        return True
    else:
        print("\n⚠ REBUILD TEST WARNING")
        print(f"  {invalid_runs} artifacts need attention.")
        return False


if __name__ == "__main__":
    results = run_health_check()
    print()
    rebuild_test()
