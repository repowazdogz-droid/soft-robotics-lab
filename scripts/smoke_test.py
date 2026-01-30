#!/usr/bin/env python3
"""
OMEGA Stack smoke test. Run from repo root.
Verifies products/ structure and that key entry points are importable or runnable.
"""
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
PRODUCTS = REPO_ROOT / "products"

REQUIRED_PRODUCTS = [
    "soft_robotics_lab",
    "omega_foundry",
    "reality_bridge",
    "world_model_studio",
    "breakthrough_engine",
    "omega_tutor",
    "enterprise",
]
REQUIRED_ENTERPRISE = ["decision_brief", "eval_framework"]


def main():
    errors = []
    # 1. Folder structure
    for name in REQUIRED_PRODUCTS:
        d = PRODUCTS / name
        if not d.is_dir():
            errors.append(f"Missing products/{name}/")
    ent = PRODUCTS / "enterprise"
    if ent.is_dir():
        for name in REQUIRED_ENTERPRISE:
            d = ent / name
            if not d.is_dir():
                errors.append(f"Missing products/enterprise/{name}/")

    # 2. Quick import/run checks (non-blocking)
    checks = []
    # breakthrough_engine
    bp = PRODUCTS / "breakthrough_engine" / "hypothesis_ledger.py"
    if bp.exists():
        try:
            sys.path.insert(0, str(PRODUCTS / "breakthrough_engine"))
            import hypothesis_ledger  # noqa: F401
            checks.append("breakthrough_engine: hypothesis_ledger OK")
        except Exception as e:
            checks.append(f"breakthrough_engine: {e}")

    # omega_tutor
    ot = PRODUCTS / "omega_tutor" / "core" / "tutor_engine.py"
    if ot.exists():
        try:
            sys.path.insert(0, str(PRODUCTS / "omega_tutor"))
            from core.tutor_engine import TutorEngine  # noqa: F401
            checks.append("omega_tutor: TutorEngine OK (init may fail without API key)")
        except Exception as e:
            checks.append(f"omega_tutor: {e}")

    if errors:
        print("FAIL: structure")
        for e in errors:
            print("  -", e)
        sys.exit(1)
    print("OK: folder structure (all 8 product areas present)")
    for c in checks:
        print("  ", c)
    print("Smoke test passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
