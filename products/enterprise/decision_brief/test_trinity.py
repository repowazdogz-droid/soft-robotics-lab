#!/usr/bin/env python3
"""
Test Translation Trinity (SRFC, TSRFC, VRFC) with the three user scenarios.
Run from repo root: PYTHONPATH=products python products/enterprise/decision_brief/test_trinity.py
Or from products: python enterprise/decision_brief/test_trinity.py
"""
import sys
from pathlib import Path

# Ensure products is on path
_REPO = Path(__file__).resolve().parent.parent.parent.parent
_PRODUCTS = _REPO / "products"
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))

from enterprise.decision_brief.decision_brief import (
    _compute_validation_trinity,
    _check_params_complete,
    _detect_domains,
)

def run_test(name: str, query: str, params: dict, domains_override: list = None):
    domains = domains_override if domains_override is not None else _detect_domains(query)
    complete, missing = _check_params_complete(params, domains)
    srfc, srfc_r, tsrfc, tsrfc_r, vrfc, vrfc_r = _compute_validation_trinity(
        query, params, complete, domains
    )
    return {
        "name": name,
        "query": query[:60] + "..." if len(query) > 60 else query,
        "domains": domains,
        "complete": complete,
        "missing": missing,
        "srfc": srfc,
        "srfc_reason": srfc_r,
        "tsrfc": tsrfc,
        "tsrfc_reason": tsrfc_r,
        "vrfc": vrfc,
        "vrfc_reason": vrfc_r,
    }

def main():
    print("=" * 70)
    print("Translation Trinity (SRFC, TSRFC, VRFC) — Test Run")
    print("=" * 70)

    # Test 1: SRFC — soft gripper / egg (physical feasibility)
    t1 = run_test(
        "SRFC: Soft gripper / egg",
        "A soft gripper made of DragonSkin 30 silicone that can pick up a raw egg without breaking it",
        {"capability": "pick egg", "environment": "lab"},
        domains_override=["robotics"],
    )
    print("\n[Test 1] SRFC — Physical feasibility (soft gripper / egg)")
    print(f"  Query: {t1['query']}")
    print(f"  Domains: {t1['domains']} | Params complete: {t1['complete']} | Missing: {t1['missing']}")
    print(f"  SRFC:   {t1['srfc']} — {t1['srfc_reason']}")
    print(f"  TSRFC:  {t1['tsrfc']} — {t1['tsrfc_reason']}")
    print(f"  VRFC:   {t1['vrfc']} — {t1['vrfc_reason']}")

    # Test 2: TSRFC — spine / AI (workflow replacement)
    t2 = run_test(
        "TSRFC: Spine surgery / AI",
        "Replace manual spine surgery case synthesis with AI-assisted cognitive support",
        {"horizon": 1, "budget": 100},
        domains_override=["general"],
    )
    print("\n[Test 2] TSRFC — Workflow replacement (spine / AI)")
    print(f"  Query: {t2['query']}")
    print(f"  Domains: {t2['domains']} | Params complete: {t2['complete']} | Missing: {t2['missing']}")
    print(f"  SRFC:   {t2['srfc']} — {t2['srfc_reason']}")
    print(f"  TSRFC:  {t2['tsrfc']} — {t2['tsrfc_reason']}")
    print(f"  VRFC:   {t2['vrfc']} — {t2['vrfc_reason']}")

    # Test 3: VRFC — Reflect for Schools (real-world viability)
    t3 = run_test(
        "VRFC: Reflect for Schools deployment",
        "Deploy Reflect for Schools to UK state schools",
        {"horizon": 2, "budget": 500},
        domains_override=["research"],
    )
    print("\n[Test 3] VRFC — Real-world viability (Reflect for Schools)")
    print(f"  Query: {t3['query']}")
    print(f"  Domains: {t3['domains']} | Params complete: {t3['complete']} | Missing: {t3['missing']}")
    print(f"  SRFC:   {t3['srfc']} — {t3['srfc_reason']}")
    print(f"  TSRFC:  {t3['tsrfc']} — {t3['tsrfc_reason']}")
    print(f"  VRFC:   {t3['vrfc']} — {t3['vrfc_reason']}")

    # Test 4: Incomplete params (should yield AMBER deferred)
    t4 = run_test(
        "Incomplete params (deferred)",
        "A soft gripper for eggs",
        {},
        domains_override=["robotics"],
    )
    print("\n[Test 4] Incomplete params (robotics, no params)")
    print(f"  Domains: {t4['domains']} | Params complete: {t4['complete']} | Missing: {t4['missing']}")
    print(f"  SRFC:   {t4['srfc']} — {t4['srfc_reason']}")
    print(f"  TSRFC:  {t4['tsrfc']} — {t4['tsrfc_reason']}")
    print(f"  VRFC:   {t4['vrfc']} — {t4['vrfc_reason']}")

    print("\n" + "=" * 70)
    print("Done.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
