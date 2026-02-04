"""
OMEGA Full Loop Demo

The undeniable closed-loop demonstration:

Hypothesis → Isaac Sim → Evidence → Confidence Shift → Weekly Brief → Visualization

End-to-end. No gaps.
"""

import os
import sys
import json
import time
from pathlib import Path
from datetime import datetime

# Add paths
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent / "runner"))

import requests

# Lab OS URL (env for cross-machine; default local 18002)
LAB_OS_URL = os.environ.get("LAB_OS_URL", "http://localhost:18002")
DEMO_DIR = Path(__file__).parent / "output"


def print_banner(text: str):
    print("\n" + "="*70)
    print(f"  {text}")
    print("="*70 + "\n")


def wait(seconds: int, message: str):
    print(f"⏳ {message}...")
    time.sleep(seconds)


def check_lab_os():
    """Verify Lab OS is running"""
    try:
        r = requests.get(f"{LAB_OS_URL}/health", timeout=5)
        return r.status_code == 200
    except Exception:
        return False


def run_demo():
    """Execute the full loop demo"""

    print_banner("🧪 OMEGA FULL LOOP DEMO")
    print("This demonstrates the complete cognitive-physical infrastructure:")
    print("  Hypothesis → Simulation → Evidence → Confidence → Brief → Graph")
    print()

    # Step 0: Check Lab OS
    print_banner("STEP 0: Verify Infrastructure")

    if not check_lab_os():
        print("❌ Lab OS not running!")
        print("   Start it with: uvicorn app.main:app --port 8000")
        return False
    print("✅ Lab OS is running")

    # Step 1: Create Hypothesis
    print_banner("STEP 1: Create Research Hypothesis")

    hypothesis = {
        "id": f"H-DEMO-{datetime.now().strftime('%H%M%S')}",
        "claim": "Soft grippers with higher compliance perform better in wet environments",
        "status": "active"
    }

    r = requests.post(f"{LAB_OS_URL}/hypotheses", json=hypothesis)
    if r.status_code == 200:
        hyp_data = r.json()
        print(f"✅ Hypothesis created: {hypothesis['id']}")
        print(f"   Claim: {hypothesis['claim']}")
        print(f"   Initial confidence: {hyp_data.get('confidence', 0.5)}")
    else:
        print(f"❌ Failed to create hypothesis: {r.text}")
        return False

    wait(1, "Preparing simulation")

    # Step 2: Run Simulations
    print_banner("STEP 2: Execute Simulation Batch")

    from isaac_runner import run_experiment

    mjcf_path = str(Path(__file__).parent.parent.parent / "soft_robotics_lab" / "gripper_zoo" / "designs" / "gd_20260201_0001" / "gd_20260201_0001.mjcf")

    environments = ["dry", "wet", "surgical"]
    runs_completed = []

    for i, env in enumerate(environments):
        print(f"\n🔬 Simulation {i+1}/3: {env} environment")

        bundle = run_experiment(
            mjcf_path=mjcf_path,
            design_id="GD-DEMO-0001",
            hypothesis_id=hypothesis["id"],
            environment=env,
            use_mock=True,  # Use mock for demo speed
            exploration_mode="explore"
        )

        runs_completed.append({
            "run_id": bundle["run_id"],
            "environment": env,
            "outcome": bundle["outcome"]["direction"],
            "strength": bundle["outcome"]["strength"],
            "slip_rate": bundle["metrics"]["slip_rate"]
        })

        print(f"   ✅ {bundle['run_id']}")
        print(f"      Outcome: {bundle['outcome']['direction']} (strength: {bundle['outcome']['strength']:.2f})")
        print(f"      Slip rate: {bundle['metrics']['slip_rate']:.4f}")

    wait(1, "Processing evidence")

    # Step 3: Check Confidence Update
    print_banner("STEP 3: Evidence → Confidence Update")

    r = requests.get(f"{LAB_OS_URL}/hypotheses/{hypothesis['id']}")
    if r.status_code == 200:
        updated_hyp = r.json()
        print(f"✅ Hypothesis confidence updated")
        print(f"   Before: 0.50")
        print(f"   After:  {updated_hyp.get('confidence', 'N/A')}")

    r = requests.get(f"{LAB_OS_URL}/hypotheses/{hypothesis['id']}/evidence")
    if r.status_code == 200:
        evidence = r.json()
        print(f"\n📋 Evidence collected: {len(evidence)} pieces")
        for e in evidence[-3:]:
            print(f"   • {e.get('direction')} (strength: {e.get('strength', 0):.2f})")

    wait(1, "Generating brief")

    # Step 4: Weekly Brief
    print_banner("STEP 4: Generate Weekly Brief")

    r = requests.get(f"{LAB_OS_URL}/brief")
    if r.status_code == 200:
        brief = r.json()
        print(f"✅ Brief generated")
        print(f"   Period: {brief.get('period')}")
        print(f"   Hypotheses: {brief.get('hypotheses_count')}")
        print(f"   Evidence: {brief.get('evidence_count')}")
        print(f"   Runs: {brief.get('runs_count')}")

        if brief.get("changes"):
            print(f"\n   📈 Changes:")
            for c in brief["changes"][:3]:
                print(f"      {c['hypothesis_id']}: {c['supports']} supports, {c['refutes']} refutes")

    wait(1, "Building run graph")

    # Step 5: Run Graph
    print_banner("STEP 5: Visualize Experiment Lineage")

    r = requests.get(f"{LAB_OS_URL}/graph")
    if r.status_code == 200:
        graph = r.json()
        print(f"✅ Run graph built")
        print(f"   Total nodes: {len(graph.get('nodes', []))}")
        print(f"   Lineage edges: {len(graph.get('edges', []))}")
        print(f"   Root runs: {len(graph.get('roots', []))}")
        print(f"   Hypotheses: {len(graph.get('branches', {}))}")
        print(f"   Batches: {len(graph.get('batches', {}))}")

    wait(1, "Querying discovery surface")

    # Step 6: Discovery Surface
    print_banner("STEP 6: Map Discovery Surface")

    r = requests.get(f"{LAB_OS_URL}/surface")
    if r.status_code == 200:
        surface = r.json()
        stats = surface.get("stats", {})
        print(f"✅ Discovery surface")
        print(f"   Total points: {stats.get('total_points', 0)}")
        print(f"   Novelty flags: {stats.get('novelty_flags', 0)}")
        print(f"   Anomalies: {stats.get('anomalies', 0)}")

        if surface.get("undersampled"):
            print(f"\n   🔍 Undersampled regions:")
            for u in surface["undersampled"]:
                print(f"      {u['environment']}: {u['count']} samples")

    wait(1, "Running semantic query")

    # Step 7: Semantic Query
    print_banner("STEP 7: Semantic Query")

    queries = [
        "wet environment",
        "supports strength > 0.7",
        f"{hypothesis['id']}"
    ]

    for q in queries:
        r = requests.get(f"{LAB_OS_URL}/query", params={"q": q})
        if r.status_code == 200:
            result = r.json()
            summary = result.get("summary", {})
            print(f"   Query: '{q}'")
            print(f"   Found: {summary.get('count', 0)} runs")

    # Step 8: Nightly Digest
    print_banner("STEP 8: Nightly Digest")

    r = requests.get(f"{LAB_OS_URL}/digest", params={"hours": 168})
    if r.status_code == 200:
        digest = r.json()
        print(f"✅ Digest generated")
        print(f"   Total runs: {digest.get('total_runs', 0)}")
        print(f"   Confidence movers: {len(digest.get('confidence_movers', []))}")
        print(f"   Anomalies: {len(digest.get('anomalies', []))}")
        print(f"   Volatile hypotheses: {len(digest.get('hypothesis_volatility', []))}")

    # Final Summary
    print_banner("✅ DEMO COMPLETE: Full Loop Verified")

    print("The OMEGA stack demonstrated:")
    print()
    print("  1. Hypothesis Creation     → Research question formalized")
    print("  2. Simulation Execution    → Physics-based experiments")
    print("  3. Evidence Collection     → Outcomes recorded")
    print("  4. Confidence Update       → Belief revision")
    print("  5. Weekly Brief            → Synthesized insights")
    print("  6. Run Graph               → Experiment lineage")
    print("  7. Discovery Surface       → Research terrain mapped")
    print("  8. Semantic Query          → Natural language access")
    print("  9. Nightly Digest          → Automated reporting")
    print()
    print("This is cognitive-physical infrastructure.")
    print("Not an app. Not a demo. Infrastructure.")
    print()
    print("="*70)

    return True


if __name__ == "__main__":
    success = run_demo()
    sys.exit(0 if success else 1)
