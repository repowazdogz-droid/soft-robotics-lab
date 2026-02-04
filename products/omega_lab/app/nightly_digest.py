"""
Nightly Lab Digest

Automated summary of:
- Confidence movers
- Anomalies
- Hypothesis volatility
- Surprising failures
- Underexplored regions

Run nightly. Read with coffee.
"""

import json
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Optional
from collections import defaultdict

ARTIFACTS_DIR = Path(__file__).parent.parent / "artifacts"
REGISTRY_PATH = Path(__file__).parent.parent / "registry" / "experiment_index.json"
SURFACE_PATH = Path(__file__).parent.parent / "registry" / "discovery_surface.json"
DIGEST_DIR = Path(__file__).parent.parent / "digests"


def get_recent_runs(hours: int = 24) -> List[Dict]:
    """Get runs from the last N hours"""
    cutoff = datetime.utcnow() - timedelta(hours=hours)
    recent = []

    if not ARTIFACTS_DIR.exists():
        return recent

    for run_dir in ARTIFACTS_DIR.iterdir():
        if not run_dir.is_dir() or not run_dir.name.startswith("R-"):
            continue

        run_json = run_dir / "run.json"
        if not run_json.exists():
            continue

        try:
            with open(run_json) as f:
                bundle = json.load(f)

            timestamp = bundle.get("timestamp")
            if timestamp:
                run_time = datetime.fromisoformat(timestamp.replace("Z", "+00:00").replace("+00:00", ""))
                if run_time > cutoff:
                    recent.append(bundle)
        except Exception:
            continue

    return recent


def get_confidence_movers(runs: List[Dict]) -> List[Dict]:
    """Find runs that significantly moved confidence"""
    movers = []

    for run in runs:
        outcome = run.get("outcome", {})
        strength = outcome.get("strength", 0)
        direction = outcome.get("direction")

        # Significant if strength > 0.7
        if strength > 0.7:
            movers.append({
                "run_id": run.get("run_id"),
                "hypothesis_id": run.get("hypothesis_id"),
                "direction": direction,
                "strength": strength,
                "environment": run.get("environment"),
                "rationale": outcome.get("rationale")
            })

    # Sort by strength
    movers.sort(key=lambda x: x["strength"], reverse=True)
    return movers


def get_anomalies(runs: List[Dict]) -> List[Dict]:
    """Find surprising results"""
    anomalies = []

    # Group by environment to find outliers
    by_env = defaultdict(list)
    for run in runs:
        env = run.get("environment", "unknown")
        slip = run.get("metrics", {}).get("slip_rate")
        if slip is not None:
            by_env[env].append((run, slip))

    for env, env_runs in by_env.items():
        if len(env_runs) < 3:
            continue

        slips = [s for _, s in env_runs]
        mean = sum(slips) / len(slips)
        variance = sum((s - mean) ** 2 for s in slips) / len(slips)
        std = variance ** 0.5

        if std == 0:
            continue

        for run, slip in env_runs:
            z_score = abs(slip - mean) / std
            if z_score > 2:
                anomalies.append({
                    "run_id": run.get("run_id"),
                    "environment": env,
                    "slip_rate": slip,
                    "z_score": round(z_score, 2),
                    "reason": f"Slip rate {slip:.4f} is {z_score:.1f}σ from mean {mean:.4f}"
                })

    return anomalies


def get_hypothesis_volatility(runs: List[Dict]) -> List[Dict]:
    """Find hypotheses with conflicting recent evidence"""
    by_hypothesis = defaultdict(list)

    for run in runs:
        hyp = run.get("hypothesis_id")
        outcome = run.get("outcome", {}).get("direction")
        if hyp and outcome:
            by_hypothesis[hyp].append(outcome)

    volatile = []
    for hyp, outcomes in by_hypothesis.items():
        supports = outcomes.count("supports")
        refutes = outcomes.count("refutes")

        if supports > 0 and refutes > 0:
            volatile.append({
                "hypothesis_id": hyp,
                "supports": supports,
                "refutes": refutes,
                "ambiguous": outcomes.count("ambiguous"),
                "volatility": min(supports, refutes) / max(supports, refutes)
            })

    volatile.sort(key=lambda x: x["volatility"], reverse=True)
    return volatile


def get_surprising_failures(runs: List[Dict]) -> List[Dict]:
    """Find runs that were expected to succeed but failed"""
    failures = []

    for run in runs:
        outcome = run.get("outcome", {})
        direction = outcome.get("direction")
        env = run.get("environment")

        # Dry environment should typically succeed
        if env == "dry" and direction == "refutes":
            failures.append({
                "run_id": run.get("run_id"),
                "environment": env,
                "reason": "Unexpected failure in dry environment",
                "metrics": run.get("metrics", {})
            })

        # High contact area but still refutes
        contact = run.get("metrics", {}).get("contact_area", 0)
        if contact > 0.035 and direction == "refutes":
            failures.append({
                "run_id": run.get("run_id"),
                "environment": env,
                "reason": f"Failed despite high contact area ({contact:.4f})",
                "metrics": run.get("metrics", {})
            })

    return failures


def get_underexplored_regions() -> List[Dict]:
    """Find parameter regions with few samples"""
    if not SURFACE_PATH.exists():
        return []

    try:
        with open(SURFACE_PATH) as f:
            surface = json.load(f)
    except Exception:
        return []

    points = surface.get("points", [])

    # Count by environment
    env_counts = defaultdict(int)
    for p in points:
        env_counts[p.get("environment", "unknown")] += 1

    underexplored = []
    expected_envs = ["dry", "wet", "surgical"]

    for env in expected_envs:
        count = env_counts.get(env, 0)
        if count < 10:
            underexplored.append({
                "region": f"environment: {env}",
                "samples": count,
                "recommendation": f"Need more runs in {env} environment"
            })

    return underexplored


def generate_digest(hours: int = 24) -> Dict:
    """Generate the full nightly digest"""
    runs = get_recent_runs(hours)

    digest = {
        "generated_at": datetime.utcnow().isoformat() + "Z",
        "period_hours": hours,
        "total_runs": len(runs),
        "confidence_movers": get_confidence_movers(runs),
        "anomalies": get_anomalies(runs),
        "hypothesis_volatility": get_hypothesis_volatility(runs),
        "surprising_failures": get_surprising_failures(runs),
        "underexplored_regions": get_underexplored_regions()
    }

    return digest


def save_digest(digest: Dict) -> Path:
    """Save digest to file"""
    DIGEST_DIR.mkdir(parents=True, exist_ok=True)

    date_str = datetime.utcnow().strftime("%Y-%m-%d")
    filepath = DIGEST_DIR / f"digest_{date_str}.json"

    with open(filepath, "w") as f:
        json.dump(digest, f, indent=2)

    return filepath


def print_digest(digest: Dict):
    """Print digest in readable format"""
    print("\n" + "="*60)
    print("☀️  OMEGA LAB NIGHTLY DIGEST")
    print(f"    Generated: {digest['generated_at']}")
    print(f"    Period: Last {digest['period_hours']} hours")
    print("="*60)

    print(f"\n📊 ACTIVITY: {digest['total_runs']} runs")

    # Confidence movers
    movers = digest['confidence_movers']
    print(f"\n🎯 CONFIDENCE MOVERS ({len(movers)})")
    if movers:
        for m in movers[:5]:
            arrow = "↑" if m['direction'] == "supports" else "↓" if m['direction'] == "refutes" else "↔"
            print(f"   {arrow} {m['hypothesis_id']}: {m['direction']} (strength: {m['strength']:.2f}) in {m['environment']}")
    else:
        print("   No significant movers")

    # Anomalies
    anomalies = digest['anomalies']
    print(f"\n⚠️  ANOMALIES ({len(anomalies)})")
    if anomalies:
        for a in anomalies[:5]:
            print(f"   • {a['run_id'][:16]}...: {a['reason']}")
    else:
        print("   No anomalies detected")

    # Volatility
    volatile = digest['hypothesis_volatility']
    print(f"\n🔄 HYPOTHESIS VOLATILITY ({len(volatile)})")
    if volatile:
        for v in volatile:
            print(f"   • {v['hypothesis_id']}: {v['supports']} supports vs {v['refutes']} refutes")
    else:
        print("   No conflicting evidence")

    # Surprising failures
    failures = digest['surprising_failures']
    print(f"\n❌ SURPRISING FAILURES ({len(failures)})")
    if failures:
        for f in failures[:5]:
            print(f"   • {f['run_id'][:16]}...: {f['reason']}")
    else:
        print("   No unexpected failures")

    # Underexplored
    underexplored = digest['underexplored_regions']
    print(f"\n🔍 UNDEREXPLORED REGIONS ({len(underexplored)})")
    if underexplored:
        for u in underexplored:
            print(f"   • {u['region']}: {u['samples']} samples - {u['recommendation']}")
    else:
        print("   All regions adequately sampled")

    print("\n" + "="*60)
    print("☕ Read with coffee. Steer toward discovery.")
    print("="*60 + "\n")


if __name__ == "__main__":
    # Generate and print digest
    digest = generate_digest(hours=168)  # Last week for demo
    print_digest(digest)

    # Save to file
    filepath = save_digest(digest)
    print(f"Saved to: {filepath}")
