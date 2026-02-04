"""
Experiment Registry

Cognitive index of all runs. Enables querying the research space.
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

REGISTRY_PATH = Path(__file__).parent.parent / "registry" / "experiment_index.json"
DISCOVERY_SURFACE_PATH = (
    Path(__file__).parent.parent / "registry" / "discovery_surface.json"
)


def ensure_registry():
    """Ensure registry file exists"""
    REGISTRY_PATH.parent.mkdir(parents=True, exist_ok=True)
    if not REGISTRY_PATH.exists():
        with open(REGISTRY_PATH, "w") as f:
            json.dump({
                "version": "1.0",
                "description": "Experiment registry - cognitive index of all runs",
                "created_at": datetime.utcnow().isoformat() + "Z",
                "runs": []
            }, f, indent=2)


def index_run(
    run_id: str,
    hypothesis_id: str,
    design_id: str,
    environment: str,
    primary_metric: float,
    primary_metric_name: str = "slip_rate",
    outcome_direction: str = "ambiguous",
    confidence_delta: float = 0.0,
    batch_id: Optional[str] = None,
    surprise_score: Optional[int] = None,  # 1-5, manual rating
) -> Dict:
    """Add run to experiment index"""
    ensure_registry()

    entry = {
        "run_id": run_id,
        "hypothesis_id": hypothesis_id,
        "design_id": design_id,
        "environment": environment,
        "primary_metric": primary_metric,
        "primary_metric_name": primary_metric_name,
        "outcome_direction": outcome_direction,
        "confidence_delta": confidence_delta,
        "batch_id": batch_id,
        "surprise_score": surprise_score,
        "indexed_at": datetime.utcnow().isoformat() + "Z",
    }

    with open(REGISTRY_PATH, "r") as f:
        registry = json.load(f)

    registry["runs"].append(entry)

    with open(REGISTRY_PATH, "w") as f:
        json.dump(registry, f, indent=2)

    return entry


def query_runs(
    hypothesis_id: Optional[str] = None,
    environment: Optional[str] = None,
    design_id: Optional[str] = None,
    min_metric: Optional[float] = None,
    max_metric: Optional[float] = None,
    outcome_direction: Optional[str] = None,
) -> List[Dict]:
    """Query experiment index"""
    ensure_registry()

    with open(REGISTRY_PATH, "r") as f:
        registry = json.load(f)

    results = registry["runs"]

    if hypothesis_id:
        results = [r for r in results if r.get("hypothesis_id") == hypothesis_id]
    if environment:
        results = [r for r in results if r.get("environment") == environment]
    if design_id:
        results = [r for r in results if r.get("design_id") == design_id]
    if min_metric is not None:
        results = [r for r in results if r.get("primary_metric", 0) >= min_metric]
    if max_metric is not None:
        results = [r for r in results if r.get("primary_metric", 999) <= max_metric]
    if outcome_direction:
        results = [
            r for r in results if r.get("outcome_direction") == outcome_direction
        ]

    return results


def get_registry_stats() -> Dict:
    """Get summary statistics"""
    ensure_registry()

    with open(REGISTRY_PATH, "r") as f:
        registry = json.load(f)

    runs = registry["runs"]

    if not runs:
        return {"total_runs": 0}

    return {
        "total_runs": len(runs),
        "hypotheses": len(set(r.get("hypothesis_id") for r in runs)),
        "designs": len(set(r.get("design_id") for r in runs)),
        "environments": list(set(r.get("environment") for r in runs)),
        "outcomes": {
            "supports": sum(
                1 for r in runs if r.get("outcome_direction") == "supports"
            ),
            "refutes": sum(
                1 for r in runs if r.get("outcome_direction") == "refutes"
            ),
            "ambiguous": sum(
                1 for r in runs if r.get("outcome_direction") == "ambiguous"
            ),
        },
    }


# --- Discovery Surface ---

def ensure_discovery_surface():
    """Ensure discovery surface file exists"""
    DISCOVERY_SURFACE_PATH.parent.mkdir(parents=True, exist_ok=True)
    if not DISCOVERY_SURFACE_PATH.exists():
        with open(DISCOVERY_SURFACE_PATH, "w") as f:
            json.dump({
                "version": "1.0",
                "description": "Discovery surface - maps the research terrain",
                "created_at": datetime.utcnow().isoformat() + "Z",
                "points": []
            }, f, indent=2)


def add_surface_point(
    run_id: str,
    hypothesis_id: str,
    parameter_vector: Dict[str, Any],
    environment: str,
    result_score: float,
    confidence_delta: float,
    novelty_flag: bool = False,
    exploration_mode: str = "exploit",  # exploit | explore | random
    information_gain: Optional[int] = None,  # 1-5
    elegance_score: Optional[int] = None,  # 1-5
    surprise_score: Optional[int] = None  # 1-5
) -> Dict:
    """
    Add a point to the discovery surface.

    This maps the research terrain:
    - Flat zones → predictable results
    - Slopes → parameter sensitivity
    - Cliffs → failure boundaries
    - Fog → unexplored regions
    - Spikes → anomalies
    - Glow → unexpected success
    """
    ensure_discovery_surface()

    point = {
        "run_id": run_id,
        "hypothesis_id": hypothesis_id,
        "parameter_vector": parameter_vector,
        "environment": environment,
        "result_score": result_score,
        "confidence_delta": confidence_delta,
        "novelty_flag": novelty_flag,
        "exploration_mode": exploration_mode,
        "information_gain": information_gain,
        "elegance_score": elegance_score,
        "surprise_score": surprise_score,
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }

    with open(DISCOVERY_SURFACE_PATH, "r") as f:
        surface = json.load(f)

    surface["points"].append(point)

    with open(DISCOVERY_SURFACE_PATH, "w") as f:
        json.dump(surface, f, indent=2)

    return point


def get_undersampled_regions(min_samples: int = 3) -> List[Dict]:
    """Find regions of parameter space with few samples"""
    ensure_discovery_surface()

    with open(DISCOVERY_SURFACE_PATH, "r") as f:
        surface = json.load(f)

    env_counts = {}
    for p in surface["points"]:
        env = p.get("environment", "unknown")
        env_counts[env] = env_counts.get(env, 0) + 1

    undersampled = [
        {"environment": env, "count": count}
        for env, count in env_counts.items()
        if count < min_samples
    ]

    return undersampled


def get_high_information_runs(min_gain: int = 4) -> List[Dict]:
    """Find runs with high information gain (breakthroughs cluster here)"""
    ensure_discovery_surface()

    with open(DISCOVERY_SURFACE_PATH, "r") as f:
        surface = json.load(f)

    return [
        p for p in surface["points"]
        if p.get("information_gain") and p["information_gain"] >= min_gain
    ]


def get_anomalies() -> List[Dict]:
    """Find surprising results (novelty_flag or high surprise_score)"""
    ensure_discovery_surface()

    with open(DISCOVERY_SURFACE_PATH, "r") as f:
        surface = json.load(f)

    return [
        p for p in surface["points"]
        if p.get("novelty_flag") or (p.get("surprise_score") and p["surprise_score"] >= 4)
    ]


def get_surface_stats() -> Dict:
    """Get discovery surface statistics"""
    ensure_discovery_surface()

    with open(DISCOVERY_SURFACE_PATH, "r") as f:
        surface = json.load(f)

    points = surface["points"]

    if not points:
        return {"total_points": 0}

    modes = {}
    for p in points:
        mode = p.get("exploration_mode", "unknown")
        modes[mode] = modes.get(mode, 0) + 1

    info_gains = [p["information_gain"] for p in points if p.get("information_gain")]
    elegance = [p["elegance_score"] for p in points if p.get("elegance_score")]
    surprises = [p["surprise_score"] for p in points if p.get("surprise_score")]

    return {
        "total_points": len(points),
        "exploration_modes": modes,
        "novelty_flags": sum(1 for p in points if p.get("novelty_flag")),
        "avg_information_gain": sum(info_gains) / len(info_gains) if info_gains else None,
        "avg_elegance_score": sum(elegance) / len(elegance) if elegance else None,
        "avg_surprise_score": sum(surprises) / len(surprises) if surprises else None,
        "high_information_runs": len([g for g in info_gains if g >= 4]),
        "anomalies": len(get_anomalies())
    }
