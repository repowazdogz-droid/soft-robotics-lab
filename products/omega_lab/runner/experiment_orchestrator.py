"""
Experiment Orchestrator

Given a hypothesis, generates parameter sweeps and executes batches.
This is what turns a lab tool into a discovery engine.
"""

import itertools
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any, Optional

import requests

# Ensure runner dir is on path when run as script
_runner_dir = Path(__file__).resolve().parent
if str(_runner_dir) not in sys.path:
    sys.path.insert(0, str(_runner_dir))

from isaac_runner import run_experiment, generate_run_id

LAB_OS_URL = "http://localhost:8000"


def generate_batch_id() -> str:
    """Generate unique batch ID"""
    return f"B-{datetime.now().strftime('%Y%m%d-%H%M%S')}"


def get_active_hypotheses() -> List[Dict]:
    """Pull active hypotheses from Lab OS"""
    try:
        r = requests.get(f"{LAB_OS_URL}/hypotheses")
        if r.status_code == 200:
            return [h for h in r.json() if h.get("status") == "active"]
    except Exception:
        pass
    return []


def generate_parameter_sweep(
    base_params: Dict[str, Any],
    sweep_params: Dict[str, List[Any]],
) -> List[Dict[str, Any]]:
    """
    Generate all combinations of parameters.

    Example:
        base_params = {"design_id": "GD-0001", "hypothesis_id": "H-TEST"}
        sweep_params = {
            "environment": ["dry", "wet", "surgical"],
            "wall_thickness": [1.2, 1.4, 1.6]
        }

    Returns 9 parameter combinations.
    """
    keys = list(sweep_params.keys())
    values = list(sweep_params.values())

    combinations = []
    for combo in itertools.product(*values):
        params = base_params.copy()
        for i, key in enumerate(keys):
            params[key] = combo[i]
        combinations.append(params)

    return combinations


def run_batch(
    mjcf_path: str,
    design_id: str,
    hypothesis_id: str,
    environments: List[str] = None,
    runs_per_env: int = 1,
    use_mock: bool = True,
    batch_id: str = None,
) -> Dict:
    """
    Execute a batch of experiments.

    Returns batch summary.
    """
    if environments is None:
        environments = ["dry", "wet", "surgical"]

    if batch_id is None:
        batch_id = generate_batch_id()

    print(f"\n{'='*60}")
    print(f"BATCH: {batch_id}")
    print(f"Design: {design_id}")
    print(f"Hypothesis: {hypothesis_id}")
    print(f"Environments: {environments}")
    print(f"Runs per env: {runs_per_env}")
    print(f"Total runs: {len(environments) * runs_per_env}")
    print(f"{'='*60}\n")

    results = []
    parent_id = None

    for env in environments:
        for i in range(runs_per_env):
            run_num = len(results) + 1
            total = len(environments) * runs_per_env
            print(f"\n>>> Run {run_num}/{total} ({env})")

            bundle = run_experiment(
                mjcf_path=mjcf_path,
                design_id=design_id,
                hypothesis_id=hypothesis_id,
                environment=env,
                parent_run_id=parent_id,
                use_mock=use_mock,
                batch_id=batch_id,
                schema_version="1.0",
            )

            results.append({
                "run_id": bundle["run_id"],
                "environment": env,
                "outcome": bundle["outcome"]["direction"],
                "strength": bundle["outcome"]["strength"],
                "metrics": bundle["metrics"],
            })

            parent_id = bundle["run_id"]

    # Batch summary
    supports = sum(1 for r in results if r["outcome"] == "supports")
    refutes = sum(1 for r in results if r["outcome"] == "refutes")
    ambiguous = sum(1 for r in results if r["outcome"] == "ambiguous")

    summary = {
        "batch_id": batch_id,
        "hypothesis_id": hypothesis_id,
        "design_id": design_id,
        "total_runs": len(results),
        "supports": supports,
        "refutes": refutes,
        "ambiguous": ambiguous,
        "success_rate": supports / len(results) if results else 0,
        "results": results,
    }

    print(f"\n{'='*60}")
    print(f"BATCH COMPLETE: {batch_id}")
    print(f"Results: {supports} supports, {refutes} refutes, {ambiguous} ambiguous")
    print(f"Success rate: {summary['success_rate']:.1%}")
    print(f"{'='*60}\n")

    return summary


def run_hypothesis_sweep(
    hypothesis_id: str,
    mjcf_paths: List[str],
    design_ids: List[str],
    environments: List[str] = None,
    use_mock: bool = True,
) -> Dict:
    """
    Run a full sweep for a hypothesis across multiple designs.
    """
    if environments is None:
        environments = ["dry", "wet", "surgical"]

    batch_id = generate_batch_id()

    all_results = []

    for mjcf_path, design_id in zip(mjcf_paths, design_ids):
        summary = run_batch(
            mjcf_path=mjcf_path,
            design_id=design_id,
            hypothesis_id=hypothesis_id,
            environments=environments,
            runs_per_env=1,
            use_mock=use_mock,
            batch_id=batch_id,
        )
        all_results.append(summary)

    return {
        "batch_id": batch_id,
        "hypothesis_id": hypothesis_id,
        "designs_tested": len(design_ids),
        "total_runs": sum(s["total_runs"] for s in all_results),
        "summaries": all_results,
    }


def stress_test(n_runs: int = 50, use_mock: bool = True) -> Dict:
    """
    Fire off many runs to stress test the system.
    """
    batch_id = generate_batch_id()

    print(f"\n{'='*60}")
    print(f"STRESS TEST: {n_runs} runs")
    print(f"Batch: {batch_id}")
    print(f"{'='*60}\n")

    # Use a sample MJCF
    mjcf_path = (
        Path(__file__).parent.parent.parent
        / "soft_robotics_lab"
        / "gripper_zoo"
        / "designs"
        / "gd_20260201_0001"
        / "gd_20260201_0001.mjcf"
    )

    environments = ["dry", "wet", "surgical"]
    results = []

    for i in range(n_runs):
        env = environments[i % len(environments)]
        design_id = f"GD-STRESS-{i:04d}"

        print(f"\n>>> Stress run {i+1}/{n_runs}")

        bundle = run_experiment(
            mjcf_path=str(mjcf_path),
            design_id=design_id,
            hypothesis_id="H-TEST",
            environment=env,
            use_mock=use_mock,
            batch_id=batch_id,
            schema_version="1.0",
        )

        results.append({
            "run_id": bundle["run_id"],
            "outcome": bundle["outcome"]["direction"],
        })

    supports = sum(1 for r in results if r["outcome"] == "supports")

    print(f"\n{'='*60}")
    print(f"STRESS TEST COMPLETE")
    print(f"Runs: {len(results)}")
    print(f"Supports: {supports} ({supports/len(results):.1%})")
    print(f"{'='*60}\n")

    return {
        "batch_id": batch_id,
        "total_runs": len(results),
        "supports": supports,
    }


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "stress":
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 50
        stress_test(n_runs=n)
    else:
        # Example batch
        mjcf_path = str(
            Path(__file__).parent.parent.parent
            / "soft_robotics_lab"
            / "gripper_zoo"
            / "designs"
            / "gd_20260201_0001"
            / "gd_20260201_0001.mjcf"
        )

        run_batch(
            mjcf_path=mjcf_path,
            design_id="GD-0001",
            hypothesis_id="H-TEST",
            environments=["dry", "wet", "surgical"],
            runs_per_env=2,
            use_mock=True,
        )
