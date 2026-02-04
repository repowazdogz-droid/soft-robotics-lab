"""
Isaac Sim Experiment Runner

Executes: MJCF → USD → Isaac Sim → Metrics → Run Bundle → Lab OS

Usage (from Isaac Sim Python):
    python isaac_runner.py --mjcf <path> --design <id> --hypothesis <id> --environment <dry|wet|surgical>

Usage (mock mode for testing):
    python isaac_runner.py --mjcf <path> --design <id> --hypothesis <id> --mock
"""

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Dict, Any

import requests

# Configuration
LAB_OS_URL = "http://localhost:8000"
ARTIFACTS_BASE = Path(__file__).parent.parent / "artifacts"

# Check for Isaac Sim
ISAAC_AVAILABLE = False
try:
    # Only import if running inside Isaac Sim Python
    if "omni" in sys.modules or os.environ.get("ISAAC_SIM"):
        from omni.isaac.kit import SimulationApp
        ISAAC_AVAILABLE = True
except ImportError:
    pass


def generate_run_id() -> str:
    """Generate unique run ID with timestamp"""
    return f"R-{datetime.now().strftime('%Y%m%d-%H%M%S-%f')[:19]}"


def generate_experiment_id(design_id: str, environment: str) -> str:
    """Generate experiment ID from design and environment"""
    return f"E-{design_id}-{environment}"


class IsaacRunner:
    """Runs gripper simulations in Isaac Sim"""

    def __init__(self, headless: bool = True):
        self.headless = headless
        self.simulation_app = None

    def __enter__(self):
        if ISAAC_AVAILABLE:
            self.simulation_app = SimulationApp({"headless": self.headless})
        return self

    def __exit__(self, *args):
        if self.simulation_app:
            self.simulation_app.close()

    def load_mjcf(self, mjcf_path: str) -> str:
        """Load MJCF and return prim path"""
        if not ISAAC_AVAILABLE:
            raise RuntimeError("Isaac Sim not available")

        import omni.isaac.core.utils.stage as stage_utils
        from omni.importer.mjcf import _mjcf

        # Import MJCF
        mjcf_importer = _mjcf.MJCFImporter()
        result = mjcf_importer.import_asset(mjcf_path)

        return result

    def setup_grasp_scene(self, gripper_prim: str, environment: str) -> Dict:
        """Set up grasp test scene with target object"""
        from omni.isaac.core import World
        from omni.isaac.core.objects import DynamicCuboid

        world = World()
        world.reset()

        # Add target object to grasp
        target = world.scene.add(
            DynamicCuboid(
                prim_path="/World/target",
                name="grasp_target",
                position=[0, 0, 0.1],
                size=0.05,
                color=[1, 0, 0]
            )
        )

        # Environment-specific physics
        env_config = {
            "dry": {"friction": 0.8, "damping": 0.1},
            "wet": {"friction": 0.3, "damping": 0.2},
            "surgical": {"friction": 0.5, "damping": 0.15}
        }

        config = env_config.get(environment, env_config["dry"])

        return {"world": world, "target": target, "config": config}

    def run_grasp_test(self, scene: Dict, steps: int = 1000) -> Dict[str, float]:
        """Execute grasp simulation and compute metrics"""
        world = scene["world"]
        target = scene["target"]

        # Simulation loop
        contact_frames = 0
        slip_events = 0
        max_force = 0.0

        for step in range(steps):
            world.step(render=not self.headless)

            # TODO: Implement actual contact/force sensing
            # This requires gripper-specific actuation logic

        # Compute metrics
        metrics = {
            "slip_rate": slip_events / max(contact_frames, 1),
            "contact_area": 0.03,  # Would come from contact sensor
            "grasp_force": max_force,
            "grasp_success": slip_events < steps * 0.1,
            "sim_steps": steps,
            "contact_frames": contact_frames
        }

        return metrics

    def export_usd(self, output_path: str):
        """Export current stage to USD"""
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        stage.Export(output_path)


class MockRunner:
    """Mock runner for testing without Isaac Sim"""

    def run_grasp_test(self, mjcf_path: str, environment: str) -> Dict[str, float]:
        """Generate mock metrics based on environment"""
        import random

        # Environment affects expected performance
        env_modifiers = {
            "dry": {"slip_mod": 0.8, "contact_mod": 1.2},
            "wet": {"slip_mod": 1.5, "contact_mod": 0.8},
            "surgical": {"slip_mod": 1.0, "contact_mod": 1.0}
        }

        mod = env_modifiers.get(environment, env_modifiers["dry"])

        base_slip = random.uniform(0.05, 0.2)
        base_contact = random.uniform(0.02, 0.04)

        metrics = {
            "slip_rate": round(base_slip * mod["slip_mod"], 4),
            "contact_area": round(base_contact * mod["contact_mod"], 4),
            "grasp_force": round(random.uniform(1, 5), 2),
            "grasp_success": base_slip < 0.15,
            "sim_steps": 1000,
            "contact_frames": random.randint(500, 900)
        }

        return metrics


def compute_outcome(metrics: Dict, baseline: Dict = None) -> Dict:
    """
    Compute outcome direction and strength from metrics

    Rules:
    - supports: slip_rate improved AND contact_area improved
    - refutes: both worsened
    - ambiguous: mixed results
    """
    if baseline is None:
        baseline = {
            "slip_rate": 0.15,
            "contact_area": 0.025
        }

    slip_better = metrics["slip_rate"] < baseline["slip_rate"]
    contact_better = metrics["contact_area"] > baseline["contact_area"]

    if slip_better and contact_better:
        direction = "supports"
        # Strength based on magnitude of improvement
        slip_improvement = (baseline["slip_rate"] - metrics["slip_rate"]) / baseline["slip_rate"]
        contact_improvement = (metrics["contact_area"] - baseline["contact_area"]) / baseline["contact_area"]
        strength = min(0.95, 0.5 + (slip_improvement + contact_improvement) / 2)
    elif not slip_better and not contact_better:
        direction = "refutes"
        slip_regression = (metrics["slip_rate"] - baseline["slip_rate"]) / baseline["slip_rate"]
        contact_regression = (baseline["contact_area"] - metrics["contact_area"]) / baseline["contact_area"]
        strength = min(0.95, 0.5 + (slip_regression + contact_regression) / 2)
    else:
        direction = "ambiguous"
        strength = 0.3

    strength = round(strength, 3)

    rationale = (
        f"Slip: {metrics['slip_rate']:.3f} (baseline {baseline['slip_rate']}), "
        f"Contact: {metrics['contact_area']:.4f} (baseline {baseline['contact_area']}). "
        f"{'Both improved' if direction == 'supports' else 'Both worsened' if direction == 'refutes' else 'Mixed results'}."
    )

    return {
        "direction": direction,
        "strength": strength,
        "rationale": rationale
    }


def create_run_bundle(
    run_id: str,
    design_id: str,
    hypothesis_id: str,
    experiment_id: str,
    environment: str,
    engine: str,
    metrics: Dict,
    outcome: Dict,
    artifacts: list,
    notes: str = "",
    parent_run_id: Optional[str] = None,
    batch_id: Optional[str] = None,
    schema_version: str = "1.0",
) -> Dict:
    """Create run bundle for Lab OS"""
    return {
        "run_id": run_id,
        "parent_run_id": parent_run_id,
        "batch_id": batch_id,
        "schema_version": schema_version,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "engine": engine,
        "design_id": design_id,
        "hypothesis_id": hypothesis_id,
        "experiment_id": experiment_id,
        "environment": environment,
        "metrics": metrics,
        "outcome": outcome,
        "artifact_manifest": artifacts,
        "notes": notes,
    }


def save_artifacts(run_id: str, bundle: Dict, usd_path: Optional[str] = None) -> Path:
    """Save run artifacts to disk"""
    artifact_dir = ARTIFACTS_BASE / run_id
    artifact_dir.mkdir(parents=True, exist_ok=True)

    # Save bundle
    bundle_path = artifact_dir / "run.json"
    with open(bundle_path, "w") as f:
        json.dump(bundle, f, indent=2)

    # Copy USD if exists
    if usd_path and Path(usd_path).exists():
        import shutil
        shutil.copy(usd_path, artifact_dir / "gripper.usd")

    return artifact_dir


def post_to_lab_os(bundle: Dict) -> Optional[Dict]:
    """POST run bundle to Lab OS"""
    try:
        r = requests.post(f"{LAB_OS_URL}/runs", json=bundle, timeout=10)
        if r.status_code == 200:
            return r.json()
        else:
            print(f"Lab OS error: {r.status_code} - {r.text}")
            return None
    except requests.exceptions.ConnectionError:
        print(f"Lab OS not reachable at {LAB_OS_URL}")
        return None
    except Exception as e:
        print(f"Error posting to Lab OS: {e}")
        return None


def run_experiment(
    mjcf_path: str,
    design_id: str,
    hypothesis_id: str,
    environment: str = "dry",
    parent_run_id: Optional[str] = None,
    use_mock: bool = False,
    headless: bool = True,
    batch_id: Optional[str] = None,
    schema_version: str = "1.0",
    exploration_mode: str = "exploit",
    parameter_vector: Optional[Dict] = None,
) -> Dict:
    """
    Full experiment pipeline:
    1. Load MJCF (or mock)
    2. Run simulation
    3. Compute metrics
    4. Compute outcome
    5. Save artifacts
    6. POST to Lab OS
    """
    run_id = generate_run_id()
    experiment_id = generate_experiment_id(design_id, environment)

    print(f"\n{'='*50}")
    print(f"Experiment: {run_id}")
    print(f"Design: {design_id}")
    print(f"Hypothesis: {hypothesis_id}")
    print(f"Environment: {environment}")
    print(f"{'='*50}\n")

    artifacts = []
    usd_path = None

    if use_mock or not ISAAC_AVAILABLE:
        # Mock mode
        print("Running in MOCK mode")
        engine = "mock"
        runner = MockRunner()
        metrics = runner.run_grasp_test(mjcf_path, environment)
    else:
        # Real Isaac Sim
        print("Running in Isaac Sim")
        engine = "isaac_sim"

        with IsaacRunner(headless=headless) as runner:
            # Load MJCF
            print(f"Loading MJCF: {mjcf_path}")
            prim_path = runner.load_mjcf(mjcf_path)

            # Set up scene
            print(f"Setting up {environment} environment")
            scene = runner.setup_grasp_scene(prim_path, environment)

            # Run simulation
            print("Running grasp test...")
            metrics = runner.run_grasp_test(scene)

            # Export USD
            usd_path = str(ARTIFACTS_BASE / run_id / "gripper.usd")
            Path(usd_path).parent.mkdir(parents=True, exist_ok=True)
            runner.export_usd(usd_path)
            artifacts.append("gripper.usd")
            print(f"Exported USD: {usd_path}")

    # Compute outcome
    outcome = compute_outcome(metrics)

    print(f"\nMetrics:")
    for k, v in metrics.items():
        print(f"  {k}: {v}")
    print(f"\nOutcome: {outcome['direction']} (strength: {outcome['strength']})")
    print(f"Rationale: {outcome['rationale']}")

    # Create bundle
    bundle = create_run_bundle(
        run_id=run_id,
        design_id=design_id,
        hypothesis_id=hypothesis_id,
        experiment_id=experiment_id,
        environment=environment,
        engine=engine,
        metrics=metrics,
        outcome=outcome,
        artifacts=artifacts,
        notes=f"Automated experiment for {design_id}",
        parent_run_id=parent_run_id,
        batch_id=batch_id,
        schema_version=schema_version,
    )

    # Save artifacts
    artifact_dir = save_artifacts(run_id, bundle, usd_path)
    print(f"\nArtifacts saved: {artifact_dir}")

    # POST to Lab OS
    print("\nPosting to Lab OS...")
    result = post_to_lab_os(bundle)

    if result:
        print(f"✓ Run accepted")
        print(f"  New confidence: {result.get('new_confidence', 'N/A')}")

    # Add to discovery surface (non-fatal)
    try:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from app.registry import add_surface_point

        if parameter_vector is None:
            parameter_vector = {"design_id": design_id, "environment": environment}
        slip_rate = metrics.get("slip_rate", 0.5)
        result_score = 1.0 - min(slip_rate, 1.0)
        novelty_flag = outcome["direction"] == "refutes" or outcome["strength"] > 0.9
        add_surface_point(
            run_id=run_id,
            hypothesis_id=hypothesis_id,
            parameter_vector=parameter_vector,
            environment=environment,
            result_score=result_score,
            confidence_delta=0,
            novelty_flag=novelty_flag,
            exploration_mode=exploration_mode,
        )
    except Exception as e:
        print(f"Discovery surface update failed: {e}")

    return bundle


def run_sweep(
    mjcf_path: str,
    design_id: str,
    hypothesis_id: str,
    environments: list = None,
    runs_per_env: int = 3,
    use_mock: bool = False
) -> list:
    """
    Run parameter sweep across environments
    Creates lineage via parent_run_id
    """
    if environments is None:
        environments = ["dry", "wet", "surgical"]

    all_bundles = []
    parent_id = None

    for env in environments:
        for i in range(runs_per_env):
            print(f"\n>>> Sweep run {len(all_bundles)+1}/{len(environments)*runs_per_env}")

            bundle = run_experiment(
                mjcf_path=mjcf_path,
                design_id=design_id,
                hypothesis_id=hypothesis_id,
                environment=env,
                parent_run_id=parent_id,
                use_mock=use_mock
            )

            all_bundles.append(bundle)
            parent_id = bundle["run_id"]  # Chain runs together

    print(f"\n{'='*50}")
    print(f"Sweep complete: {len(all_bundles)} runs")
    print(f"{'='*50}")

    return all_bundles


def main():
    parser = argparse.ArgumentParser(description="Isaac Sim Experiment Runner")
    parser.add_argument("--mjcf", required=True, help="Path to MJCF file")
    parser.add_argument("--design", required=True, help="Design ID")
    parser.add_argument("--hypothesis", required=True, help="Hypothesis ID")
    parser.add_argument("--environment", default="dry", choices=["dry", "wet", "surgical"])
    parser.add_argument("--parent", default=None, help="Parent run ID for lineage")
    parser.add_argument("--mock", action="store_true", help="Use mock runner (no Isaac Sim)")
    parser.add_argument("--sweep", action="store_true", help="Run sweep across all environments")
    parser.add_argument("--runs", type=int, default=3, help="Runs per environment in sweep")
    parser.add_argument("--headless", action="store_true", default=True, help="Run headless")

    args = parser.parse_args()

    if args.sweep:
        run_sweep(
            mjcf_path=args.mjcf,
            design_id=args.design,
            hypothesis_id=args.hypothesis,
            runs_per_env=args.runs,
            use_mock=args.mock
        )
    else:
        run_experiment(
            mjcf_path=args.mjcf,
            design_id=args.design,
            hypothesis_id=args.hypothesis,
            environment=args.environment,
            parent_run_id=args.parent,
            use_mock=args.mock,
            headless=args.headless
        )


if __name__ == "__main__":
    main()
