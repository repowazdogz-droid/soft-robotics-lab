#!/usr/bin/env python3
"""
MuJoCo Validator
================

Validates MJCF gripper files by:
1. Loading into MuJoCo
2. Running basic simulation
3. Testing gripper close action
4. Reporting any errors or warnings
"""

import json
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np


@dataclass
class ValidationResult:
    """Result of validating a single design."""
    design_id: str
    mjcf_path: str
    valid: bool
    loads: bool
    simulates: bool
    errors: List[str]
    warnings: List[str]
    stats: Dict

    def to_dict(self) -> Dict:
        return {
            "design_id": self.design_id,
            "mjcf_path": self.mjcf_path,
            "valid": self.valid,
            "loads": self.loads,
            "simulates": self.simulates,
            "errors": self.errors,
            "warnings": self.warnings,
            "stats": self.stats
        }


class MuJoCoValidator:
    """Validates MJCF gripper files."""

    def __init__(self):
        self.results: List[ValidationResult] = []

    def validate_mjcf(self, mjcf_path: str, design_id: str = None) -> ValidationResult:
        """
        Validate a single MJCF file.

        Args:
            mjcf_path: Path to MJCF file
            design_id: Optional design ID

        Returns:
            ValidationResult
        """
        path = Path(mjcf_path)
        design_id = design_id or path.stem

        errors = []
        warnings = []
        stats = {}
        loads = False
        simulates = False
        model = None

        # Step 1: Try to load the model
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
            loads = True

            stats["bodies"] = model.nbody
            stats["joints"] = model.njnt
            stats["actuators"] = model.nu
            stats["sensors"] = model.nsensor
            stats["timestep"] = float(model.opt.timestep)

        except Exception as e:
            errors.append(f"Failed to load: {str(e)}")
            return ValidationResult(
                design_id=design_id,
                mjcf_path=str(path),
                valid=False,
                loads=False,
                simulates=False,
                errors=errors,
                warnings=warnings,
                stats=stats
            )

        # Step 2: Check for common issues
        if model.nbody < 2:
            warnings.append("Very few bodies - gripper may be too simple")

        if model.nu == 0:
            warnings.append("No actuators defined - gripper cannot move")

        if model.njnt == 0:
            warnings.append("No joints defined - gripper is static")

        # Step 3: Try to simulate
        try:
            data = mujoco.MjData(model)

            # Run a short simulation
            sim_steps = 100
            start_time = time.time()

            for _ in range(sim_steps):
                mujoco.mj_step(model, data)

            sim_time = time.time() - start_time
            simulates = True

            stats["sim_steps"] = sim_steps
            stats["sim_time_ms"] = sim_time * 1000
            stats["steps_per_sec"] = sim_steps / sim_time if sim_time > 0 else 0

            # Check for simulation instability
            if np.any(np.isnan(data.qpos)) or np.any(np.isinf(data.qpos)):
                errors.append("Simulation became unstable (NaN/Inf in positions)")
                simulates = False

            if np.any(np.abs(data.qpos) > 1000):
                warnings.append("Large position values detected - possible instability")

        except Exception as e:
            errors.append(f"Simulation failed: {str(e)}")
            simulates = False

        # Step 4: Test actuator response (if actuators exist)
        if simulates and model.nu > 0:
            try:
                data = mujoco.MjData(model)

                # Apply control signal
                data.ctrl[:] = 1.0  # Activate all actuators

                # Run simulation
                initial_qpos = data.qpos.copy()

                for _ in range(200):
                    mujoco.mj_step(model, data)

                final_qpos = data.qpos.copy()

                # Check if anything moved
                position_change = np.linalg.norm(final_qpos - initial_qpos)
                stats["position_change"] = float(position_change)

                if position_change < 0.001:
                    warnings.append("Actuators had no effect - check actuator setup")
                else:
                    stats["actuators_responsive"] = True

            except Exception as e:
                warnings.append(f"Actuator test failed: {str(e)}")

        # Determine overall validity
        valid = loads and simulates and len(errors) == 0

        result = ValidationResult(
            design_id=design_id,
            mjcf_path=str(path),
            valid=valid,
            loads=loads,
            simulates=simulates,
            errors=errors,
            warnings=warnings,
            stats=stats
        )

        self.results.append(result)
        return result

    def validate_directory(self, directory: str, pattern: str = "*.mjcf") -> List[ValidationResult]:
        """
        Validate all MJCF files in a directory.
        """
        path = Path(directory)
        results = []

        for mjcf_file in sorted(path.rglob(pattern)):
            result = self.validate_mjcf(str(mjcf_file))
            results.append(result)

        return results

    def generate_report(self) -> Dict:
        """Generate summary report of all validations."""
        total = len(self.results)
        valid = sum(1 for r in self.results if r.valid)
        loads = sum(1 for r in self.results if r.loads)
        simulates = sum(1 for r in self.results if r.simulates)

        total_errors = sum(len(r.errors) for r in self.results)
        total_warnings = sum(len(r.warnings) for r in self.results)

        return {
            "summary": {
                "total": total,
                "valid": valid,
                "valid_pct": valid / total * 100 if total > 0 else 0,
                "loads": loads,
                "simulates": simulates,
                "total_errors": total_errors,
                "total_warnings": total_warnings
            },
            "results": [r.to_dict() for r in self.results]
        }

    def print_report(self):
        """Print human-readable report."""
        report = self.generate_report()
        summary = report["summary"]

        print("\n" + "=" * 60)
        print("  MUJOCO VALIDATION REPORT")
        print("=" * 60)
        print(f"\n  Total designs:  {summary['total']}")
        print(f"  Valid:          {summary['valid']} ({summary['valid_pct']:.0f}%)")
        print(f"  Loads:          {summary['loads']}")
        print(f"  Simulates:      {summary['simulates']}")
        print(f"  Errors:         {summary['total_errors']}")
        print(f"  Warnings:       {summary['total_warnings']}")

        # Details for failures
        failures = [r for r in self.results if not r.valid]
        if failures:
            print("\n" + "-" * 60)
            print("  FAILURES:")
            print("-" * 60)
            for r in failures:
                print(f"\n  ❌ {r.design_id}")
                for e in r.errors:
                    print(f"     Error: {e}")
                for w in r.warnings:
                    print(f"     Warning: {w}")

        # Warnings for valid designs
        warned = [r for r in self.results if r.valid and r.warnings]
        if warned:
            print("\n" + "-" * 60)
            print("  WARNINGS (valid designs):")
            print("-" * 60)
            for r in warned:
                print(f"\n  ⚠️  {r.design_id}")
                for w in r.warnings:
                    print(f"     {w}")

        print("\n" + "=" * 60)


def validate_gripper_zoo(zoo_path: str) -> Dict:
    """
    Validate all grippers in the zoo.

    Args:
        zoo_path: Path to gripper_zoo/designs directory

    Returns:
        Validation report
    """
    validator = MuJoCoValidator()

    zoo_dir = Path(zoo_path)

    for design_dir in sorted(zoo_dir.iterdir()):
        if design_dir.is_dir():
            mjcf_files = list(design_dir.glob("*.mjcf"))
            for mjcf_file in mjcf_files:
                validator.validate_mjcf(str(mjcf_file), design_dir.name)

    return validator.generate_report()


# ════════════════════════════════════════════════════════════════════════════════
# CLI
# ════════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Validate MuJoCo MJCF files")
    parser.add_argument("path", nargs="?", help="MJCF file or directory")
    parser.add_argument("--zoo", action="store_true", help="Validate gripper zoo")
    parser.add_argument("--output", "-o", help="Output JSON report path")

    args = parser.parse_args()

    validator = MuJoCoValidator()

    if args.zoo:
        # Validate gripper zoo
        zoo_path = Path(__file__).resolve().parent.parent / "gripper_zoo" / "designs"
        if zoo_path.exists():
            print(f"Validating gripper zoo at {zoo_path}...")
            validator.validate_directory(str(zoo_path))
        else:
            print(f"Gripper zoo not found at {zoo_path}")
            exit(1)

    elif args.path:
        path = Path(args.path)
        if path.is_file():
            print(f"Validating {path}...")
            result = validator.validate_mjcf(str(path))
            print(f"  Valid: {result.valid}")
            print(f"  Errors: {result.errors}")
            print(f"  Warnings: {result.warnings}")
        elif path.is_dir():
            print(f"Validating all MJCF files in {path}...")
            validator.validate_directory(str(path))
        else:
            print(f"Path not found: {path}")
            exit(1)

    else:
        parser.print_help()
        exit(0)

    # Print report
    validator.print_report()

    # Save JSON report if requested
    if args.output:
        report = validator.generate_report()
        with open(args.output, "w") as f:
            json.dump(report, f, indent=2)
        print(f"\n✓ Report saved to {args.output}")
