#!/usr/bin/env python3
"""
Failure Mode Predictor
======================

Analyzes a gripper design and predicts potential failure modes.

Usage:
    python failure_predictor.py design.json
    python failure_predictor.py design.json --environment wet --task egg_handling
"""

import sys
import json
from pathlib import Path
from typing import List, Dict
from dataclasses import dataclass


@dataclass
class FailureMode:
    """A predicted failure mode."""
    mode: str
    probability: float
    severity: str  # low, medium, high, critical
    cause: str
    mitigation: str
    detection: str


class FailurePredictor:
    """Predicts failure modes for gripper designs."""

    def __init__(self):
        self.failure_database = {
            "slip": {
                "mode": "Object slip during grasp",
                "base_probability": 0.2,
                "severity": "medium",
                "cause": "Insufficient friction or normal force",
                "detection": "Force sensors, tactile feedback",
                "factors": {
                    "wet": 1.5,
                    "smooth_object": 1.3,
                    "high_compliance": 0.8,
                    "low_force": 1.4,
                },
            },
            "crush": {
                "mode": "Object damage from excessive force",
                "base_probability": 0.15,
                "severity": "high",
                "cause": "Poor force control or compliance mismatch",
                "detection": "Force limiting, compliance sensing",
                "factors": {
                    "high_compliance": 1.6,
                    "pneumatic": 1.2,
                    "no_force_feedback": 1.5,
                },
            },
            "tendon_failure": {
                "mode": "Tendon fatigue or breakage",
                "base_probability": 0.08,
                "severity": "high",
                "cause": "Cyclic loading, sharp routing",
                "detection": "Tension monitoring, visual inspection",
                "factors": {
                    "tendon": 1.0,
                    "high_cycles": 1.5,
                    "sharp_angles": 1.8,
                },
            },
            "seal_leak": {
                "mode": "Pneumatic seal failure",
                "base_probability": 0.12,
                "severity": "medium",
                "cause": "Material degradation, pressure cycling",
                "detection": "Pressure monitoring",
                "factors": {
                    "pneumatic": 1.0,
                    "high_pressure": 1.4,
                    "temperature_cycling": 1.3,
                },
            },
            "material_tear": {
                "mode": "Silicone tear at stress concentration",
                "base_probability": 0.1,
                "severity": "high",
                "cause": "Sharp corners, excessive strain",
                "detection": "Visual inspection, strain monitoring",
                "factors": {
                    "silicone_soft": 1.3,
                    "sharp_geometry": 1.6,
                    "high_strain": 1.5,
                },
            },
            "slow_response": {
                "mode": "Insufficient grasp speed",
                "base_probability": 0.25,
                "severity": "low",
                "cause": "Pneumatic lag, large chamber volume",
                "detection": "Timing measurements",
                "factors": {
                    "pneumatic": 1.4,
                    "large_fingers": 1.3,
                    "long_tubing": 1.5,
                },
            },
            "interference": {
                "mode": "Finger-finger collision",
                "base_probability": 0.1,
                "severity": "medium",
                "cause": "Poor trajectory planning, object position error",
                "detection": "Contact sensors, position feedback",
                "factors": {
                    "many_fingers": 1.4,
                    "small_aperture": 1.3,
                    "no_feedback": 1.5,
                },
            },
        }

    def predict(
        self,
        design: Dict,
        environment: str = "dry",
        task: str = "general",
    ) -> List[FailureMode]:
        """Predict failure modes for a design."""
        failures = []

        actuator = design.get("primary_actuator", "tendon")
        num_fingers = design.get("num_fingers", 3)
        max_force = design.get("max_force_n", 10)

        fingers = design.get("finger_designs", [{}])
        material = (
            fingers[0].get("material", "silicone_medium") if fingers else "silicone_medium"
        )

        for name, fm_template in self.failure_database.items():
            prob = fm_template["base_probability"]

            factors = fm_template["factors"]

            if environment == "wet" and "wet" in factors:
                prob *= factors["wet"]

            if actuator in factors:
                prob *= factors[actuator]

            if material in factors:
                prob *= factors[material]

            if num_fingers >= 4 and "many_fingers" in factors:
                prob *= factors["many_fingers"]

            if max_force < 3 and "low_force" in factors:
                prob *= factors["low_force"]

            if prob < 0.05:
                continue

            mitigation = self._get_mitigation(name, design, environment)

            failures.append(
                FailureMode(
                    mode=fm_template["mode"],
                    probability=min(0.95, prob),
                    severity=fm_template["severity"],
                    cause=fm_template["cause"],
                    mitigation=mitigation,
                    detection=fm_template["detection"],
                )
            )

        failures.sort(key=lambda f: f.probability, reverse=True)

        return failures

    def _get_mitigation(
        self, failure_type: str, design: Dict, environment: str
    ) -> str:
        """Get specific mitigation for failure type."""
        mitigations = {
            "slip": "Add textured fingertips, increase grip force, use gecko-adhesion surfaces",
            "crush": "Implement force feedback, use softer materials, add compliance layer",
            "tendon_failure": "Use braided Dyneema, add strain relief, limit max tension",
            "seal_leak": "Use redundant seals, add pressure monitoring, reduce operating pressure",
            "material_tear": "Fillet sharp corners, use reinforced silicone, limit strain to 50%",
            "slow_response": "Use smaller chambers, increase flow rate, pre-pressurize",
            "interference": "Add collision detection, improve path planning, increase finger spacing",
        }

        base = mitigations.get(failure_type, "Review design parameters")

        if environment == "wet" and failure_type == "slip":
            base += "; Consider hydrophilic coating or suction elements"

        if environment == "surgical":
            base += "; Ensure all mitigations are biocompatible"

        return base

    def report(self, design: Dict, failures: List[FailureMode]) -> str:
        """Generate failure analysis report."""
        report = f"""
╔══════════════════════════════════════════════════════════════════╗
║  FAILURE MODE ANALYSIS REPORT
╠══════════════════════════════════════════════════════════════════╣
║  Design: {design.get('name', 'Unknown')}
║  Analyzed: {len(failures)} potential failure modes
║
"""

        critical = [f for f in failures if f.severity == "critical"]
        high = [f for f in failures if f.severity == "high"]
        medium = [f for f in failures if f.severity == "medium"]
        low = [f for f in failures if f.severity == "low"]

        if critical:
            report += "║  ⛔ CRITICAL FAILURES:\n"
            for f in critical:
                report += f"║    • {f.mode} ({f.probability:.0%})\n"
                report += f"║      Cause: {f.cause}\n"
                report += f"║      Mitigation: {f.mitigation}\n"

        if high:
            report += "\n║  🔴 HIGH SEVERITY:\n"
            for f in high:
                report += f"║    • {f.mode} ({f.probability:.0%})\n"
                report += f"║      Mitigation: {f.mitigation}\n"

        if medium:
            report += "\n║  🟡 MEDIUM SEVERITY:\n"
            for f in medium:
                report += f"║    • {f.mode} ({f.probability:.0%})\n"

        if low:
            report += "\n║  🟢 LOW SEVERITY:\n"
            for f in low:
                report += f"║    • {f.mode} ({f.probability:.0%})\n"

        report += """║
╠══════════════════════════════════════════════════════════════════╣
║  RECOMMENDATIONS:
║    1. Address critical and high-severity modes before prototyping
║    2. Add detection methods for top 3 failure modes
║    3. Test mitigations in simulation before fabrication
╚══════════════════════════════════════════════════════════════════╝
"""

        return report


def _resolve_design_path(path_str: str) -> Path:
    """Resolve design path: file path or directory (pick latest gd_*.json)."""
    path = Path(path_str)
    if path.is_file():
        return path
    if path.is_dir():
        # Design JSONs only (exclude *_usd.json)
        candidates = [
            p for p in path.glob("gd_*.json")
            if not p.name.endswith("_usd.json")
        ]
        if not candidates:
            raise FileNotFoundError(
                f"No design JSON (gd_*.json) found in directory: {path}"
            )
        # Latest by name (gd_YYYYMMDD_NNNN)
        candidates.sort(key=lambda p: p.name, reverse=True)
        return candidates[0]
    raise FileNotFoundError(f"Not a file or directory: {path}")


# CLI
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Failure Mode Predictor")
    parser.add_argument(
        "design_file",
        help="Path to design JSON file or output directory (uses latest gd_*.json)",
    )
    parser.add_argument("--environment", "-e", default="dry", help="Environment")
    parser.add_argument("--task", "-t", default="general", help="Task type")

    args = parser.parse_args()

    design_path = _resolve_design_path(args.design_file)
    design = json.loads(design_path.read_text())

    predictor = FailurePredictor()
    failures = predictor.predict(design, args.environment, args.task)

    print(predictor.report(design, failures))
