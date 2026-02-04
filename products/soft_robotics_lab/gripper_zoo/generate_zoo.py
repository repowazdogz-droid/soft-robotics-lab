#!/usr/bin/env python3
"""
Gripper Zoo Generator
=====================

Generates 50 diverse gripper designs covering different:
- Gestures (pinch, power, wrap, hook, lateral, squeeze, spread)
- Environments (dry, wet, surgical)
- Object compliance levels (rigid, medium, soft)
- Force requirements (low, medium, high)

Run: python generate_zoo.py
"""

import sys
import json
from pathlib import Path

_soft_lab_root = Path(__file__).resolve().parent.parent
if str(_soft_lab_root) not in sys.path:
    sys.path.insert(0, str(_soft_lab_root))

from workbench.motion_to_morphology import MotionToMorphology, export_design


def generate_zoo():
    """Generate 50 diverse gripper designs."""
    m2m = MotionToMorphology()
    output_dir = Path(__file__).resolve().parent / "designs"
    output_dir.mkdir(exist_ok=True)

    gestures = ["pinch", "power", "wrap", "hook", "lateral", "squeeze", "spread"]
    environments = ["dry", "wet", "surgical"]
    compliances = [0.2, 0.5, 0.8]
    designs = []
    design_count = 0
    target_matrix = 45  # Leave room for 5 special designs

    for gesture in gestures:
        for env in environments:
            for compliance in compliances:
                if design_count >= target_matrix:
                    break

                if gesture in ["pinch", "lateral"]:
                    aperture = 0.03
                    force = 3.0
                elif gesture in ["power", "squeeze"]:
                    aperture = 0.05
                    force = 10.0
                else:
                    aperture = 0.06
                    force = 5.0

                if compliance > 0.6:
                    force = min(force, 5.0)

                try:
                    design = m2m.from_gesture(
                        gesture,
                        aperture=aperture,
                        force_requirement=force,
                        object_compliance=compliance,
                        environment=env,
                    )

                    compliance_name = (
                        "rigid"
                        if compliance < 0.4
                        else "medium"
                        if compliance < 0.7
                        else "soft"
                    )
                    design.name = (
                        f"{gesture.title()} Gripper - {env.title()} - {compliance_name.title()} Objects"
                    )
                    design.tags.append(compliance_name)

                    design_dir = output_dir / design.id.lower().replace("-", "_")
                    design_dir.mkdir(exist_ok=True)
                    exports = export_design(design, str(design_dir))

                    base = design.id.lower().replace("-", "_")
                    designs.append({
                        "id": design.id,
                        "name": design.name,
                        "gesture": gesture,
                        "environment": env,
                        "compliance": compliance_name,
                        "confidence": design.confidence,
                        "num_fingers": design.num_fingers,
                        "actuator": design.primary_actuator.value,
                        "max_force_n": design.max_force_n,
                        "exports": {
                            "json": f"{base}/{base}.json",
                            "mjcf": f"{base}/{base}.mjcf",
                            "urdf": f"{base}/{base}.urdf",
                            "usd": f"{base}/{base}_usd.json",
                        },
                    })

                    design_count += 1
                    print(f"[{design_count:02d}/50] {design.name}")

                except Exception as e:
                    print(f"Error generating {gesture}/{env}/{compliance}: {e}")

            if design_count >= target_matrix:
                break
        if design_count >= target_matrix:
            break

    special_designs = [
        ("pinch", 0.02, 2.0, 0.9, "surgical", "Surgical Micro-Gripper"),
        ("wrap", 0.10, 5.0, 0.7, "wet", "Kelp Harvester"),
        ("power", 0.08, 20.0, 0.3, "dry", "Industrial Pick-and-Place"),
        ("squeeze", 0.04, 3.0, 0.95, "dry", "Egg Handler"),
        ("hook", 0.06, 8.0, 0.5, "wet", "Underwater Retrieval"),
    ]

    for gesture, aperture, force, compliance, env, name in special_designs:
        if design_count >= 50:
            break

        try:
            design = m2m.from_gesture(
                gesture,
                aperture=aperture,
                force_requirement=force,
                object_compliance=compliance,
                environment=env,
            )
            design.name = name

            design_dir = output_dir / design.id.lower().replace("-", "_")
            design_dir.mkdir(exist_ok=True)
            exports = export_design(design, str(design_dir))

            compliance_name = (
                "rigid"
                if compliance < 0.4
                else "medium"
                if compliance < 0.7
                else "soft"
            )
            base = design.id.lower().replace("-", "_")
            designs.append({
                "id": design.id,
                "name": design.name,
                "gesture": gesture,
                "environment": env,
                "compliance": compliance_name,
                "confidence": design.confidence,
                "num_fingers": design.num_fingers,
                "actuator": design.primary_actuator.value,
                "max_force_n": design.max_force_n,
                "exports": {
                    "json": f"{base}/{base}.json",
                    "mjcf": f"{base}/{base}.mjcf",
                    "urdf": f"{base}/{base}.urdf",
                    "usd": f"{base}/{base}_usd.json",
                },
            })

            design_count += 1
            print(f"[{design_count:02d}/50] {name}")

        except Exception as e:
            print(f"Error generating {name}: {e}")

    index_path = output_dir / "index.json"
    index_path.write_text(
        json.dumps(
            {
                "generated": str(Path(__file__).name),
                "count": len(designs),
                "designs": designs,
            },
            indent=2,
        )
    )

    print(f"\nGenerated {len(designs)} gripper designs")
    print(f"Index saved to: {index_path}")

    return designs


if __name__ == "__main__":
    generate_zoo()
