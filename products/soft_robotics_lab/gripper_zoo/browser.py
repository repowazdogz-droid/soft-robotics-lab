#!/usr/bin/env python3
"""
Gripper Zoo Browser
===================

Browse and filter pre-generated gripper designs.

Usage:
    python browser.py                     # List all
    python browser.py --gesture pinch     # Filter by gesture
    python browser.py --env surgical      # Filter by environment
    python browser.py --compliance soft   # Filter by object compliance
    python browser.py --show GD-20260128-0001  # Show details
"""

import sys
import json
import argparse
from pathlib import Path


def load_index():
    """Load the gripper zoo index."""
    index_path = Path(__file__).resolve().parent / "designs" / "index.json"
    if not index_path.exists():
        print("Error: Gripper zoo not generated. Run: python generate_zoo.py")
        sys.exit(1)
    return json.loads(index_path.read_text())


def list_designs(
    designs,
    gesture=None,
    env=None,
    compliance=None,
    actuator=None,
):
    """List designs with optional filters."""
    filtered = designs

    if gesture:
        filtered = [d for d in filtered if d["gesture"] == gesture]
    if env:
        filtered = [d for d in filtered if d["environment"] == env]
    if compliance:
        filtered = [d for d in filtered if d["compliance"] == compliance]
    if actuator:
        filtered = [d for d in filtered if d["actuator"] == actuator]

    print(f"\n{'ID':<20} {'Name':<40} {'Fingers':>7} {'Force':>6} {'Conf':>5}")
    print("-" * 85)

    for d in filtered:
        conf_pct = f"{d['confidence']:.0%}" if isinstance(d["confidence"], float) else str(d["confidence"])
        print(
            f"{d['id']:<20} {d['name'][:40]:<40} {d['num_fingers']:>7} "
            f"{d['max_force_n']:>5.1f}N {conf_pct:>5}"
        )

    print(f"\nTotal: {len(filtered)} designs")


def show_design(designs, design_id):
    """Show detailed info for a design."""
    design = None
    for d in designs:
        if d["id"] == design_id:
            design = d
            break

    if not design:
        print(f"Design {design_id} not found")
        return

    conf_str = f"{design['confidence']:.0%}" if isinstance(design["confidence"], float) else str(design["confidence"])
    print(f"""
╔══════════════════════════════════════════════════════════════════╗
║  {design['name']}
╠══════════════════════════════════════════════════════════════════╣
║  ID: {design['id']}
║  Gesture: {design['gesture']}
║  Environment: {design['environment']}
║  Object Compliance: {design['compliance']}
║
║  Configuration:
║    Fingers: {design['num_fingers']}
║    Actuator: {design['actuator']}
║    Max Force: {design['max_force_n']:.1f}N
║    Confidence: {conf_str}
║
║  Export Files:
║    JSON: {design['exports'].get('json', 'N/A')}
║    MJCF: {design['exports'].get('mjcf', 'N/A')}
║    URDF: {design['exports'].get('urdf', 'N/A')}
║    USD:  {design['exports'].get('usd', 'N/A')}
╚══════════════════════════════════════════════════════════════════╝
""")


def print_stats(designs):
    """Print zoo statistics."""
    print("\n📊 Gripper Zoo Statistics")
    print("=" * 40)

    print("\nBy Gesture:")
    gestures = {}
    for d in designs:
        g = d["gesture"]
        gestures[g] = gestures.get(g, 0) + 1
    for g, count in sorted(gestures.items()):
        print(f"  {g}: {count}")

    print("\nBy Environment:")
    envs = {}
    for d in designs:
        e = d["environment"]
        envs[e] = envs.get(e, 0) + 1
    for e, count in sorted(envs.items()):
        print(f"  {e}: {count}")

    print("\nBy Actuator:")
    actuators = {}
    for d in designs:
        a = d["actuator"]
        actuators[a] = actuators.get(a, 0) + 1
    for a, count in sorted(actuators.items()):
        print(f"  {a}: {count}")

    print("\nConfidence Distribution:")
    low = sum(1 for d in designs if d["confidence"] < 0.5)
    med = sum(1 for d in designs if 0.5 <= d["confidence"] < 0.75)
    high = sum(1 for d in designs if d["confidence"] >= 0.75)
    print(f"  Low (<50%): {low}")
    print(f"  Medium (50-75%): {med}")
    print(f"  High (>75%): {high}")


def main():
    parser = argparse.ArgumentParser(description="Gripper Zoo Browser")
    parser.add_argument("--gesture", "-g", help="Filter by gesture type")
    parser.add_argument("--env", "-e", help="Filter by environment")
    parser.add_argument(
        "--compliance", "-c", help="Filter by compliance (rigid/medium/soft)"
    )
    parser.add_argument("--actuator", "-a", help="Filter by actuator type")
    parser.add_argument("--show", "-s", help="Show details for specific design ID")
    parser.add_argument("--stats", action="store_true", help="Show statistics")

    args = parser.parse_args()

    index = load_index()
    designs = index["designs"]

    print(f"🦾 Gripper Zoo - {index['count']} designs available")

    if args.stats:
        print_stats(designs)
    elif args.show:
        show_design(designs, args.show)
    else:
        list_designs(
            designs,
            args.gesture,
            args.env,
            args.compliance,
            args.actuator,
        )


if __name__ == "__main__":
    main()
