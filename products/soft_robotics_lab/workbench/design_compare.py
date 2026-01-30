#!/usr/bin/env python3
"""
Design Compare Tool
===================

Side-by-side comparison of two gripper designs.

Usage:
    python design_compare.py design1.json design2.json
    python design_compare.py design1.json design2.json --output comparison.md
"""

import sys
import json
import argparse
from pathlib import Path
from typing import Dict, List


def load_design(path: str) -> Dict:
    """Load a design from JSON."""
    p = Path(path)
    if p.is_dir():
        # Find first *.json that is not *_usd.json
        candidates = [f for f in p.glob("*.json") if not f.name.endswith("_usd.json")]
        if not candidates:
            raise FileNotFoundError(f"No design JSON found in directory: {path}")
        p = sorted(candidates, key=lambda x: x.name)[0]
    return json.loads(p.read_text())


def compare(design1: Dict, design2: Dict) -> str:
    """Generate comparison report."""
    name1 = design1.get("name", "Design 1")
    name2 = design2.get("name", "Design 2")

    report = f"""
╔══════════════════════════════════════════════════════════════════════════════╗
║  DESIGN COMPARISON
╠══════════════════════════════════════════════════════════════════════════════╣
║  {name1[:35]:<35} vs {name2[:35]:<35}
╠══════════════════════════════════════════════════════════════════════════════╣

"""

    report += "## Overview\n\n"
    report += f"| Property | {name1[:20]} | {name2[:20]} | Difference |\n"
    report += f"|----------|{'-'*20}|{'-'*20}|------------|\n"

    comparisons = [
        ("Gesture", "source_gesture", None),
        ("Fingers", "num_fingers", lambda a, b: f"{'+' if b > a else ''}{b - a}"),
        ("Confidence", "confidence", lambda a, b: f"{(b-a)*100:+.0f}%"),
        ("Max Aperture (mm)", "max_aperture_mm", lambda a, b: f"{b-a:+.1f}"),
        ("Max Force (N)", "max_force_n", lambda a, b: f"{b-a:+.1f}"),
        ("Speed (mm/s)", "grasp_speed_mm_per_s", lambda a, b: f"{b-a:+.1f}"),
        ("Actuator", "primary_actuator", None),
    ]

    for label, key, diff_fn in comparisons:
        val1 = design1.get(key, "N/A")
        val2 = design2.get(key, "N/A")

        if key == "confidence" and isinstance(val1, (int, float)) and isinstance(val2, (int, float)):
            val1_str = f"{val1*100:.0f}%"
            val2_str = f"{val2*100:.0f}%"
        elif isinstance(val1, (int, float)) and isinstance(val2, (int, float)):
            val1_str = f"{val1:.1f}"
            val2_str = f"{val2:.1f}"
        else:
            val1_str = str(val1)
            val2_str = str(val2)

        if diff_fn and isinstance(val1, (int, float)) and isinstance(val2, (int, float)):
            diff = diff_fn(val1, val2)
        elif val1 == val2:
            diff = "="
        else:
            diff = "≠"

        report += f"| {label} | {val1_str} | {val2_str} | {diff} |\n"

    report += "\n## Finger Configuration\n\n"

    f1 = design1.get("finger_designs", [{}])[0] if design1.get("finger_designs") else {}
    f2 = design2.get("finger_designs", [{}])[0] if design2.get("finger_designs") else {}

    report += f"| Property | {name1[:20]} | {name2[:20]} |\n"
    report += f"|----------|{'-'*20}|{'-'*20}|\n"
    report += f"| Length | {f1.get('length_mm', 'N/A')} mm | {f2.get('length_mm', 'N/A')} mm |\n"
    report += f"| Segments | {f1.get('num_segments', 'N/A')} | {f2.get('num_segments', 'N/A')} |\n"
    report += f"| Material | {f1.get('material', 'N/A')} | {f2.get('material', 'N/A')} |\n"
    report += f"| Taper | {f1.get('taper_ratio', 'N/A')} | {f2.get('taper_ratio', 'N/A')} |\n"

    report += "\n## Failure Modes\n\n"

    fm1 = design1.get("failure_modes", [])
    fm2 = design2.get("failure_modes", [])

    report += f"### {name1}\n"
    for fm in fm1[:5]:
        prob = fm.get("probability", 0)
        pct = f"{prob*100:.0f}%" if isinstance(prob, (int, float)) else str(prob)
        report += f"- {fm.get('mode', 'Unknown')}: {pct}\n"

    report += f"\n### {name2}\n"
    for fm in fm2[:5]:
        prob = fm.get("probability", 0)
        pct = f"{prob*100:.0f}%" if isinstance(prob, (int, float)) else str(prob)
        report += f"- {fm.get('mode', 'Unknown')}: {pct}\n"

    report += "\n## Uncertainty Analysis\n\n"

    unc1 = design1.get("uncertainty_factors", [])
    unc2 = design2.get("uncertainty_factors", [])

    report += f"### {name1}\n"
    for u in unc1:
        report += f"- {u}\n"

    report += f"\n### {name2}\n"
    for u in unc2:
        report += f"- {u}\n"

    report += "\n## Recommendation\n\n"

    conf1 = design1.get("confidence", 0) or 0
    conf2 = design2.get("confidence", 0) or 0

    if conf1 > conf2 + 0.1:
        report += f"**{name1}** has higher confidence ({conf1*100:.0f}% vs {conf2*100:.0f}%).\n"
    elif conf2 > conf1 + 0.1:
        report += f"**{name2}** has higher confidence ({conf2*100:.0f}% vs {conf1*100:.0f}%).\n"
    else:
        report += f"Both designs have similar confidence levels (~{(conf1+conf2)/2*100:.0f}%).\n"

    report += "\n### When to use each:\n\n"

    if design1.get("max_force_n", 0) > design2.get("max_force_n", 0):
        report += f"- **{name1}**: Higher force tasks, heavier objects\n"
        report += f"- **{name2}**: Delicate objects, precision tasks\n"
    else:
        report += f"- **{name1}**: Delicate objects, precision tasks\n"
        report += f"- **{name2}**: Higher force tasks, heavier objects\n"

    report += """
---
*Comparison generated by OMEGA Soft Robotics Workbench*
"""

    return report


def main():
    parser = argparse.ArgumentParser(description="Compare two gripper designs")
    parser.add_argument("design1", help="Path to first design JSON (or design directory)")
    parser.add_argument("design2", help="Path to second design JSON (or design directory)")
    parser.add_argument("--output", "-o", help="Output file (markdown)")

    args = parser.parse_args()

    d1 = load_design(args.design1)
    d2 = load_design(args.design2)

    report = compare(d1, d2)

    if args.output:
        Path(args.output).write_text(report)
        print(f"Comparison saved to: {args.output}")
    else:
        print(report)


if __name__ == "__main__":
    main()
