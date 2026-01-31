"""
Prescriptive Fixer - Not just "this failed" but "change X to Y"

Given a validation failure, suggest specific parameter changes.
"""

from dataclasses import dataclass
from typing import List, Dict, Optional, Any
from enum import Enum


class FixType(Enum):
    PARAMETER_CHANGE = "parameter_change"
    GEOMETRY_CHANGE = "geometry_change"
    MATERIAL_CHANGE = "material_change"
    ACTUATOR_CHANGE = "actuator_change"
    STRUCTURAL_CHANGE = "structural_change"


@dataclass
class PrescriptiveFix:
    fix_type: FixType
    component: str
    parameter: str
    current_value: Any
    suggested_value: Any
    unit: str
    confidence: float
    reasoning: str
    priority: int


@dataclass
class FixReport:
    design_id: str
    failure_codes: List[str]
    fixes: List[PrescriptiveFix]
    estimated_improvement: float
    warnings: List[str]


# Map Reality Bridge test names to FIX_RULES keys
_TEST_TO_FAILURE_CODE = {
    "STABILITY_TEST": "STABILITY_FAIL",
    "KINEMATICS_TEST": "KINEMATICS_FAIL",
    "DYNAMICS_TEST": "DYNAMICS_FAIL",
    "SELF_COLLISION_TEST": "SELF_COLLISION",
    "MASS_PROPERTIES_TEST": "MASS_FAIL",
    "LOAD_TEST": "GENERIC",
}

# Fix rules based on failure patterns
FIX_RULES = {
    "STABILITY_FAIL": [
        {
            "condition": lambda d: d.get("tip_over") or "tip" in str(d).lower() or "fall" in str(d).lower(),
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.GEOMETRY_CHANGE,
                component="base",
                parameter="width",
                current_value=d.get("base_width", 0),
                suggested_value=(d.get("base_width") or 0) * 1.3,
                unit="m",
                confidence=0.8,
                reasoning="Wider base improves stability against tip-over",
                priority=1,
            ),
        },
        {
            "condition": lambda d: d.get("center_of_mass_high") or "position" in str(d).lower(),
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.GEOMETRY_CHANGE,
                component="palm",
                parameter="height",
                current_value=d.get("palm_height", 0),
                suggested_value=(d.get("palm_height") or 0) * 0.8,
                unit="m",
                confidence=0.7,
                reasoning="Lower palm reduces center of mass height",
                priority=2,
            ),
        },
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.PARAMETER_CHANGE,
                component="simulation",
                parameter="damping",
                current_value=d.get("current_damping", "unknown"),
                suggested_value="increase joint damping",
                unit="",
                confidence=0.7,
                reasoning="Add or increase joint damping to reduce oscillations",
                priority=2,
            ),
        },
    ],
    "KINEMATICS_FAIL": [
        {
            "condition": lambda d: d.get("limited_range") or "nan" in str(d).lower(),
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.PARAMETER_CHANGE,
                component=d.get("joint", "joint"),
                parameter="range",
                current_value=d.get("current_range", 0),
                suggested_value=(d.get("current_range") or 0) * 1.5,
                unit="rad",
                confidence=0.85,
                reasoning="Increased joint range allows fuller motion",
                priority=1,
            ),
        },
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.GEOMETRY_CHANGE,
                component="linkage",
                parameter="length_ratio",
                current_value=d.get("length_ratio", 1.0),
                suggested_value=0.7,
                unit="ratio",
                confidence=0.6,
                reasoning="Adjusted linkage ratio avoids singularity",
                priority=2,
            ),
        },
    ],
    "DYNAMICS_FAIL": [
        {
            "condition": lambda d: d.get("oscillation") or "acceleration" in str(d).lower(),
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.PARAMETER_CHANGE,
                component=d.get("joint", "actuator"),
                parameter="damping",
                current_value=d.get("current_damping", 0),
                suggested_value=(d.get("current_damping") or 0) * 2.0,
                unit="N·m·s/rad",
                confidence=0.9,
                reasoning="Increased damping reduces oscillation",
                priority=1,
            ),
        },
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.PARAMETER_CHANGE,
                component=d.get("actuator", "actuator"),
                parameter="gain",
                current_value=d.get("current_gain", 1.0),
                suggested_value=(d.get("current_gain") or 1.0) * 1.5,
                unit="",
                confidence=0.75,
                reasoning="Higher gain improves response time",
                priority=2,
            ),
        },
    ],
    "SELF_COLLISION": [
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.GEOMETRY_CHANGE,
                component=d.get("geom_a", "finger"),
                parameter="size",
                current_value=d.get("size_a", 0),
                suggested_value=(d.get("size_a") or 0) * 0.9 if d.get("size_a") else "reduce",
                unit="m",
                confidence=0.7,
                reasoning="Reduced geometry size prevents collision",
                priority=1,
            ),
        },
        {
            "condition": lambda d: d.get("collision_count", 0) > 3,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.GEOMETRY_CHANGE,
                component="finger_spacing",
                parameter="spread_angle",
                current_value=d.get("spread_angle", 0),
                suggested_value=(d.get("spread_angle") or 0) + 15,
                unit="deg",
                confidence=0.8,
                reasoning="Wider finger spread reduces collision risk",
                priority=1,
            ),
        },
    ],
    "MASS_FAIL": [
        {
            "condition": lambda d: d.get("too_heavy") or d.get("mass_kg", 0) > 10,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.MATERIAL_CHANGE,
                component="body",
                parameter="density",
                current_value=d.get("density", 1000),
                suggested_value=(d.get("density") or 1000) * 0.7,
                unit="kg/m³",
                confidence=0.8,
                reasoning="Lower density material reduces mass",
                priority=1,
            ),
        },
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.GEOMETRY_CHANGE,
                component="body",
                parameter="symmetry",
                current_value="asymmetric",
                suggested_value="symmetric",
                unit="",
                confidence=0.6,
                reasoning="Symmetric design improves inertia distribution",
                priority=2,
            ),
        },
    ],
    "GRIP_FORCE_LOW": [
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.ACTUATOR_CHANGE,
                component="finger_actuator",
                parameter="max_force",
                current_value=d.get("current_force", 0),
                suggested_value=d.get("required_force", (d.get("current_force") or 0) * 1.5),
                unit="N",
                confidence=0.9,
                reasoning="Higher actuator force achieves required grip",
                priority=1,
            ),
        },
    ],
    "FINGER_TOO_SHORT": [
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.GEOMETRY_CHANGE,
                component=d.get("finger", "finger"),
                parameter="length",
                current_value=d.get("current_length", 0),
                suggested_value=d.get("required_length", (d.get("current_length") or 0) * 1.2),
                unit="m",
                confidence=0.95,
                reasoning="Longer finger reaches object",
                priority=1,
            ),
        },
    ],
    "GENERIC": [
        {
            "condition": lambda d: True,
            "fix": lambda d: PrescriptiveFix(
                fix_type=FixType.PARAMETER_CHANGE,
                component="design",
                parameter="review",
                current_value="current",
                suggested_value="manual_review",
                unit="",
                confidence=0.3,
                reasoning="Manual review recommended for this failure type",
                priority=3,
            ),
        },
    ],
}


def generate_prescriptive_fixes(
    validation_result: Dict[str, Any],
    design_id: str = "unknown",
) -> FixReport:
    """
    Generate prescriptive fixes from validation result.

    Args:
        validation_result: Output from Reality Bridge validation (dict from to_dict or API).
        design_id: Design identifier.

    Returns:
        FixReport with specific fixes.
    """
    fixes: List[PrescriptiveFix] = []
    warnings: List[str] = []
    failure_codes: List[str] = []

    tests = validation_result.get("tests", {})
    details = validation_result.get("details", {})
    metrics = validation_result.get("metrics", {})
    errors = validation_result.get("errors", [])

    # Merge metrics into details for rule conditions
    def merged_details(test_details: Dict[str, Any]) -> Dict[str, Any]:
        out = dict(details)
        out.update(metrics)
        out.update(test_details or {})
        return out

    for test_name, test_result in tests.items():
        if not isinstance(test_result, dict):
            continue
        if test_result.get("passed", True):
            continue
        failure_code = _TEST_TO_FAILURE_CODE.get(
            test_name, test_name.replace("_TEST", "_FAIL") if "_TEST" in test_name else "GENERIC"
        )
        failure_codes.append(failure_code)
        test_details = test_result.get("details", {})
        ctx = merged_details(test_details)

        rules = FIX_RULES.get(failure_code, FIX_RULES["GENERIC"])
        for rule in rules:
            try:
                if rule["condition"](ctx):
                    fix = rule["fix"](ctx)
                    fixes.append(fix)
            except Exception as e:
                warnings.append(f"Could not generate fix for {failure_code}: {e}")

    for error in errors:
        error_lower = (error or "").lower()
        if "collision" in error_lower:
            failure_codes.append("SELF_COLLISION")
            for rule in FIX_RULES.get("SELF_COLLISION", []):
                try:
                    fix = rule["fix"](merged_details({}))
                    fixes.append(fix)
                except Exception:
                    pass
        elif "mass" in error_lower or "heavy" in error_lower:
            failure_codes.append("MASS_FAIL")
            for rule in FIX_RULES.get("MASS_FAIL", []):
                try:
                    fix = rule["fix"](merged_details({}))
                    fixes.append(fix)
                except Exception:
                    pass

    fixes.sort(key=lambda f: f.priority)
    seen: set = set()
    unique_fixes: List[PrescriptiveFix] = []
    for fix in fixes:
        key = (fix.component, fix.parameter)
        if key not in seen:
            seen.add(key)
            unique_fixes.append(fix)

    if unique_fixes:
        avg_confidence = sum(f.confidence for f in unique_fixes) / len(unique_fixes)
        estimated_improvement = avg_confidence * 0.5
    else:
        estimated_improvement = 0.0

    return FixReport(
        design_id=design_id,
        failure_codes=list(set(failure_codes)),
        fixes=unique_fixes,
        estimated_improvement=min(estimated_improvement, 1.0),
        warnings=warnings,
    )


def format_fix_for_display(fix: PrescriptiveFix) -> str:
    """Format a fix for human-readable display."""
    return (
        f"**{fix.component}.{fix.parameter}**: "
        f"{fix.current_value} → {fix.suggested_value} {fix.unit}\n"
        f"*{fix.reasoning}* (confidence: {fix.confidence:.0%})"
    )


def generate_mjcf_patch(fixes: List[PrescriptiveFix]) -> str:
    """
    Generate MJCF modification suggestions.
    Returns string with suggested XML comments.
    """
    lines = ["<!-- Suggested MJCF modifications -->"]
    for fix in fixes:
        if fix.fix_type == FixType.PARAMETER_CHANGE:
            lines.append(f'<!-- {fix.component}: set {fix.parameter}="{fix.suggested_value}" -->')
        elif fix.fix_type == FixType.GEOMETRY_CHANGE:
            lines.append(f"<!-- {fix.component} geom: adjust {fix.parameter} to {fix.suggested_value} {fix.unit} -->")
        elif fix.fix_type == FixType.MATERIAL_CHANGE:
            lines.append(f"<!-- {fix.component}: change material {fix.parameter} to {fix.suggested_value} {fix.unit} -->")
        elif fix.fix_type == FixType.ACTUATOR_CHANGE:
            lines.append(f'<!-- {fix.component}: set {fix.parameter}="{fix.suggested_value}" -->')
    return "\n".join(lines)
