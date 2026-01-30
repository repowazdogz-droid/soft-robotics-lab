"""
Reality Bridge — Fix suggestions from failure code and validation context.
Provides specific, actionable suggestions and Tutor link.
"""

import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

_PRODUCTS = Path(__file__).resolve().parent.parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))

try:
    from shared.failures import FailureCode, FAILURE_ACTIONS, FAILURE_TUTOR_TOPICS
    from shared.tutor_links import get_tutor_link
except ImportError:
    FailureCode = None
    FAILURE_ACTIONS = {}
    FAILURE_TUTOR_TOPICS = {}
    def get_tutor_link(topic: str, port: int = 8503) -> str:
        return f"http://localhost:{port}/?topic={topic.replace(' ', '+')}"


# Map test names to failure codes for validation result
TEST_TO_FAILURE_CODE = {
    "STABILITY_TEST": "PHYSICS_INSTABILITY",
    "KINEMATICS_TEST": "PHYSICS_INSTABILITY",
    "DYNAMICS_TEST": "PHYSICS_INSTABILITY",
    "MASS_PROPERTIES_TEST": "MATERIAL_OUT_OF_RANGE",  # or PHYSICS if mass huge
    "SELF_COLLISION_TEST": "GEOMETRY_SELF_INTERSECTION",
    "LOAD_TEST": "UNSUPPORTED_ASSUMPTION",
}


def suggest_fixes(failure_code: str, context: Optional[Dict[str, Any]] = None) -> List[str]:
    """
    Return actionable fix suggestions for a failure code and optional context.
    context can include: mass_kg, body_names, errors, failed_tests, metrics, design_type.
    """
    context = context or {}
    suggestions: List[str] = []

    try:
        code = FailureCode(failure_code) if isinstance(failure_code, str) else failure_code
    except (ValueError, TypeError):
        code = None
    if code is None:
        try:
            code = FailureCode(failure_code)
        except (ValueError, TypeError):
            suggestions = ["Review the error message and try again.", "Check API documentation."]
            return suggestions

    base_actions = FAILURE_ACTIONS.get(code, ["Try again"])
    mass_kg = context.get("mass_kg")
    body_names = context.get("body_names") or context.get("bodies") or []
    errors = context.get("errors") or []
    failed_tests = context.get("failed_tests") or []
    metrics = context.get("metrics") or {}

    if code == FailureCode.PHYSICS_INSTABILITY:
        if mass_kg is not None and float(mass_kg) > 100:
            suggestions.append(f"Reduce total mass from {mass_kg:.1f} kg to <10 kg for stability.")
        if "damping" in str(metrics).lower() or "joint" in str(errors).lower():
            suggestions.append("Add or increase joint damping to reduce oscillations.")
        suggestions.append("Reduce simulation timestep (e.g. 0.002 → 0.001).")
        suggestions.extend([a for a in base_actions if a not in suggestions])

    elif code == FailureCode.GEOMETRY_SELF_INTERSECTION:
        if body_names:
            suggestions.append(f"Increase spacing between: {', '.join(body_names[:3])}.")
        else:
            suggestions.append("Increase spacing between components or check joint limits.")
        suggestions.extend([a for a in base_actions if a not in suggestions])

    elif code == FailureCode.GRASP_FAILURE:
        if "gripper" in str(context).lower() or "finger" in str(context).lower():
            suggestions.append("Increase finger length or gripper aperture for the target object.")
        suggestions.append("Adjust grasp approach angle or increase grip force.")
        suggestions.extend([a for a in base_actions if a not in suggestions])

    elif code == FailureCode.MATERIAL_OUT_OF_RANGE:
        if mass_kg is not None:
            if float(mass_kg) <= 0:
                suggestions.append("Set positive mass for all bodies.")
            elif float(mass_kg) > 1e4:
                suggestions.append(f"Reduce total mass from {mass_kg:.1f} kg to a reasonable range (<1000 kg).")
        suggestions.extend([a for a in base_actions if a not in suggestions])

    elif code == FailureCode.INVALID_INPUT_UNITS:
        suggestions.append("Ensure all lengths are in meters, mass in kg, time in seconds.")
        suggestions.append("Check for mm vs m confusion (e.g. 10mm = 0.01 m).")
        suggestions.extend([a for a in base_actions if a not in suggestions])

    else:
        suggestions = list(base_actions)

    return suggestions[:8]  # cap so response stays readable


def get_tutor_link_for_failure(failure_code: str, port: int = 8503) -> Optional[str]:
    """Return Tutor URL for the failure code, or None if no topic."""
    try:
        code = FailureCode(failure_code) if isinstance(failure_code, str) else failure_code
        topic = FAILURE_TUTOR_TOPICS.get(code)
        if topic:
            return get_tutor_link(topic, port)
    except (ValueError, TypeError, AttributeError):
        pass
    return None


def failure_from_validation_result(result) -> Optional[Dict[str, Any]]:
    """
    Infer primary failure code and build context from ValidationResult (core.validator).
    Returns dict: code, message, suggestions, tutor_link; or None if passed.
    """
    if getattr(result, "passed", False):
        return None
    errors = getattr(result, "errors", []) or []
    tests = getattr(result, "tests", {}) or {}
    metrics = getattr(result, "metrics", {}) or {}
    failed_tests = [k for k, v in tests.items() if getattr(v, "passed", True) is False]
    mass_kg = metrics.get("mass_kg")
    code_str = None
    for t in failed_tests:
        code_str = TEST_TO_FAILURE_CODE.get(t)
        if code_str:
            break
    if not code_str and errors:
        err_lower = " ".join(errors).lower()
        if "nan" in err_lower or "inf" in err_lower or "unstable" in err_lower:
            code_str = "PHYSICS_INSTABILITY"
        elif "mass" in err_lower or "stiffness" in err_lower:
            code_str = "MATERIAL_OUT_OF_RANGE"
        elif "collision" in err_lower or "intersect" in err_lower:
            code_str = "GEOMETRY_SELF_INTERSECTION"
    if not code_str:
        code_str = "UNKNOWN"
    context = {"mass_kg": mass_kg, "errors": errors, "failed_tests": failed_tests, "metrics": metrics}
    suggestions = suggest_fixes(code_str, context)
    tutor_link = get_tutor_link_for_failure(code_str)
    try:
        from shared.failures import FAILURE_MESSAGES, FailureCode
        code_enum = FailureCode(code_str)
        msg = FAILURE_MESSAGES.get(code_enum, "Validation failed.")
    except Exception:
        msg = "Validation failed."
    return {
        "code": code_str,
        "message": msg,
        "suggestions": suggestions,
        "tutor_link": tutor_link,
    }
