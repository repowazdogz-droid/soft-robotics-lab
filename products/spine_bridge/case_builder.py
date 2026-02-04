"""
Converts Reality Bridge validation output + user problem context into Spine CaseInput.
Maps physics failures → constraints; validation warnings → uncertainties.
"""

from typing import Any, Dict, List

from schemas import CaseInput


def build_case(
    problem_context: Dict[str, Any],
    physics_result: Dict[str, Any],
) -> CaseInput:
    """
    Build Spine CaseInput from:
    - problem_context: { "name", "domain", "objectives" } (and optional extra fields)
    - physics_result: Reality Bridge /validate response (success, passed, failures, warnings, errors).
    """
    name = problem_context.get("name") or "Unnamed design"
    domain = problem_context.get("domain") or "robotics"
    objectives = problem_context.get("objectives")
    if isinstance(objectives, str):
        objectives = [o.strip() for o in objectives.split(",") if o.strip()]
    objectives = objectives or ["Validate physics", "Assess feasibility"]

    # Constraints: from physics failures (must satisfy for design to be acceptable)
    constraints: List[str] = []
    for f in physics_result.get("failures") or []:
        code = f.get("code", "UNKNOWN")
        msg = f.get("message", "")
        constraints.append(f"[{code}] {msg}".strip() or code)
    for e in physics_result.get("errors") or []:
        if isinstance(e, str):
            constraints.append(e)
        else:
            constraints.append(str(e))

    # Uncertainties: from warnings and unknowns
    uncertainties: List[str] = []
    for w in physics_result.get("warnings") or []:
        if isinstance(w, str):
            uncertainties.append(w)
        else:
            uncertainties.append(str(w))
    # If validation failed, add failure suggestions as uncertainties (design may be fixable)
    for f in physics_result.get("failures") or []:
        for s in f.get("suggestions") or []:
            uncertainties.append(f"Suggestion: {s}")

    return CaseInput(
        name=name,
        domain=domain,
        objectives=objectives,
        constraints=constraints,
        uncertainties=uncertainties,
        context={
            "physics_passed": physics_result.get("passed", False),
            "physics_score": physics_result.get("score"),
            "validation_id": physics_result.get("validation_id"),
            **{k: v for k, v in problem_context.items() if k not in ("name", "domain", "objectives")},
        },
    )
