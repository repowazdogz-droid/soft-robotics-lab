"""
Design → Physics Validation → Decision Analysis pipeline.
Deterministic: no LLM calls. Handles Reality Bridge offline gracefully.
"""

import sys
from pathlib import Path
from typing import Any, Dict, Optional

from reality_bridge_client import (
    RealityBridgeUnavailable,
    get_base_url,
    validate_design,
)
from case_builder import build_case
from schemas import CaseInput

# Repo root so spine.runtime is importable (spine/ lives at repo root)
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Spine Decision Runtime: optional import
_analyzer = None
_SpineCaseInput = None
_using_spine_runtime = False

try:
    from spine.runtime.analyzer import DecisionAnalyzer
    from spine.runtime.schemas import CaseInput as SpineCaseInput
    _analyzer = DecisionAnalyzer()
    _SpineCaseInput = SpineCaseInput
    _using_spine_runtime = True
except ImportError as e:
    print(
        "Spine Bridge: spine.runtime not available "
        "(add repo root to PYTHONPATH or ensure spine/runtime/ exists). "
        f"ImportError: {e}. Decision analysis will use stub."
    )


def analyze_design(
    design_path: Optional[str] = None,
    design_xml: Optional[str] = None,
    problem_context: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    1. Call Reality Bridge /validate (design_path or design_xml).
    2. Extract physics results (pass/fail, issues, suggestions).
    3. Convert to Spine case format (constraints from physics, uncertainties from warnings).
    4. Run Spine analyzer if available.
    5. Return combined result: physics_report, spine_case, decision_analysis, spine_available.
    """
    problem_context = problem_context or {}
    result = {
        "physics_report": None,
        "physics_available": False,
        "spine_case": None,
        "decision_analysis": None,
        "spine_available": _using_spine_runtime,
        "error": None,
    }

    # 1. Reality Bridge validation
    try:
        if design_path:
            rb_response = validate_design(file_path=design_path, artifact_id=problem_context.get("name"))
        elif design_xml:
            rb_response = validate_design(xml_string=design_xml, artifact_id=problem_context.get("name"))
        else:
            result["error"] = "Provide design_path or design_xml"
            return result
    except RealityBridgeUnavailable as e:
        result["error"] = f"Reality Bridge unavailable: {e}"
        result["physics_report"] = {"passed": None, "message": str(e)}
        result["spine_case"] = build_case(
            problem_context,
            {"passed": None, "failures": [], "warnings": [str(e)], "errors": []},
        ).to_dict()
        result["decision_analysis"] = _stub_decision_analysis(result["spine_case"], reason="physics_unavailable")
        return result
    except Exception as e:
        result["error"] = str(e)
        result["physics_report"] = {"passed": False, "message": str(e)}
        result["physics_available"] = False
        result["spine_case"] = build_case(
            problem_context,
            {"passed": False, "failures": [{"code": "UNKNOWN", "message": str(e), "suggestions": []}], "warnings": [], "errors": []},
        ).to_dict()
        result["decision_analysis"] = _stub_decision_analysis(result["spine_case"], reason="validation_error")
        return result

    result["physics_available"] = True
    result["physics_report"] = {
        "passed": rb_response.get("passed"),
        "score": rb_response.get("score"),
        "validation_id": rb_response.get("validation_id"),
        "validation_time_ms": rb_response.get("validation_time_ms"),
        "tests": rb_response.get("tests"),
        "metrics": rb_response.get("metrics"),
        "warnings": rb_response.get("warnings") or [],
        "errors": rb_response.get("errors") or [],
        "failures": rb_response.get("failures") or [],
    }

    # 2 & 3. Build Spine case
    case = build_case(problem_context, rb_response)
    result["spine_case"] = case.to_dict()

    # 4. Run Spine analyzer if available
    if _using_spine_runtime and _analyzer is not None and _SpineCaseInput is not None:
        try:
            spine_case = _to_spine_case_input(case)
            analysis = _analyzer.analyze(spine_case)
            result["decision_analysis"] = _analysis_to_dict(analysis)
        except Exception as e:
            result["decision_analysis"] = _stub_decision_analysis(result["spine_case"], reason=f"spine_analyzer_error: {e}")
    else:
        result["decision_analysis"] = _stub_decision_analysis(result["spine_case"], reason="spine_runtime_not_installed")

    return result


def _to_spine_case_input(case: CaseInput):
    """Convert local CaseInput to spine.runtime.schemas.CaseInput for the analyzer."""
    if _SpineCaseInput is None:
        return case.to_dict()
    d = case.to_dict()
    # Spine CaseInput may use same or subset of fields; try dict unpacking first
    try:
        return _SpineCaseInput(**d)
    except TypeError:
        return _SpineCaseInput(
            name=d.get("name", ""),
            domain=d.get("domain", ""),
            objectives=d.get("objectives", []),
            constraints=d.get("constraints", []),
            uncertainties=d.get("uncertainties", []),
        )


def _analysis_to_dict(analysis: Any) -> Dict[str, Any]:
    """Normalize Spine analyzer output to a JSON-serializable dict."""
    if analysis is None:
        return {}
    if isinstance(analysis, dict):
        return analysis
    if hasattr(analysis, "model_dump"):
        return analysis.model_dump()
    if hasattr(analysis, "__dict__"):
        return {k: v for k, v in analysis.__dict__.items() if not k.startswith("_")}
    return {"raw": str(analysis)}


def _stub_decision_analysis(spine_case: Dict[str, Any], reason: str = "") -> Dict[str, Any]:
    """Deterministic stub when Spine runtime is unavailable."""
    constraints = spine_case.get("constraints") or []
    uncertainties = spine_case.get("uncertainties") or []
    return {
        "summary": "Decision analysis run without Spine runtime; interpret physics report and case below.",
        "recommendation": "PROCEED" if not constraints else "ADDRESS_CONSTRAINTS",
        "constraints_count": len(constraints),
        "uncertainties_count": len(uncertainties),
        "reason": reason,
        "constraints": constraints,
        "uncertainties": uncertainties,
    }
