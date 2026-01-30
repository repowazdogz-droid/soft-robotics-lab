"""
Tests for Reality Bridge validator, loader, reporter, analyzer.
"""

import pytest
import sys
from pathlib import Path

# Add project root so "core" is importable
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.loader import detect_format, load_mjcf, load_model
from core.validator import PhysicsValidator, ValidationResult, TestResult
from core.reporter import to_dict, to_markdown, to_html
from core.analyzer import find_weak_points, estimate_failure_modes, suggest_improvements, analyze


# Minimal valid MJCF (no actuators, static)
MINIMAL_MJCF = """
<mujoco model="test">
  <compiler angle="degree"/>
  <worldbody>
    <body name="box" pos="0 0 0.5">
      <freejoint/>
      <geom name="box_geom" type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

# URDF-like snippet for format detection
URDF_SNIPPET = '<?xml version="1.0"?><robot name="test"><link name="base"><visual><geometry><box size="1 1 1"/></geometry></visual></link></robot>'


class TestDetectFormat:
    def test_mjcf(self):
        assert detect_format("<mujoco model='x'>") == "mjcf"
        assert detect_format(MINIMAL_MJCF) == "mjcf"

    def test_urdf(self):
        assert detect_format("<robot name='x'>") == "urdf"
        assert detect_format(URDF_SNIPPET) == "urdf"

    def test_unknown(self):
        assert detect_format("<foo>") == "unknown"
        assert detect_format("") == "unknown"


class TestLoader:
    def test_load_mjcf_string(self):
        try:
            import mujoco
        except ImportError:
            pytest.skip("MuJoCo not installed")
        model = load_mjcf(xml_string=MINIMAL_MJCF)
        assert model is not None
        assert model.nq > 0

    def test_load_model_returns_tuple(self):
        try:
            import mujoco
        except ImportError:
            pytest.skip("MuJoCo not installed")
        model, fmt = load_model(xml_string=MINIMAL_MJCF)
        assert model is not None
        assert fmt == "mjcf"


class TestValidator:
    def test_validation_result_structure(self):
        result = ValidationResult(
            passed=True,
            score=1.0,
            tests={"LOAD_TEST": TestResult("LOAD_TEST", True, "ok")},
            warnings=[],
            errors=[],
            metrics={"dofs": 7},
        )
        assert result.passed is True
        assert result.score == 1.0
        assert "LOAD_TEST" in result.tests
        assert result.tests["LOAD_TEST"].passed

    def test_validate_minimal_mjcf(self):
        try:
            import mujoco
        except ImportError:
            pytest.skip("MuJoCo not installed")
        v = PhysicsValidator()
        result = v.validate(xml_string=MINIMAL_MJCF)
        assert isinstance(result, ValidationResult)
        assert "LOAD_TEST" in result.tests
        assert result.tests["LOAD_TEST"].passed
        assert "STABILITY_TEST" in result.tests
        assert "metrics" in result
        assert "dofs" in result.metrics or "mass_kg" in result.metrics


class TestReporter:
    def test_to_dict(self):
        result = ValidationResult(
            passed=True,
            score=0.8,
            tests={"T": TestResult("T", True, "ok", {"k": 1})},
            warnings=["w"],
            errors=[],
            metrics={"dofs": 1.0},
        )
        d = to_dict(result)
        assert d["passed"] is True
        assert d["score"] == 0.8
        assert d["tests"]["T"]["passed"] is True
        assert d["tests"]["T"]["details"]["k"] == 1
        assert d["warnings"] == ["w"]
        assert d["metrics"]["dofs"] == 1.0

    def test_to_markdown(self):
        result = ValidationResult(
            passed=True,
            score=1.0,
            tests={"LOAD_TEST": TestResult("LOAD_TEST", True, "Loaded")},
            warnings=[],
            errors=[],
            metrics={"dofs": 7},
        )
        md = to_markdown(result)
        assert "# Validation Report" in md
        assert "LOAD_TEST" in md
        assert "dofs" in md

    def test_to_html(self):
        result = ValidationResult(
            passed=True,
            score=1.0,
            tests={"LOAD_TEST": TestResult("LOAD_TEST", True, "Loaded")},
            warnings=[],
            errors=[],
            metrics={},
        )
        html_str = to_html(result)
        assert "<!DOCTYPE html>" in html_str
        assert "Validation Report" in html_str
        assert "pass" in html_str or "fail" in html_str


class TestAnalyzer:
    def test_analyze_minimal_model(self):
        try:
            import mujoco
        except ImportError:
            pytest.skip("MuJoCo not installed")
        model = load_mjcf(xml_string=MINIMAL_MJCF)
        weak = find_weak_points(model)
        modes = estimate_failure_modes(model)
        sugg = suggest_improvements(model)
        assert isinstance(weak, list)
        assert isinstance(modes, list)
        assert isinstance(sugg, list)
        a = analyze(model)
        assert hasattr(a, "weak_points")
        assert hasattr(a, "failure_modes")
        assert hasattr(a, "suggestions")
        assert hasattr(a, "metrics")
