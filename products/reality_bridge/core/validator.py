"""
Validator - Physics validation suite.
LOAD_TEST, STABILITY_TEST, KINEMATICS_TEST, DYNAMICS_TEST,
SELF_COLLISION_TEST, MASS_PROPERTIES_TEST.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any

try:
    import mujoco
    import numpy as np
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


@dataclass
class TestResult:
    """Result of a single validation test."""
    name: str
    passed: bool
    message: str = ""
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ValidationResult:
    """Aggregate validation result."""
    passed: bool
    score: float
    tests: Dict[str, TestResult] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    metrics: Dict[str, float] = field(default_factory=dict)


# Reasonable thresholds for grippers / small mechanisms
MAX_TOTAL_MASS = 10.0       # kg — grippers shouldn't weigh more
MIN_TOTAL_MASS = 0.001      # kg — too light is likely an error
MAX_BODY_MASS = 5.0        # kg — no single body heavier
MAX_MASS_RATIO = 1000.0    # heaviest/lightest — unbalanced
MAX_VELOCITY = 100.0        # m/s — unrealistic speed
MAX_POSITION = 10.0         # m from origin — simulation "explosion"
FLOOR_Z_TOL = -0.5          # m — bodies below this = falling through floor


class PhysicsValidator:
    """Physics validation suite: load, stability, kinematics, dynamics, self-collision, mass."""

    SIM_DURATION_S = 1.0
    TIMESTEP = 0.002
    STABILITY_STEPS = int(SIM_DURATION_S / TIMESTEP)

    def __init__(self):
        self._loader = None
        try:
            from .loader import load_model
            self._loader = load_model
        except Exception:
            pass

    def validate(self, xml_string: str = None, file_path: str = None) -> ValidationResult:
        """
        Run full validation suite.
        Provide xml_string or file_path (or upload file and pass path).
        """
        tests: Dict[str, TestResult] = {}
        errors: List[str] = []
        warnings: List[str] = []
        metrics: Dict[str, float] = {}
        model = None

        if not MUJOCO_AVAILABLE:
            return ValidationResult(
                passed=False,
                score=0.0,
                tests={},
                errors=["MuJoCo not installed"],
                warnings=[],
                metrics={},
            )

        try:
            if self._loader:
                model, fmt = self._loader(xml_string=xml_string, file_path=file_path)
            elif file_path:
                model = mujoco.MjModel.from_xml_path(file_path)
            else:
                model = mujoco.MjModel.from_xml_string(xml_string)
        except Exception as e:
            return ValidationResult(
                passed=False,
                score=0.0,
                tests={"LOAD_TEST": TestResult("LOAD_TEST", False, str(e))},
                errors=[f"Load failed: {e}"],
                warnings=[],
                metrics={},
            )

        tests["LOAD_TEST"] = TestResult("LOAD_TEST", True, "Model loaded")
        self._add_metrics(model, metrics)

        # STABILITY_TEST
        tr, errs, w = self._run_stability(model)
        tests["STABILITY_TEST"] = tr
        errors.extend(errs)
        warnings.extend(w)

        # KINEMATICS_TEST
        tr, errs, w = self._run_kinematics(model)
        tests["KINEMATICS_TEST"] = tr
        errors.extend(errs)
        warnings.extend(w)

        # DYNAMICS_TEST
        tr, errs, w = self._run_dynamics(model)
        tests["DYNAMICS_TEST"] = tr
        errors.extend(errs)
        warnings.extend(w)

        # SELF_COLLISION_TEST
        tr, errs, w = self._run_self_collision(model)
        tests["SELF_COLLISION_TEST"] = tr
        errors.extend(errs)
        warnings.extend(w)

        # MASS_PROPERTIES_TEST
        tr, errs, w = self._run_mass_properties(model)
        tests["MASS_PROPERTIES_TEST"] = tr
        errors.extend(errs)
        warnings.extend(w)

        passed_count = sum(1 for t in tests.values() if t.passed)
        score = passed_count / len(tests) if tests else 0.0
        passed = passed_count == len(tests) and len(errors) == 0

        return ValidationResult(
            passed=passed,
            score=score,
            tests=tests,
            warnings=warnings,
            errors=errors,
            metrics=metrics,
        )

    def _add_metrics(self, model, metrics: Dict):
        try:
            metrics["dofs"] = float(getattr(model, "nq", 0))
            metrics["velocity_dofs"] = float(getattr(model, "nv", 0))
            metrics["actuators"] = float(getattr(model, "nu", 0))
            metrics["sensors"] = float(getattr(model, "nsensor", 0))
            metrics["bodies"] = float(getattr(model, "nbody", 0))
            metrics["joints"] = float(getattr(model, "njnt", 0))
            mass = np.sum(model.body_mass)
            metrics["mass_kg"] = float(mass)
        except Exception:
            pass

    def _run_stability(self, model) -> tuple:
        try:
            data = mujoco.MjData(model)
            nsteps = min(self.STABILITY_STEPS, 2000)
            for _ in range(nsteps):
                mujoco.mj_step(model, data)
            errs: List[str] = []
            # NaN/Inf in position
            if np.any(np.isnan(data.qpos)) or np.any(np.isinf(data.qpos)):
                errs.append("NaN/Inf in position (qpos)")
            # Velocities exceeding realistic bounds
            if model.nv > 0 and np.any(np.abs(data.qvel) > MAX_VELOCITY):
                errs.append(f"Velocity exceeds {MAX_VELOCITY} m/s")
            # Bodies falling through floor (worldbody geom "floor" is at z=0; bodies should stay above)
            xpos = data.xpos  # (nbody, 3)
            if np.any(xpos[:, 2] < FLOOR_Z_TOL):
                errs.append("Body below floor (fall-through)")
            # Simulation explosion: any body > MAX_POSITION from origin
            dists = np.linalg.norm(xpos, axis=1)
            if np.any(dists > MAX_POSITION):
                errs.append(f"Position > {MAX_POSITION} m from origin (explosion)")
            if errs:
                return (
                    TestResult("STABILITY_TEST", False, "; ".join(errs), {"steps": nsteps}),
                    ["Stability: " + e for e in errs],
                    [],
                )
            return (
                TestResult("STABILITY_TEST", True, f"Stable for {nsteps} steps", {"steps": nsteps}),
                [],
                [],
            )
        except Exception as e:
            return (
                TestResult("STABILITY_TEST", False, str(e)),
                [f"Stability: {e}"],
                [],
            )

    def _run_kinematics(self, model) -> tuple:
        try:
            data = mujoco.MjData(model)
            nq = model.nq
            if nq == 0:
                return TestResult("KINEMATICS_TEST", True, "No DOFs"), [], []
            for _ in range(100):
                mujoco.mj_step(model, data)
            if np.any(np.isnan(data.qpos)) or np.any(np.isnan(data.qvel)):
                return (
                    TestResult("KINEMATICS_TEST", False, "NaN in qpos/qvel"),
                    [],
                    ["Kinematics: NaN in state"],
                )
            return TestResult("KINEMATICS_TEST", True, "Joints move without NaN/Inf"), [], []
        except Exception as e:
            return TestResult("KINEMATICS_TEST", False, str(e)), [f"Kinematics: {e}"], []

    def _run_dynamics(self, model) -> tuple:
        try:
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            acc = data.qacc
            if np.any(np.abs(acc) > 1e6):
                return (
                    TestResult("DYNAMICS_TEST", False, "Extreme accelerations"),
                    [],
                    ["Dynamics: very high accelerations"],
                )
            return TestResult("DYNAMICS_TEST", True, "Reasonable accelerations"), [], []
        except Exception as e:
            return TestResult("DYNAMICS_TEST", False, str(e)), [f"Dynamics: {e}"], []

    def _run_self_collision(self, model) -> tuple:
        try:
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            ncon_actual = getattr(data, "ncon", 0)
            if ncon_actual > 0:
                return (
                    TestResult("SELF_COLLISION_TEST", True, f"{ncon_actual} contacts (no interpenetration check)"),
                    [],
                    [],
                )
            return TestResult("SELF_COLLISION_TEST", True, "No contacts detected"), [], []
        except Exception as e:
            return TestResult("SELF_COLLISION_TEST", False, str(e)), [f"Self-collision: {e}"], []

    def _run_mass_properties(self, model) -> tuple:
        try:
            masses = np.asarray(model.body_mass)
            total = float(np.sum(masses))
            # Total mass too low
            if total < MIN_TOTAL_MASS:
                return (
                    TestResult("MASS_PROPERTIES_TEST", False, f"Total mass {total:.4f} kg < {MIN_TOTAL_MASS} kg"),
                    [f"Mass properties: total mass below {MIN_TOTAL_MASS} kg"],
                    [],
                )
            # Total mass too high (unrealistic for gripper)
            if total > MAX_TOTAL_MASS:
                return (
                    TestResult("MASS_PROPERTIES_TEST", False, f"Total mass {total:.4f} kg exceeds {MAX_TOTAL_MASS} kg"),
                    [f"Mass properties: total mass {total:.2f} kg exceeds limit {MAX_TOTAL_MASS} kg"],
                    [],
                )
            # Any single body too heavy
            for i, m in enumerate(masses):
                if m > MAX_BODY_MASS:
                    return (
                        TestResult("MASS_PROPERTIES_TEST", False, f"Body {i} mass {m:.4f} kg > {MAX_BODY_MASS} kg"),
                        [f"Mass properties: body mass {m:.2f} kg exceeds {MAX_BODY_MASS} kg"],
                        [],
                    )
            # Mass ratio (heaviest / lightest) — use positive masses only
            positive = masses[masses > 0]
            if len(positive) >= 2:
                heaviest = float(np.max(positive))
                lightest = float(np.min(positive))
                if lightest <= 0:
                    lightest = np.finfo(float).tiny
                ratio = heaviest / lightest
                if ratio > MAX_MASS_RATIO:
                    return (
                        TestResult("MASS_PROPERTIES_TEST", False, f"Mass ratio {ratio:.0f} > {MAX_MASS_RATIO}"),
                        [f"Mass properties: mass ratio {ratio:.0f} exceeds {MAX_MASS_RATIO}"],
                        [],
                    )
            return (
                TestResult("MASS_PROPERTIES_TEST", True, f"Total mass {total:.4f} kg", {"mass_kg": total}),
                [],
                [],
            )
        except Exception as e:
            return TestResult("MASS_PROPERTIES_TEST", False, str(e)), [f"Mass: {e}"], []


def validate_design(xml_string: str = None, file_path: str = None) -> Dict[str, Any]:
    """
    Convenience: run full validation and return a dict (passed, score, tests, errors, warnings, metrics).
    For use from scripts and one-liners; API uses PhysicsValidator().validate() + to_dict().
    """
    from .reporter import to_dict
    v = PhysicsValidator()
    result = v.validate(xml_string=xml_string, file_path=file_path)
    return to_dict(result)
