"""
Physics validator - MuJoCo load + stability check.
Returns: load result, DOFs, actuators, sensors, estimated mass, warnings.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

try:
    import mujoco
    import numpy as np
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


@dataclass
class ValidationResult:
    """Result of validating a design (MJCF)."""
    valid: bool
    loads: bool
    simulates: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    stats: dict = field(default_factory=dict)


class PhysicsValidator:
    """Validate MJCF: load model, run simulation, check stability."""

    def validate_mjcf_path(self, mjcf_path: str) -> ValidationResult:
        if not MUJOCO_AVAILABLE:
            return ValidationResult(
                valid=False,
                loads=False,
                simulates=False,
                errors=["MuJoCo not installed"],
            )
        path = Path(mjcf_path)
        if not path.exists():
            return ValidationResult(
                valid=False,
                loads=False,
                simulates=False,
                errors=[f"File not found: {mjcf_path}"],
            )
        return self._validate(mujoco.MjModel.from_xml_path(str(path)))

    def validate_mjcf_string(self, mjcf_xml: str) -> ValidationResult:
        if not MUJOCO_AVAILABLE:
            return ValidationResult(
                valid=False,
                loads=False,
                simulates=False,
                errors=["MuJoCo not installed"],
            )
        try:
            model = mujoco.MjModel.from_xml_string(mjcf_xml)
        except Exception as e:
            return ValidationResult(
                valid=False,
                loads=False,
                simulates=False,
                errors=[f"Failed to load MJCF: {e}"],
            )
        return self._validate(model)

    def _validate(self, model) -> ValidationResult:
        errors: List[str] = []
        warnings: List[str] = []

        nq = getattr(model, "nq", 0)
        nv = getattr(model, "nv", 0)
        nu = getattr(model, "nu", 0)
        nsensor = getattr(model, "nsensor", 0)
        nbody = getattr(model, "nbody", 0)

        stats = {
            "bodies": nbody,
            "joints": getattr(model, "njnt", 0),
            "dofs": nq,
            "velocity_dofs": nv,
            "actuators": nu,
            "sensors": nsensor,
        }

        try:
            mass = float(np.sum(model.body_mass))
            stats["mass_kg"] = round(mass, 4)
        except Exception:
            stats["mass_kg"] = None

        simulates = False
        try:
            data = mujoco.MjData(model)
            for _ in range(100):
                mujoco.mj_step(model, data)
            if np.any(np.isnan(data.qpos)) or np.any(np.isinf(data.qpos)):
                errors.append("Simulation unstable (NaN/Inf in positions)")
            else:
                simulates = True
            stats["sim_steps"] = 100
        except Exception as e:
            errors.append(f"Simulation failed: {e}")

        if nu == 0 and getattr(model, "njnt", 0) > 0:
            warnings.append("No actuators defined")

        try:
            jnt_stiffness = getattr(model, "jnt_stiffness", None)
            if jnt_stiffness is not None and np.any(jnt_stiffness > 1.0):
                warnings.append("High joint stiffness may cause instability")
        except Exception:
            pass

        valid = len(errors) == 0 and simulates
        return ValidationResult(
            valid=valid,
            loads=True,
            simulates=simulates,
            errors=errors,
            warnings=warnings,
            stats=stats,
        )
