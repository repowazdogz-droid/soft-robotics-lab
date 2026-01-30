"""
Analyzer - Structural analysis, failure modes, improvement suggestions.
Uses MuJoCo 3.x model API.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional

try:
    import mujoco
    import numpy as np
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


@dataclass
class WeakPoint:
    """A structural weak point (joint or geom)."""
    kind: str  # "joint" | "geom"
    name: str
    reason: str
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class AnalysisResult:
    """Result of structural analysis."""
    weak_points: List[WeakPoint] = field(default_factory=list)
    failure_modes: List[str] = field(default_factory=list)
    suggestions: List[str] = field(default_factory=list)
    metrics: Dict[str, Any] = field(default_factory=dict)


def find_weak_points(model) -> List[WeakPoint]:
    """
    Identify joints with extreme ranges and thin or extreme geoms.
    Expects MuJoCo MjModel (3.x).
    """
    out: List[WeakPoint] = []
    if not MUJOCO_AVAILABLE or model is None:
        return out

    try:
        njnt = getattr(model, "njnt", 0)
        if njnt > 0 and hasattr(model, "jnt_range"):
            rng = model.jnt_range  # (njnt, 2)
            for i in range(njnt):
                lo, hi = float(rng[i, 0]), float(rng[i, 1])
                span = hi - lo
                name = _jnt_name(model, i)
                if span > 1e4:
                    out.append(WeakPoint(
                        "joint", name or f"jnt_{i}",
                        "Extreme joint range",
                        {"range": [lo, hi], "span": span},
                    ))
                elif span > 0 and span < 1e-6:
                    out.append(WeakPoint(
                        "joint", name or f"jnt_{i}",
                        "Very small joint range",
                        {"range": [lo, hi], "span": span},
                    ))

        ngeom = getattr(model, "ngeom", 0)
        if ngeom > 0 and hasattr(model, "geom_size"):
            sz = model.geom_size  # (ngeom, 3)
            for i in range(ngeom):
                s = sz[i]
                min_dim = float(np.min(s))
                max_dim = float(np.max(s))
                name = _geom_name(model, i)
                if min_dim > 0 and min_dim < 1e-4:
                    out.append(WeakPoint(
                        "geom", name or f"geom_{i}",
                        "Thin or very small geom",
                        {"size": s.tolist(), "min_dim": min_dim},
                    ))
                if max_dim > 100:
                    out.append(WeakPoint(
                        "geom", name or f"geom_{i}",
                        "Very large geom",
                        {"size": s.tolist(), "max_dim": max_dim},
                    ))
    except Exception:
        pass
    return out


def estimate_failure_modes(model) -> List[str]:
    """
    Estimate what could break: no damping, extreme stiffness, zero mass, etc.
    """
    modes: List[str] = []
    if not MUJOCO_AVAILABLE or model is None:
        return modes

    try:
        nv = getattr(model, "nv", 0)
        if nv > 0 and hasattr(model, "dof_damping"):
            d = model.dof_damping
            for i in range(nv):
                if float(d[i]) <= 0:
                    jname = _dof_jnt_name(model, i)
                    modes.append(f"Joint/DOF '{jname}' has no or negative damping")
        nu = getattr(model, "nu", 0)
        if nu == 0 and (getattr(model, "nq", 0) > 0 or nv > 0):
            modes.append("No actuators defined; open-loop only")
        mass = np.sum(model.body_mass)
        if mass <= 0:
            modes.append("Total model mass is zero or negative")
        if mass > 1e5:
            modes.append("Very high total mass may cause numerical issues")
        njnt = getattr(model, "njnt", 0)
        if njnt > 0 and hasattr(model, "jnt_stiffness"):
            stiff = model.jnt_stiffness
            for i in range(njnt):
                if float(stiff[i]) > 1e6:
                    modes.append(f"Joint '{_jnt_name(model, i)}' has very high stiffness")
    except Exception:
        pass
    return modes


def suggest_improvements(model) -> List[str]:
    """
    Suggest improvements: increase damping, add limits, etc.
    """
    suggestions: List[str] = []
    if not MUJOCO_AVAILABLE or model is None:
        return suggestions

    try:
        weak = find_weak_points(model)
        for w in weak:
            if w.kind == "joint" and "Extreme" in w.reason:
                suggestions.append(f"Consider limiting range of joint '{w.name}'")
            if w.kind == "geom" and "Thin" in w.reason:
                suggestions.append(f"Increase minimum dimension of geom '{w.name}' or check units")

        nv = getattr(model, "nv", 0)
        if nv > 0 and hasattr(model, "dof_damping"):
            d = model.dof_damping
            for i in range(nv):
                if float(d[i]) <= 0:
                    jname = _dof_jnt_name(model, i)
                    suggestions.append(f"Increase damping on joint '{jname}'")
        nu = getattr(model, "nu", 0)
        if nu == 0 and nv > 0:
            suggestions.append("Add actuators for controlled motion")
    except Exception:
        pass
    return suggestions


def analyze(model) -> AnalysisResult:
    """
    Full analysis: weak points, failure modes, suggestions.
    """
    weak = find_weak_points(model) if model else []
    modes = estimate_failure_modes(model) if model else []
    sugg = suggest_improvements(model) if model else []
    metrics: Dict[str, Any] = {}
    if MUJOCO_AVAILABLE and model is not None:
        try:
            metrics["nq"] = getattr(model, "nq", 0)
            metrics["nv"] = getattr(model, "nv", 0)
            metrics["nu"] = getattr(model, "nu", 0)
            metrics["nbody"] = getattr(model, "nbody", 0)
            metrics["njnt"] = getattr(model, "njnt", 0)
            metrics["ngeom"] = getattr(model, "ngeom", 0)
            metrics["mass_kg"] = float(np.sum(model.body_mass))
        except Exception:
            pass
    return AnalysisResult(
        weak_points=weak,
        failure_modes=modes,
        suggestions=sugg,
        metrics=metrics,
    )


def _jnt_name(model, i: int) -> Optional[str]:
    try:
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    except Exception:
        pass
    return None


def _geom_name(model, i: int) -> Optional[str]:
    try:
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    except Exception:
        pass
    return None


def _dof_jnt_name(model, i: int) -> Optional[str]:
    try:
        if hasattr(model, "dof_jntid"):
            jid = model.dof_jntid[i]
            return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
    except Exception:
        pass
    return f"dof_{i}"
