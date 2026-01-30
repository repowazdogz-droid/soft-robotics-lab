"""
Grasp test v2: high-friction egg (no rolling), gripper starts beside/above egg (no approach push).
Sequence: close fingers -> lift. No lower phase that pushes egg away.
Success: egg_z > 0.15 after lift.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import mujoco
    import numpy as np
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


def get_egg_z(data, model):
    """Return egg body z position."""
    try:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object_egg")
        return float(data.xpos[bid][2])
    except Exception:
        if model.nq >= 7:
            return float(data.qpos[-4])
        return 0.0


def run_grasp_v2(
    model,
    data,
    n_close_steps: int = 500,
    n_lift_steps: int = 600,
    z_lift: float = 0.2,
    verbose: bool = True,
) -> dict:
    """
    No approach phase. Gripper already at egg level (composed that way).
    Phase 1: close fingers (ctrl flex=1, ext=0).
    Phase 2: lift (ctrl z = z_lift).
    Actuator order: [x, y, z, finger0_flex, finger0_ext, finger1_flex, finger1_ext].
    """
    nu = model.nu
    if nu < 7:
        if verbose:
            print(f"  [WARN] Only {nu} actuators; need 7.")
        return {"egg_z_initial": get_egg_z(data, model), "egg_z_after_lift": 0.0, "success": False}

    egg_z_initial = get_egg_z(data, model)
    if verbose:
        print(f"  Initial: egg_z = {egg_z_initial:.4f} (no lower phase)")

    data.ctrl[:] = 0.0
    data.ctrl[0] = 0.0
    data.ctrl[1] = 0.0
    data.ctrl[2] = 0.0
    for _ in range(100):
        mujoco.mj_step(model, data)
    egg_z_after_settle = get_egg_z(data, model)
    if verbose:
        print(f"  After settle (100 steps): egg_z = {egg_z_after_settle:.4f}")

    data.ctrl[3] = 1.0
    data.ctrl[4] = 0.0
    data.ctrl[5] = 1.0
    data.ctrl[6] = 0.0
    for _ in range(n_close_steps):
        mujoco.mj_step(model, data)
    egg_z_after_close = get_egg_z(data, model)
    if verbose:
        print(f"  After close ({n_close_steps} steps): egg_z = {egg_z_after_close:.4f}")

    data.ctrl[2] = z_lift
    for _ in range(n_lift_steps):
        mujoco.mj_step(model, data)
    egg_z_after_lift = get_egg_z(data, model)
    if verbose:
        print(f"  After lift ({n_lift_steps} steps): egg_z = {egg_z_after_lift:.4f}")

    success = egg_z_after_lift > 0.15
    return {
        "egg_z_initial": egg_z_initial,
        "egg_z_after_settle": egg_z_after_settle,
        "egg_z_after_close": egg_z_after_close,
        "egg_z_after_lift": egg_z_after_lift,
        "success": success,
    }


def load_scene_v2(gripper_pos=None):
    """Compose scene: high-friction egg (asset_library), gripper beside/above egg, no approach."""
    from core.scene_composer import SceneComposer
    composer = SceneComposer()
    composer.add_surface("table", [0, 0, 0])
    gripper_path = ROOT / "assets" / "grippers" / "test_gripper.mjcf"
    if not gripper_path.exists():
        gripper_path = ROOT / "assets" / "grippers" / "test_gripper.xml"
    if not gripper_path.exists():
        gripper_path = ROOT.parent / "omega_foundry" / "outputs" / "test_gripper.xml"
    if not gripper_path.exists():
        raise FileNotFoundError("No gripper file found")
    pos = gripper_pos if gripper_pos is not None else [0.0, 0.015, 0.055]
    composer.add_gripper_from_file(str(gripper_path), pos=pos, mobile=True)
    composer.add_object("egg", pos=[0, 0, 0.05], mass=0.08)
    return mujoco.MjModel.from_xml_string(composer.compose())


def run_test_v2(
    n_close_steps: int = 500,
    n_lift_steps: int = 600,
    z_lift: float = 0.2,
    gripper_pos=None,
    max_attempts: int = 5,
    verbose: bool = True,
) -> dict:
    """
    Load scene v2 (high-friction egg, gripper beside/above egg); run close -> lift; retry with different pos if needed.
    """
    if not MUJOCO_AVAILABLE:
        print("MuJoCo not available.")
        return {"success": False}

    if gripper_pos is None:
        gripper_pos = [0.0, 0.015, 0.055]
    gripper_pos = list(gripper_pos)
    result = {}

    for attempt in range(max_attempts):
        try:
            model = load_scene_v2(gripper_pos=gripper_pos)
        except Exception as e:
            print(f"Load failed: {e}")
            return {"success": False, "attempt": attempt + 1}
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        if verbose:
            print(f"\n--- Attempt {attempt + 1}/{max_attempts} (gripper_pos={gripper_pos}) ---")
        result = run_grasp_v2(
            model,
            data,
            n_close_steps=n_close_steps,
            n_lift_steps=n_lift_steps,
            z_lift=z_lift,
            verbose=verbose,
        )
        if result["success"]:
            if verbose:
                print("  SUCCESS: egg_z_after_lift > 0.15")
            result["attempt"] = attempt + 1
            return result
        result["attempt"] = attempt + 1
        if attempt < max_attempts - 1:
            gripper_pos[1] += 0.005
            gripper_pos[2] = max(0.04, gripper_pos[2] - 0.005)
            if verbose:
                print(f"  Retry with gripper_pos={gripper_pos}")

    result["attempt"] = max_attempts
    return result


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Grasp v2: close -> lift, no approach, high-friction egg")
    p.add_argument("--close-steps", type=int, default=500)
    p.add_argument("--lift-steps", type=int, default=600)
    p.add_argument("--z-lift", type=float, default=0.2)
    p.add_argument("--gripper-x", type=float, default=0.0)
    p.add_argument("--gripper-y", type=float, default=0.015, help="Slight Y offset so palm does not push egg")
    p.add_argument("--gripper-z", type=float, default=0.055, help="Palm at egg level / slightly above")
    p.add_argument("--max-attempts", type=int, default=5)
    p.add_argument("--quiet", action="store_true")
    args = p.parse_args()

    gripper_pos = [args.gripper_x, args.gripper_y, args.gripper_z]
    result = run_test_v2(
        n_close_steps=args.close_steps,
        n_lift_steps=args.lift_steps,
        z_lift=args.z_lift,
        gripper_pos=gripper_pos,
        max_attempts=args.max_attempts,
        verbose=not args.quiet,
    )
    print("\nResult:", "SUCCESS" if result.get("success") else "FAILED", "| egg_z_after_lift =", result.get("egg_z_after_lift", 0))
    sys.exit(0 if result.get("success") else 1)
