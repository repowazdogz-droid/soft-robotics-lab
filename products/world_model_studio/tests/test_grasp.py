"""
Grasp test: load scene (gripper + egg), run sequence lower -> close -> lift,
print egg Z at each phase. Iterate until egg Z > 0.15 after lift.
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


def get_egg_z(data, model) -> float:
    """Return egg body z position. Uses body name 'object_egg'."""
    try:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object_egg")
        return float(data.xpos[bid][2])
    except Exception:
        if model.nq >= 7:
            return float(data.qpos[-4])
        return 0.0


def run_grasp_sequence(
    model,
    data,
    n_lower_steps: int = 250,
    n_close_steps: int = 400,
    n_lift_steps: int = 500,
    z_lower: float = -0.04,
    z_lift: float = 0.2,
    verbose: bool = True,
) -> dict:
    """
    Run: lower gripper -> close fingers -> lift.
    Actuator order assumed: [x, y, z, finger0_flex, finger0_ext, finger1_flex, finger1_ext].
    Returns dict with egg_z at each phase and success (egg_z_after_lift > 0.15).
    """
    nu = model.nu
    if nu < 7:
        if verbose:
            print(f"  [WARN] Only {nu} actuators; need 7 for grasp sequence.")
        return {"egg_z_initial": get_egg_z(data, model), "egg_z_after_lift": 0.0, "success": False}

    egg_z_initial = get_egg_z(data, model)
    if verbose:
        print(f"  Phase 0 (initial): egg_z = {egg_z_initial:.4f}")

    data.ctrl[:] = 0.0
    data.ctrl[0] = 0.0
    data.ctrl[1] = 0.0
    data.ctrl[2] = z_lower
    data.ctrl[3] = 0.0
    data.ctrl[4] = 0.0
    data.ctrl[5] = 0.0
    data.ctrl[6] = 0.0
    for _ in range(n_lower_steps):
        mujoco.mj_step(model, data)
    egg_z_after_lower = get_egg_z(data, model)
    if verbose:
        print(f"  Phase 1 (lower): egg_z = {egg_z_after_lower:.4f} (target z_lower={z_lower})")

    data.ctrl[3] = 1.0
    data.ctrl[4] = 0.0
    data.ctrl[5] = 1.0
    data.ctrl[6] = 0.0
    for _ in range(n_close_steps):
        mujoco.mj_step(model, data)
    egg_z_after_close = get_egg_z(data, model)
    if verbose:
        print(f"  Phase 2 (close): egg_z = {egg_z_after_close:.4f}")

    data.ctrl[2] = z_lift
    for _ in range(n_lift_steps):
        mujoco.mj_step(model, data)
    egg_z_after_lift = get_egg_z(data, model)
    if verbose:
        print(f"  Phase 3 (lift): egg_z = {egg_z_after_lift:.4f} (target z_lift={z_lift})")

    success = egg_z_after_lift > 0.15
    return {
        "egg_z_initial": egg_z_initial,
        "egg_z_after_lower": egg_z_after_lower,
        "egg_z_after_close": egg_z_after_close,
        "egg_z_after_lift": egg_z_after_lift,
        "success": success,
    }


def load_scene_and_test(
    scene_path: str = None,
    scene_xml: str = None,
    n_lower_steps: int = 250,
    n_close_steps: int = 400,
    n_lift_steps: int = 500,
    z_lower: float = -0.04,
    z_lift: float = 0.2,
    max_attempts: int = 5,
    verbose: bool = True,
) -> dict:
    """
    Load scene from path or XML; run grasp sequence; optionally iterate (z_lower, timings) until success.
    """
    if not MUJOCO_AVAILABLE:
        print("MuJoCo not available.")
        return {"success": False}

    if scene_path:
        path = Path(scene_path)
        if not path.exists():
            print(f"Scene not found: {scene_path}")
            return {"success": False}
        model = mujoco.MjModel.from_xml_path(str(path))
    elif scene_xml:
        model = mujoco.MjModel.from_xml_string(scene_xml)
    else:
        from core.scene_composer import SceneComposer
        composer = SceneComposer()
        composer.add_surface("table", [0, 0, 0])
        gripper_path = ROOT / "assets" / "grippers" / "test_gripper.mjcf"
        if not gripper_path.exists():
            gripper_path = ROOT.parent / "omega_foundry" / "outputs" / "test_gripper.xml"
        if gripper_path.exists():
            composer.add_gripper_from_file(str(gripper_path), pos=[0, 0, 0.10], mobile=True)
        else:
            print("No gripper file found.")
            return {"success": False}
        composer.add_object("egg", pos=[0, 0, 0.05], mass=0.08)
        scene_xml = composer.compose()
        model = mujoco.MjModel.from_xml_string(scene_xml)

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    for attempt in range(max_attempts):
        mujoco.mj_resetData(model, data)
        if verbose:
            print(f"\n--- Attempt {attempt + 1}/{max_attempts} ---")
        result = run_grasp_sequence(
            model,
            data,
            n_lower_steps=n_lower_steps,
            n_close_steps=n_close_steps,
            n_lift_steps=n_lift_steps,
            z_lower=z_lower,
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
            z_lower -= 0.01
            n_close_steps = min(500, n_close_steps + 50)
            if verbose:
                print(f"  Retry with z_lower={z_lower:.2f}, n_close_steps={n_close_steps}")

    result["attempt"] = max_attempts
    return result


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Test grasp: lower -> close -> lift egg")
    p.add_argument("--scene", type=str, default=None, help="Path to scene MJCF/XML")
    p.add_argument("--lower-steps", type=int, default=250)
    p.add_argument("--close-steps", type=int, default=400)
    p.add_argument("--lift-steps", type=int, default=500)
    p.add_argument("--z-lower", type=float, default=-0.05, help="Gripper z target for lower phase (palm to egg center ~0.05)")
    p.add_argument("--z-lift", type=float, default=0.2, help="Gripper z target for lift phase")
    p.add_argument("--max-attempts", type=int, default=5)
    p.add_argument("--quiet", action="store_true", help="Less output")
    args = p.parse_args()

    result = load_scene_and_test(
        scene_path=args.scene,
        n_lower_steps=args.lower_steps,
        n_close_steps=args.close_steps,
        n_lift_steps=args.lift_steps,
        z_lower=args.z_lower,
        z_lift=args.z_lift,
        max_attempts=args.max_attempts,
        verbose=not args.quiet,
    )
    print("\nResult:", "SUCCESS" if result.get("success") else "FAILED", "| egg_z_after_lift =", result.get("egg_z_after_lift", 0))
    sys.exit(0 if result.get("success") else 1)
