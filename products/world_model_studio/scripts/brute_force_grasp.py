"""
Brute-force grasp: try combinations of gripper start Z, lower target Z, close steps
with box (size 0.03). Use large_gripper.xml from omega_foundry/outputs/.
Omega Max: diagnostics, longer phases, settle, smarter ordering, close-first variant, box friction.
On first success: save videos/successful_grasp.mp4 and exit.
"""

import re
import sys
import itertools
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from core import Simulator, SceneComposer
import imageio

try:
    import mujoco
    import numpy as np
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False

# Omega: more lower steps so Z actuator reaches target; settle steps after lower
LOWER_STEPS = 500
SETTLE_AFTER_LOWER = 80
LIFT_STEPS = 600
SUCCESS_BOX_Z = 0.15
LIFT_TARGET_GRIPPER_Z = 0.25


def get_box_z(sim) -> float:
    """Return object_box body Z from sim (model/data)."""
    if not MUJOCO_AVAILABLE or sim.model is None or sim.data is None:
        return 0.0
    try:
        bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "object_box")
        return float(sim.data.xpos[bid, 2])
    except Exception:
        return 0.0


def get_gripper_z(sim) -> float:
    """Gripper base Z (first slide joint = qpos[2] for mobile base)."""
    if sim._data is None or sim._model is None or sim._model.nq < 3:
        return 0.0
    return float(sim._data.qpos[2])


def build_action(z_ctrl: float, fingers_closed: bool = True) -> list:
    """Action: [x, y, z, f0_flex, f0_ext, f1_flex, f1_ext, f2_flex, f2_ext, f3_flex, f3_ext]."""
    close = [1, 0, 1, 0, 1, 0, 1, 0] if fingers_closed else [0] * 8
    return [0.0, 0.0, z_ctrl] + close


def _add_box_friction(mjcf: str) -> str:
    """Omega: add friction to box_geom to reduce slip during lift."""
    if "box_geom" not in mjcf:
        return mjcf
    idx = mjcf.find("box_geom")
    if "friction=" in mjcf[max(0, idx - 50) : idx + 250]:
        return mjcf
    # Insert friction before closing /> of box_geom tag (after mass="...")
    m = re.search(r'(<geom\s+name="box_geom"[^>]*mass="[^"]+")(\s*/>)', mjcf)
    if m:
        mjcf = mjcf.replace(m.group(0), m.group(1) + ' friction="1.2 0.005 0.0001"' + m.group(2), 1)
    return mjcf


def run_one_combo(
    gripper_start_z: float,
    lower_z: float,
    close_steps: int,
    gripper_path: Path,
    record_frames: bool,
    verbose_diagnostic: bool = False,
    close_first: bool = False,
) -> tuple:
    """
    Create scene, run lower -> close -> lift (or close_first variant). Return (success, frames).
    Omega: longer lower, settle after lower, optional close-first strategy, box friction.
    """
    composer = (
        SceneComposer()
        .add_surface("table", [0, 0, 0])
        .add_gripper_from_file(str(gripper_path), pos=[0, 0, gripper_start_z], mobile=True)
        .add_object("box", [0, 0, 0.05], mass=0.1)
        .set_camera()
    )
    mjcf = _add_box_friction(composer.compose())

    sim = Simulator()
    sim.load_scene(mjcf_string=mjcf)
    sim.reset()

    frames = [] if record_frames else None
    nu = sim.model.nu
    if nu < 7:
        return False, frames

    act_lower = build_action(lower_z)
    act_hold_z_start = build_action(0.0)  # hold joint at 0 so gripper stays at start Z (close-first)
    act_lift = build_action(0.3)
    _slice = lambda a: a[:nu] if len(a) > nu else a

    def maybe_frame():
        if record_frames:
            img = sim.render()
            if img is not None:
                frames.append(img)

    if close_first:
        # Omega: close fingers first (gripper still at start Z), then lower into box
        for i in range(150):
            sim.step(_slice(act_hold_z_start))
            if i % 4 == 0:
                maybe_frame()
        for i in range(LOWER_STEPS):
            sim.step(_slice(act_lower))
            if i % 4 == 0:
                maybe_frame()
        for i in range(close_steps):
            sim.step(_slice(act_lower))
            if i % 4 == 0:
                maybe_frame()
    else:
        # Phase 1: Lower (longer so Z reaches target)
        for i in range(LOWER_STEPS):
            sim.step(_slice(act_lower))
            if i % 4 == 0:
                maybe_frame()
        if verbose_diagnostic:
            print(f"    after lower: box_z={get_box_z(sim):.4f}, gripper_z={get_gripper_z(sim):.4f}")

        # Omega: settle (hold position, fingers still closing) so contacts stabilize
        for i in range(SETTLE_AFTER_LOWER):
            sim.step(_slice(act_lower))
            if i % 4 == 0:
                maybe_frame()

        # Phase 2: Close (hold position, fingers closed)
        for i in range(close_steps):
            sim.step(_slice(act_lower))
            if i % 4 == 0:
                maybe_frame()
        if verbose_diagnostic:
            print(f"    after close: box_z={get_box_z(sim):.4f}")

    # Phase 3: Lift
    for i in range(LIFT_STEPS):
        sim.step(_slice(act_lift))
        if i % 4 == 0:
            maybe_frame()
        obs = sim._get_obs()
        if obs.get("qpos") is not None and len(obs["qpos"]) > 2 and obs["qpos"][2] >= LIFT_TARGET_GRIPPER_Z:
            break
    if verbose_diagnostic:
        print(f"    after lift: box_z={get_box_z(sim):.4f} (success={get_box_z(sim) > SUCCESS_BOX_Z})")

    object_z = get_box_z(sim)
    success = object_z > SUCCESS_BOX_Z
    return success, frames


def main():
    if not MUJOCO_AVAILABLE:
        print("MuJoCo not available.")
        return

    gripper_path = PROJECT_ROOT.parent / "omega_foundry" / "outputs" / "large_gripper.xml"
    if not gripper_path.exists():
        print(f"Gripper not found: {gripper_path}")
        return

    gripper_start_z_list = [0.08, 0.10, 0.12, 0.14]
    lower_z_list = [-0.02, -0.04, -0.06]
    close_steps_list = [200, 400, 600]

    # Omega: try most promising first (middle params), then full grid
    def combo_priority(c):
        gz, lz, cs = c
        # Prefer gripper 0.10, lower -0.04, close 400
        score = (
            -abs(gz - 0.10) * 2 - abs(lz - (-0.04)) * 3 - abs(cs - 400) / 200
        )
        return score
    combos = list(itertools.product(gripper_start_z_list, lower_z_list, close_steps_list))
    combos.sort(key=combo_priority, reverse=True)

    # Omega: first run with diagnostic
    first = combos[0]
    print(
        f"Testing gripper_z={first[0]:.2f}, lower={first[1]:.2f}, close_steps={first[2]} [diagnostic]..."
    )
    run_one_combo(
        gripper_start_z=first[0],
        lower_z=first[1],
        close_steps=first[2],
        gripper_path=gripper_path,
        record_frames=False,
        verbose_diagnostic=True,
        close_first=False,
    )

    # Standard pass: lower -> close -> lift
    for gripper_z, lower_z, close_steps in combos:
        print(
            f"Testing gripper_z={gripper_z:.2f}, lower={lower_z:.2f}, "
            f"close_steps={close_steps}..."
        )
        success, frames = run_one_combo(
            gripper_start_z=gripper_z,
            lower_z=lower_z,
            close_steps=close_steps,
            gripper_path=gripper_path,
            record_frames=True,
            verbose_diagnostic=False,
            close_first=False,
        )
        if success:
            _save_success(gripper_z, lower_z, close_steps, frames, strategy="lower_close_lift")
            return

    # Omega: close-first pass (fingers close then lower; can cage box)
    print("Trying close-first strategy...")
    for gripper_z, lower_z, close_steps in combos[:12]:  # subset to avoid long run
        print(
            f"Testing [close-first] gripper_z={gripper_z:.2f}, lower={lower_z:.2f}, "
            f"close_steps={close_steps}..."
        )
        success, frames = run_one_combo(
            gripper_start_z=gripper_z,
            lower_z=lower_z,
            close_steps=close_steps,
            gripper_path=gripper_path,
            record_frames=True,
            verbose_diagnostic=False,
            close_first=True,
        )
        if success:
            _save_success(gripper_z, lower_z, close_steps, frames, strategy="close_first")
            return

    print("No successful combination found.")
    print("Tip: Check diagnostic output above; try increasing LOWER_STEPS or close_steps.")
    return


def _save_success(gripper_z: float, lower_z: float, close_steps: int, frames: list, strategy: str):
    out_path = PROJECT_ROOT / "videos" / "successful_grasp.mp4"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if frames:
        imageio.mimsave(str(out_path), frames, fps=30)
    print(f"Params: gripper_z={gripper_z}, lower_z={lower_z}, close_steps={close_steps} (strategy={strategy})")
    print(f"Saved {out_path}")
    print("SUCCESS!")


if __name__ == "__main__":
    main()
