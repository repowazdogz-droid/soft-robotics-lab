"""
Diagnose grasp: check contacts, finger positions vs egg during close phase.
Prints: ncon, contact pairs involving egg, finger tip positions vs egg position.
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


def get_egg_pos(data, model):
    """Return egg body position [x,y,z]. Uses body name 'object_egg'."""
    try:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object_egg")
        return np.array(data.xpos[bid], dtype=np.float64)
    except Exception:
        return np.array([0.0, 0.0, 0.0])


def get_geom_name(model, geom_id):
    """Return geom name for geom_id, or None."""
    try:
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    except Exception:
        return None


def get_contacts_involving_egg(model, data):
    """Return list of (geom1_name, geom2_name, dist) for contacts involving egg_geom."""
    out = []
    ncon = getattr(data, "ncon", 0)
    if ncon <= 0 or not hasattr(data, "contact"):
        return out
    for i in range(ncon):
        c = data.contact[i]
        g1 = c.geom1
        g2 = c.geom2
        n1 = get_geom_name(model, g1)
        n2 = get_geom_name(model, g2)
        if n1 is None:
            n1 = f"geom_{g1}"
        if n2 is None:
            n2 = f"geom_{g2}"
        if "egg" in n1 or "egg" in n2:
            dist = getattr(c, "dist", 0.0)
            out.append((n1, n2, float(dist)))
    return out


def get_finger_tip_positions(model, data):
    """Return dict of site_name -> [x,y,z] for finger tip sites."""
    out = {}
    for site_name in ["finger_0_site_3", "finger_1_site_3", "finger_0_site_0", "finger_1_site_0"]:
        try:
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            out[site_name] = np.array(data.site_xpos[sid], dtype=np.float64)
        except Exception:
            pass
    return out


def load_scene(scene_path=None):
    """Load model and data from path or compose default scene."""
    if scene_path:
        path = Path(scene_path)
        if not path.exists():
            raise FileNotFoundError(scene_path)
        model = mujoco.MjModel.from_xml_path(str(path))
    else:
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
        composer.add_gripper_from_file(str(gripper_path), pos=[0, 0, 0.10], mobile=True)
        composer.add_object("egg", pos=[0, 0, 0.05], mass=0.08)
        model = mujoco.MjModel.from_xml_string(composer.compose())
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    return model, data


def run_diagnosis(scene_path=None, n_lower_steps=250, n_close_steps=400, sample_every=50):
    """
    Run lower -> close; during close phase sample every sample_every steps:
    ncon, contacts involving egg, finger tip positions vs egg position.
    """
    if not MUJOCO_AVAILABLE:
        print("MuJoCo not available.")
        return

    model, data = load_scene(scene_path)
    nu = model.nu
    if nu < 7:
        print(f"Only {nu} actuators; need 7. Skipping control.")
        nu = 0

    egg_pos = get_egg_pos(data, model)
    print("=== INITIAL ===")
    print(f"  Egg position: [{egg_pos[0]:.4f}, {egg_pos[1]:.4f}, {egg_pos[2]:.4f}]")
    tips = get_finger_tip_positions(model, data)
    for name, pos in tips.items():
        dist = np.linalg.norm(pos - egg_pos)
        print(f"  {name}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]  dist_to_egg={dist:.4f}")
    print(f"  data.ncon = {getattr(data, 'ncon', 0)}")
    egg_contacts = get_contacts_involving_egg(model, data)
    if egg_contacts:
        for g1, g2, d in egg_contacts:
            print(f"  Contact: {g1} <-> {g2}  dist={d:.4f}")
    else:
        print("  No contacts involving egg.")
    print()

    if nu >= 7:
        data.ctrl[:] = 0.0
        data.ctrl[2] = -0.05
        for _ in range(n_lower_steps):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)

    print("=== AFTER LOWER ===")
    egg_pos = get_egg_pos(data, model)
    print(f"  Egg position: [{egg_pos[0]:.4f}, {egg_pos[1]:.4f}, {egg_pos[2]:.4f}]")
    tips = get_finger_tip_positions(model, data)
    for name, pos in tips.items():
        dist = np.linalg.norm(pos - egg_pos)
        print(f"  {name}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]  dist_to_egg={dist:.4f}")
    print(f"  data.ncon = {getattr(data, 'ncon', 0)}")
    egg_contacts = get_contacts_involving_egg(model, data)
    if egg_contacts:
        for g1, g2, d in egg_contacts:
            print(f"  Contact: {g1} <-> {g2}  dist={d:.4f}")
    else:
        print("  No contacts involving egg.")
    print()

    if nu >= 7:
        data.ctrl[3] = 1.0
        data.ctrl[4] = 0.0
        data.ctrl[5] = 1.0
        data.ctrl[6] = 0.0

    print("=== DURING CLOSE (sampled every {} steps) ===".format(sample_every))
    for step in range(n_close_steps):
        mujoco.mj_step(model, data)
        if nu >= 7:
            mujoco.mj_forward(model, data)
        if (step + 1) % sample_every != 0 and step + 1 != n_close_steps:
            continue
        egg_pos = get_egg_pos(data, model)
        ncon = getattr(data, "ncon", 0)
        egg_contacts = get_contacts_involving_egg(model, data)
        tips = get_finger_tip_positions(model, data)
        print(f"  Step {step + 1}: egg_z={egg_pos[2]:.4f}  ncon={ncon}  egg_contacts={len(egg_contacts)}")
        if egg_contacts:
            for g1, g2, d in egg_contacts:
                print(f"    -> {g1} <-> {g2}  dist={d:.4f}")
        min_dist = float("inf")
        for name, pos in tips.items():
            dist = np.linalg.norm(pos - egg_pos)
            min_dist = min(min_dist, dist)
            if "site_3" in name:
                print(f"    {name} dist_to_egg={dist:.4f}")
        if min_dist > 0.02 and not egg_contacts:
            print(f"    [FINGERS MAY BE MISSING EGG - min tip dist={min_dist:.4f}]")
    print()

    print("=== SUMMARY ===")
    egg_pos = get_egg_pos(data, model)
    tips = get_finger_tip_positions(model, data)
    ncon = getattr(data, "ncon", 0)
    egg_contacts = get_contacts_involving_egg(model, data)
    print(f"  Egg position: [{egg_pos[0]:.4f}, {egg_pos[1]:.4f}, {egg_pos[2]:.4f}]")
    print(f"  Total contacts: {ncon}")
    print(f"  Contacts involving egg: {len(egg_contacts)}")
    if tips:
        for name in ["finger_0_site_3", "finger_1_site_3"]:
            if name in tips:
                pos = tips[name]
                dist = np.linalg.norm(pos - egg_pos)
                print(f"  {name} tip: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]  dist_to_egg={dist:.4f}")
    print("\n  FINDINGS:")
    if not egg_contacts and ncon > 0:
        print("  -> Egg is NOT in contact with gripper geoms. Other contacts present.")
        print("  -> Likely cause: finger spread or X/Y offset - fingers passing beside egg.")
        print("  -> FIX: Center egg under gripper (0,0); or lower gripper start Z so fingertips reach egg height.")
    elif not egg_contacts:
        print("  -> No contacts at all. Fingers may not reach egg (Z or geometry).")
        print("  -> FIX: Lower gripper default position (e.g. base Z=0.08); increase z_lower magnitude.")
    else:
        print("  -> Egg is in contact with gripper. If still no lift, increase friction or hold close longer.")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Diagnose grasp: contacts and positions during close")
    p.add_argument("--scene", type=str, default=None, help="Path to scene MJCF/XML")
    p.add_argument("--lower-steps", type=int, default=250)
    p.add_argument("--close-steps", type=int, default=400)
    p.add_argument("--sample-every", type=int, default=50, help="Print every N steps during close")
    args = p.parse_args()
    run_diagnosis(scene_path=args.scene, n_lower_steps=args.lower_steps, n_close_steps=args.close_steps, sample_every=args.sample_every)
