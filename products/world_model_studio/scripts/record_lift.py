import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from core import Simulator
import imageio

sim = Simulator()
sim.load_scene((PROJECT_ROOT / "scenes" / "pick_box2.xml").read_text())
sim.reset()

frames = []

# Phase 1: Lower and close (300 steps)
for i in range(300):
    sim.step([0, 0, -0.05, 1, 0, 1, 0])
    if i % 4 == 0:
        frames.append(sim.render())

# Phase 2: Lift while gripping (500 steps)
for i in range(500):
    obs, _, _, _ = sim.step([0, 0, 0.3, 1, 0, 1, 0])
    if i % 4 == 0:
        frames.append(sim.render())

print(f"Gripper Z: {obs['qpos'][2]:.4f}, Box Z: {obs['qpos'][13]:.4f}")

out_path = PROJECT_ROOT / "videos" / "full_lift.mp4"
imageio.mimsave(out_path, frames, fps=30)
print(f"Saved {out_path}")
