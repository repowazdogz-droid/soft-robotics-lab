# Soft Robotics Lab Toolkit

**OMEGA Research Platform — Gripper Design, Analysis & Simulation**

A complete toolkit for designing, analyzing, and simulating soft robotic grippers with physics-validated simulation files.

---

## Quick Start

```bash
# Install dependencies
pip install streamlit plotly trimesh mujoco numpy

# Run the app
streamlit run app.py
```

Open http://localhost:8501 in your browser.

---

## What's Included

### 1. Gripper Design Studio
Transform grasp gestures into optimized gripper morphologies:
- **7 gesture types**: pinch, power, wrap, hook, lateral, squeeze, spread
- **3D interactive preview**
- **STL export** for 3D printing
- **Confidence scoring** with uncertainty quantification

### 2. Failure Analysis
Predict and mitigate failure modes:
- **Visual risk matrix** (probability × severity)
- **Top failure modes**: slip, fatigue, damage, etc.
- **Mitigation recommendations**

### 3. Gripper Zoo (50 Designs)
Pre-generated, simulation-ready grippers:
- All validated in MuJoCo (50/50 pass)
- Multiple formats: **JSON, MJCF, URDF, USD**
- Covers all gestures, environments, actuator types

### 4. Research Memory
Track your lab's knowledge:
- **Notes with tags**
- **Hypothesis tracking** with confidence over time
- **Visual dashboard**
- **Weekly briefs** for lab meetings

---

## Physics Features

The MJCF simulation files include production-quality physics:

- **Stiffness gradient** — Base to tip decreasing for compliant fingertips
- **Damping gradient** — Matched to stiffness for responsive motion
- **Antagonistic tendons** — Flexor + extensor pairs per finger
- **Tendon frictionloss** — Simulates cable routing friction
- **Friction pads** — Flat pads on fingertips for rigid objects
- **Newton solver** — Stable soft-body simulation
- **Full sensors** — Joint pos/vel, tendon pos/vel

---

## Structure

```
soft_robotics_lab/
├── app.py                      # Main Streamlit app
├── pages/
│   ├── 1_Gripper_Design.py     # Design studio
│   ├── 2_Failure_Analysis.py   # Risk analysis
│   ├── 3_Compare_Designs.py    # Side-by-side comparison
│   ├── 4_Gripper_Zoo.py        # Browse 50 designs
│   └── 5_Research_Memory.py    # Notes, hypotheses & briefs
├── workbench/
│   ├── motion_to_morphology.py # Core design engine
│   ├── failure_predictor.py    # Failure mode analysis
│   ├── gripper_cad.py          # STL generation
│   ├── mjcf_generator.py      # Physics-correct MJCF
│   ├── mujoco_validator.py     # Validation suite
│   └── calibration.py          # Learning from experiments
├── gripper_zoo/
│   └── designs/                # 50 pre-generated grippers
└── research_system/
    └── research_memory.py      # Knowledge base
```

---

## Usage Examples

### Generate a new gripper

```python
from workbench.motion_to_morphology import MotionToMorphology, export_design

m2m = MotionToMorphology()
design = m2m.from_gesture(
    gesture="pinch",
    aperture=0.05,         # 50mm
    force_requirement=3.0, # 3N
    object_compliance=0.3, # Semi-rigid
    environment="dry"
)

export_design(design, "my_gripper/")
```

### Run MuJoCo simulation

```python
import mujoco

model = mujoco.MjModel.from_xml_path(
    "gripper_zoo/designs/gd_20260128_0001/gd_20260128_0001.mjcf"
)
data = mujoco.MjData(model)

# Close gripper (activate flexor tendons)
data.ctrl[0] = 1.0  # finger_0_flexor
data.ctrl[2] = 1.0  # finger_1_flexor

for _ in range(1000):
    mujoco.mj_step(model, data)
```

### Generate STL for 3D printing

```python
from workbench.gripper_cad import export_gripper_stl

export_gripper_stl(design.to_dict(), "my_gripper.stl")
```

### Validate MJCF files

```bash
python workbench/mujoco_validator.py --zoo
```

### Log experiment results

```bash
python workbench/calibration.py log --design GD-20260128-0001 --success --task egg_handling
python workbench/calibration.py stats
```

---

## Validation Results

```
MUJOCO VALIDATION REPORT
========================
Total designs:  50
Valid:          50 (100%)
Loads:          50
Simulates:      50
Errors:         0
```

---

## Requirements

- Python 3.10+
- streamlit
- plotly
- trimesh
- mujoco
- numpy

---

## License

Research use permitted. Contact for commercial licensing.
