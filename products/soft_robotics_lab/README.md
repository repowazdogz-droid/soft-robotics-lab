# OMEGA Soft Robotics Lab

**A Morphological Computation Toolkit for Soft Actuator Design**

## Overview

OMEGA Soft Robotics Lab is a hypothesis-driven design and simulation framework for soft robotic grippers. It bridges biological material science with robotic manufacturing through:

- **Embodied Intelligence**: Gripper geometries optimized to handle uncertainty through physical structure, not complex control
- **Validated Physics**: 100% MJCF validation with antagonistic tendon pairs and non-linear damping gradients
- **Self-Healing Digital Twin**: Calibration system that updates simulation parameters from physical experiment logs
- **Multi-Scale Modeling**: From crimp-angle collagen mechanics to macro-scale gripper behavior

## Key Features

### 1. Gripper Zoo
A library of 50+ validated gripper designs exploring:
- Pinch vs. wrap gestures
- Stiffness gradients (proximal → distal)
- Tendon routing optimization
- Contact surface geometry

### 2. Material Models
Pre-calibrated material profiles for:
- DragonSkin 10/20/30 (Smooth-On silicones)
- Ecoflex 00-30/00-50
- Custom TPU blends
- Hydrogel composites (experimental)

### 3. Sim-to-Real Bridge
The calibration module (`calibration.py`) enables:
- Physical test data import
- Automatic parameter refinement
- Confidence tracking per material/geometry pair
- Failure mode classification

### 4. Tactile Sensor Integration
Pre-allocated mounting sites for:
- TacTip-style optical tactile sensors
- Capacitive sensing arrays
- Strain gauge placement optimization

## Research Applications

This toolkit supports research in:
- Morphological computation
- Variable stiffness actuators
- Bio-hybrid robotics
- Grasping under uncertainty

## Hypothesis Ledger

Active research hypotheses are tracked with confidence intervals:
- `HYP-XXXXXXXX`: Hypothesis ID
- SRFC/VRFC validation status
- Simulated vs. physical trial results

## Installation

```bash
pip install -r requirements.txt
```

## Quick Start

```python
from soft_robotics_lab import GripperDesigner, Calibrator

# Design a gripper
designer = GripperDesigner()
gripper = designer.create(
    fingers=2,
    gesture="pinch",
    material="dragonskin_10",
    stiffness_gradient=True
)

# Validate physics
gripper.validate()

# Export for fabrication
gripper.export_stl("gripper_v1.stl")
```

*Note: For full design workflow use `workbench.motion_to_morphology.MotionToMorphology` and `workbench.gripper_cad.export_gripper_stl`. Material models: `materials.dragonskin`. Calibration: `calibration.Calibrator`.*

## Citation

If you use this toolkit in your research, please cite:

```
@software{omega_soft_robotics_lab,
  title={OMEGA Soft Robotics Lab: A Morphological Computation Toolkit},
  author={[Your Name]},
  year={2026},
  url={https://github.com/repowazdogz-droid/omega_stack}
}
```

## Contact

For collaboration inquiries: [your email]

## License

MIT License - See LICENSE file
