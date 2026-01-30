---
title: Variable Stiffness Gripper Test Results - January 2026
date: 2026-01-29 09:41
tags: [soft-robotics, gripper, experiment]
---

# Variable Stiffness Gripper Test Results - January 2026

## Summary

Tested the 4-finger pneumatic gripper (GD-20260128-0015) on a range of objects with varying compliance. Key finding: variable stiffness mechanism significantly improves grasp success on deformable objects.

## Test Setup

- Gripper: 4-finger pneumatic, silicone body
- Objects: egg, tomato, foam ball, rigid cube
- Environment: dry, room temperature
- Trials: 10 per object

## Results

| Object | Success Rate | Notes |
|--------|-------------|-------|
| Egg | 90% | One slip at high speed |
| Tomato | 100% | No damage observed |
| Foam ball | 80% | Difficult to detect contact |
| Rigid cube | 70% | Edges caused issues |

## Key Observations

1. **Compliance matching matters** - Best results when gripper stiffness matched object compliance
2. **Speed affects success** - Slower approach = higher success rate
3. **Sensor feedback needed** - Contact detection was unreliable

## Next Steps

- Test tendon-driven alternative
- Add tactile sensors
- Try wet environment
```

---

After saving, create a hypothesis to go with it:

**Claim:**
```
Variable stiffness grippers achieve >90% success rate on deformable objects when stiffness is matched to object compliance
```

**Scope:**
```
Deformable objects 20-100g, dry environment, approach speed <50mm/s
