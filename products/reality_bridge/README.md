# 🌉 Reality Bridge

**Physics Validation for Soft Robot Designs**

Validate designs in MuJoCo before building. Prescriptive fixes when things fail. Compare designs head-to-head.

---

## 🚀 Quick Start

```bash
cd products/reality_bridge
pip install -r requirements.txt

# Run API server
uvicorn app:app --host 0.0.0.0 --port 8000

# Run dashboard
streamlit run dashboard.py --server.port 8501
```

API: http://localhost:8000/docs
Dashboard: http://localhost:8501

---

## 💡 What Problem Does This Solve?

Designs fail in reality because they weren't validated in physics.

| Problem | Solution |
|---------|----------|
| Design looks good on paper | Validate in MuJoCo simulation |
| Failure with no explanation | Prescriptive fixes ("change X to Y") |
| Which design is better? | Head-to-head comparison |
| No audit trail | Full validation bundles |

---

## 📦 Features

### 1. Physics Validation
Submit MJCF designs, get physics validation:

| Test | What It Checks |
|------|----------------|
| **Stability** | Will it tip over? Center of mass issues? |
| **Kinematics** | Joint ranges, singularities, workspace |
| **Dynamics** | Oscillation, response time, damping |
| **Self-Collision** | Do parts collide with each other? |
| **Mass Properties** | Too heavy? Asymmetric inertia? |

**Result**: Pass/fail per test + overall score + detailed metrics.

### 2. Prescriptive Fixer
Not just "this failed" but "change X to Y":

```
❌ STABILITY_FAIL
   → Increase base width: 0.05m → 0.065m
   → Confidence: 80%
   → Reasoning: Wider base improves stability against tip-over

❌ DYNAMICS_FAIL (oscillation)
   → Increase damping: 0.1 → 0.2 N·m·s/rad
   → Confidence: 90%
   → Reasoning: Increased damping reduces oscillation
```

Outputs:
- Prioritized fixes (🔴 high, 🟡 medium, 🟢 low)
- MJCF modification suggestions
- Estimated improvement if fixes applied

### 3. Design Comparator
Compare two designs for a specific task:

| Metric | Design A | Design B | Winner |
|--------|----------|----------|--------|
| Stability | 85% | 72% | A |
| Kinematics | 68% | 91% | B |
| Dynamics | 77% | 80% | B |
| Grip Force | 90% | 65% | A |

**Task-specific weighting:**
- `pick_egg`: Lower grip force weight, higher dynamics weight
- `heavy_lift`: Higher grip force weight, higher stability weight
- `surgical`: Higher kinematics and robustness weight

**Output**: Winner, confidence, strengths, recommendation.

### 4. Audit Bundles
Every validation produces:
- Input design (MJCF)
- All test results
- Metrics and scores
- Timestamp and version
- Reproducible bundle for paper/grant

---

## 🏗️ Architecture

```
reality_bridge/
├── app.py                      # FastAPI server
├── dashboard.py                # Streamlit UI
├── core/
│   ├── validator.py            # MuJoCo validation engine
│   ├── loader.py               # Load MJCF/URDF
│   ├── reporter.py             # Generate reports
│   ├── prescriptive_fixer.py   # "Change X to Y" fixes
│   ├── design_comparator.py    # A vs B comparison
│   ├── analyzer.py             # Weak points, failure modes
│   ├── fix_suggestions.py      # Failure → suggestions
│   ├── database.py             # Validation history
│   └── webhooks.py             # Event webhooks
├── tests/
│   └── test_validator.py
└── data/
    └── (validation DB)
```

---

## 🔌 API Endpoints

### Validate Design
```http
POST /validate
Content-Type: multipart/form-data  (or application/json with xml_string)

# Option 1: File upload
file: <MJCF/XML file>

# Option 2: JSON body
{"xml_string": "<mujoco>...</mujoco>"}
```

Response:
```json
{
  "success": true,
  "passed": false,
  "score": 0.72,
  "tests": {
    "STABILITY_TEST": {"passed": true, "message": "..."},
    "KINEMATICS_TEST": {"passed": false, "message": "..."}
  },
  "errors": [],
  "validation_time_ms": 45
}
```

### Generate Fixes
```http
POST /fixes
Content-Type: application/json

{
  "validation_result": { ... },
  "design_id": "gripper_v1"
}
```

Response:
```json
{
  "success": true,
  "design_id": "gripper_v1",
  "fixes": [
    {
      "component": "joint_2",
      "parameter": "range",
      "current_value": 1.57,
      "suggested_value": 2.35,
      "confidence": 0.85,
      "reasoning": "Increased joint range allows fuller motion"
    }
  ],
  "estimated_improvement": 0.25
}
```

### Compare Designs
```http
POST /compare
Content-Type: application/json

{
  "design_a": {"mjcf": "...", "id": "v1"},
  "design_b": {"mjcf": "...", "id": "v2"},
  "task": "pick_egg"
}
```

Response:
```json
{
  "success": true,
  "overall_winner": "A",
  "confidence": 0.78,
  "design_a_strengths": ["grip_force", "stability"],
  "design_b_strengths": ["speed", "kinematics"],
  "recommendation": "Design A recommended for pick_egg. Key advantages: grip_force, stability."
}
```

---

## 🔄 Integration with OMEGA Stack

### From OMEGA Foundry
```python
# Design generated by Foundry
mjcf = foundry.generate_mjcf(intent)

# Validate in Reality Bridge
result = validator.validate(xml_string=mjcf)

if not result.passed:
    fixes = generate_prescriptive_fixes(to_dict(result), design_id="my_design")
    # Apply fixes and re-validate
```

### To Hypothesis Ledger
```python
# Validation informs SRFC status
if result.passed and result.score > 0.8:
    ledger.update_fields(hypothesis_id, srfc_status="GREEN")
elif result.passed:
    ledger.update_fields(hypothesis_id, srfc_status="AMBER")
else:
    ledger.update_fields(hypothesis_id, srfc_status="RED")
```

---

## 📊 Fix Types

| Type | Examples |
|------|----------|
| `PARAMETER_CHANGE` | Joint range, damping, gain |
| `GEOMETRY_CHANGE` | Length, width, spread angle |
| `MATERIAL_CHANGE` | Density, friction coefficient |
| `ACTUATOR_CHANGE` | Max force, response time |
| `STRUCTURAL_CHANGE` | Symmetry, topology |

---

## 🧪 Example Workflow

```python
from core.validator import PhysicsValidator
from core.reporter import to_dict
from core.prescriptive_fixer import generate_prescriptive_fixes
from core.design_comparator import compare_designs

# 1. Validate a design
validator = PhysicsValidator()
result = validator.validate(xml_string=mjcf)

print(f"Passed: {result.passed}, Score: {result.score}")

# 2. Get fixes if failed (use to_dict for API-shaped input)
if not result.passed:
    fix_report = generate_prescriptive_fixes(
        to_dict(result),
        design_id="my_gripper"
    )
    for fix in fix_report.fixes:
        print(f"Fix: {fix.component}.{fix.parameter}: {fix.current_value} → {fix.suggested_value}")

# 3. Compare two designs
result_v1 = validator.validate(xml_string=mjcf_v1)
result_v2 = validator.validate(xml_string=mjcf_v2)
comparison = compare_designs(
    design_a={"id": "v1", "mjcf": mjcf_v1},
    design_b={"id": "v2", "mjcf": mjcf_v2},
    validation_a=to_dict(result_v1),
    validation_b=to_dict(result_v2),
    task="heavy_lift"
)
print(f"Winner: {comparison.overall_winner} ({comparison.confidence:.0%})")
```

---

## 📋 Requirements

```
fastapi>=0.104.0
uvicorn>=0.24.0
mujoco>=3.0.0
streamlit>=1.28.0
numpy>=1.24.0
```

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"Validate in simulation. Fix before you build. Compare before you commit."*
