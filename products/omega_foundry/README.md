# 🏭 OMEGA Foundry

**Soft Robot Design from Intent**

Describe what you want. Get a physics-ready design.

---

## 🚀 Quick Start

```bash
cd products/omega_foundry
pip install -r requirements.txt
streamlit run app.py --server.port 8504
```

Open http://localhost:8504 in your browser.

---

## 💡 What Problem Does This Solve?

Designing soft robots requires deep expertise. The gap from idea to MJCF is huge.

| Problem | Solution |
|---------|----------|
| "I need a gripper for eggs" | Intent parsing → design parameters |
| Complex parametric design | Templates + constraint satisfaction |
| Manual MJCF writing | Auto-generation from specs |
| No physics validation | Reality Bridge integration |
| Design iteration by hand | Evolutionary refinement from failures |

---

## 📦 Features

### 1. Intent Parsing
Natural language to design parameters:

```
Input: "A soft gripper with 3 fingers for picking delicate fruit"

Output:
├── domain: gripper
├── scale: medium
├── target_object: fruit
├── params: num_fingers=3, stiffness=low, ...
└── raw_intent: ...
```

### 2. Design Templates
Pre-validated base designs:

| Template | Use Case |
|----------|----------|
| **Two-finger pinch** | Small, precise objects |
| **Soft pneumatic** | Compliant, delicate grasp |
| **Tendon-driven** | Precise force control |
| **Three-finger adaptive** | Irregular objects |
| **Gecko-inspired** | Smooth surfaces |
| **Mechanisms** | Hinge, linkage, cam, gear |
| **Enclosures** | Housing, bracket, mount |

Each template includes:
- Parameterizable dimensions
- Actuator configurations
- MJCF or URDF generation

### 3. MJCF Generation
Physics-ready simulation files:

```xml
<mujoco model="soft_gripper_v1">
  <compiler autolimits="true"/>
  <option solver="Newton" iterations="50"/>

  <worldbody>
    <body name="palm">
      <!-- Generated geometry -->
    </body>
    <body name="finger_1">
      <!-- Stiffness gradient: base → tip -->
      <!-- Tendon routing -->
      <!-- Contact surfaces -->
    </body>
  </worldbody>

  <tendon>
    <!-- Antagonistic pairs -->
  </tendon>

  <actuator>
    <!-- Position/velocity/torque control -->
  </actuator>
</mujoco>
```

### 4. Constraint Satisfaction
Hard constraints enforced:

```
Constraints:
├── max_size: 100mm (fits in target space)
├── max_mass: 200g (robot payload limit)
├── min_grip_force: 10N (object weight)
├── max_pressure: 150kPa (safety)
└── environment: sterilizable
```

If constraints conflict → reports "impossible" with explanation.

### 5. Design Evolution
When Reality Bridge reports failures:

```
v1 → Reality Bridge → FAIL: finger too short
v2 → (finger length +20%) → Reality Bridge → FAIL: collision
v3 → (spread angle +15°) → Reality Bridge → PASS
```

Tracks design lineage (v1 → v2 → v3) with changes at each step.

### 6. Voice Design
Speak your intent:

```
"Make me a gripper that can pick up a wine glass
without breaking it, and it needs to fit on a UR5"

→ Parses: delicate object, cylindrical, size constraint
→ Suggests: 4-finger wrap gripper, low stiffness, 80mm span
```

---

## 🏗️ Architecture

```
omega_foundry/
├── app.py                      # Streamlit UI
├── core/
│   ├── intent_parser.py        # NL → DesignSpec
│   ├── design_engine.py       # DesignSpec → GeneratedDesign
│   ├── template_loader.py     # Load template JSONs
│   ├── primitives/
│   │   ├── grippers.py        # GripperGenerator, MJCF
│   │   ├── mechanisms.py     # MechanismGenerator
│   │   └── enclosures.py     # EnclosureGenerator, URDF/STL
│   ├── preview.py             # MJCF → 3D preview
│   ├── exporter.py            # Export MJCF/URDF/STL/JSON
│   ├── validator.py          # Local physics check
│   ├── voice_design.py        # Voice → intent → design
│   └── history.py             # Version / restore
├── templates/
│   ├── grippers/              # soft_pneumatic.json, tendon_driven.json, ...
│   ├── mechanisms/            # four_bar_linkage.json, ...
│   └── enclosures/            # vented_housing.json, ...
└── outputs/                   # Generated designs (MJCF, STL, URDF)
```

---

## 🔄 Design Flow

```
┌─────────────────────────────────────────────────────────────────┐
│   1. Intent (natural language or structured)                    │
│   2. Parse → DesignSpec                                          │
│   3. Select template (or auto from domain)                       │
│   4. Generate design → MJCF / URDF / STL                         │
│   5. Validate in Reality Bridge                                  │
│   6. If fail → evolve design → goto 4                            │
│   7. Export for simulation/fabrication                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Design Parameters

| Category | Parameters |
|----------|------------|
| **Geometry** | num_fingers, finger_length, finger_width, palm_size, spread_angle |
| **Actuation** | type (pneumatic/tendon/SMA), max_force, max_pressure, response_time |
| **Materials** | stiffness, damping, density, friction |
| **Sensors** | joint_position, joint_velocity, contact, tendon_tension |
| **Environment** | food_safe, sterilizable, waterproof, high_temp |

---

## 🔌 Integration

### With Reality Bridge
```python
# Generate design (omega_foundry)
from core.intent_parser import IntentParser
from core.design_engine import DesignEngine

parser = IntentParser()
spec = parser.parse("3-finger gripper for eggs")
engine = DesignEngine()
result = engine.generate(spec)
mjcf = result.mjcf_xml

# Validate (Reality Bridge API or local PhysicsValidator)
# POST http://localhost:8000/validate with mjcf
# If not passed: POST /fixes with validation_result, then re-generate
```

### With Hypothesis Ledger
```python
# Design supports a hypothesis (from Breakthrough Engine)
ledger.add_evidence(
    hypothesis_id,
    f"Design {design_id} passed validation with score {result.score}",
    "supports",
    source="reality_bridge"
)
```

---

## 🧪 Example Workflow

```python
from core.intent_parser import IntentParser, DesignSpec
from core.design_engine import DesignEngine, GeneratedDesign

# 1. Parse intent
parser = IntentParser()
spec = parser.parse("A soft gripper for picking strawberries")
# → spec.domain="gripper", spec.params, spec.target_object, ...

# 2. Generate design
engine = DesignEngine()
result = engine.generate(spec)  # → GeneratedDesign

# 3. Get MJCF
mjcf = result.mjcf_xml
if mjcf:
    with open("strawberry_gripper.mjcf", "w") as f:
        f.write(mjcf)

# 4. Optional: export URDF/STL via DesignExporter
from core.exporter import DesignExporter
exporter = DesignExporter()
paths = exporter.export(result, output_dir="outputs")
# → paths["mjcf"], paths["stl"], paths["urdf"], ...
```

---

## 📁 Output Formats

| Format | Use |
|--------|-----|
| **MJCF** | MuJoCo simulation |
| **URDF** | ROS integration (enclosures/mechanisms) |
| **STL** | 3D printing |
| **JSON** | Design parameters |

---

## 🎯 Use Cases

1. **Rapid prototyping**: Idea → simulation in minutes
2. **Design exploration**: Generate variants, compare in Reality Bridge
3. **Lab demos**: Quick grippers for specific tasks
4. **Teaching**: Show how parameters affect design
5. **Grant proposals**: Visualize proposed designs

---

## 📋 Requirements

```
streamlit>=1.28.0
numpy>=1.24.0
trimesh>=3.23.0
```

Optional for voice:
- `whisper` for speech-to-text
- `sounddevice` for microphone input

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"Describe what you want. Get a design that works."*
