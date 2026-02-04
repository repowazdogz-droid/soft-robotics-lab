# 🌍 World Model Studio

**Train Manipulation Policies in Simulation**

Scene composition, task building, policy training, batch jobs.

---

## 🚀 Quick Start

```bash
cd products/world_model_studio
pip install -r requirements.txt
streamlit run app.py --server.port 8505
```

Open http://localhost:8505 in your browser.

---

## 💡 What Problem Does This Solve?

Training robot policies requires:
- Scene setup (tedious)
- Task definition (manual)
- Training infrastructure (complex)
- Sim-to-real tracking (forgotten)

| Problem | Solution |
|---------|----------|
| Building scenes from scratch | Scene composer with object library |
| Defining tasks manually | Task builder with templates |
| Training infrastructure | Built-in trainer with logging |
| No batch processing | Job queue for overnight runs |
| Sim-to-real gap unknown | Reality gap tracking |

---

## 📦 Features

### 1. Scene Composer
Build simulation environments visually:

**Object Library:**
- Primitives: box, sphere, cylinder, capsule
- Household: cup, bottle, bowl, plate
- Industrial: bin, conveyor, shelf
- Custom: import your own meshes

**Scene Elements:**
- Ground plane with configurable friction
- Lighting conditions
- Camera positions for observation
- Randomization ranges

```python
scene = {
    "objects": [
        {"type": "table", "position": [0, 0, 0]},
        {"type": "cup", "position": [0.2, 0, 0.8], "randomize": True},
        {"type": "gripper", "mount": "arm_ee"}
    ],
    "randomization": {
        "object_position": 0.05,  # ±5cm
        "object_rotation": 15,    # ±15°
        "lighting": 0.2           # ±20% intensity
    }
}
```

### 2. Task Builder
Define manipulation tasks:

| Task Type | Description |
|-----------|-------------|
| **Pick** | Grasp object, lift to height |
| **Place** | Move object to target position |
| **Pick-Place** | Combined pick and place |
| **Insert** | Insert peg into hole |
| **Pour** | Tilt container, pour contents |
| **Handover** | Transfer to another gripper/hand |

**Task Definition:**
```python
task = {
    "name": "pick_cup",
    "type": "pick",
    "target_object": "cup",
    "success_criteria": {
        "lift_height": 0.1,       # 10cm lift
        "grasp_stability": 0.95,  # 95% contact maintained
        "time_limit": 5.0         # seconds
    },
    "reward": {
        "grasp": 1.0,
        "lift": 2.0,
        "time_bonus": 0.5
    }
}
```

### 3. Policy Trainer
Train policies with standard RL algorithms:

**Supported Algorithms:**
- PPO (Proximal Policy Optimization)
- SAC (Soft Actor-Critic)
- TD3 (Twin Delayed DDPG)

**Training Config:**
```python
config = {
    "algorithm": "PPO",
    "num_envs": 16,
    "total_timesteps": 1_000_000,
    "learning_rate": 3e-4,
    "batch_size": 256,
    "checkpoint_freq": 10000
}
```

**Logging:**
- TensorBoard integration
- Episode rewards
- Success rate
- Policy entropy
- Value loss

### 4. Batch Jobs
Queue training runs:

```
Job Queue:
├── [RUNNING] pick_cup_v1 - 45% complete
├── [QUEUED] pick_cup_v2 - different reward
├── [QUEUED] pick_bottle - same policy, new object
└── [COMPLETED] reach_target - 89% success rate
```

**Features:**
- Priority queue
- Overnight runs
- Email/Slack notifications
- Auto-checkpoint

### 5. Reality Gap Tracking
Measure sim-to-real gap:

| Metric | Sim | Real | Gap |
|--------|-----|------|-----|
| Success Rate | 92% | 78% | 14% |
| Grasp Stability | 0.95 | 0.82 | 0.13 |
| Execution Time | 2.1s | 2.8s | 0.7s |

**Gap Reduction:**
- Domain randomization
- System identification
- Fine-tuning on real data

---

## 🏗️ Architecture

```
world_model_studio/
├── app.py                      # Streamlit UI
├── core/
│   ├── scene_composer.py       # Build scenes
│   ├── task_builder.py         # Define tasks
│   ├── simulator.py            # MuJoCo wrapper
│   ├── trainer.py              # RL training
│   └── batch.py                # Job queue
├── assets/
│   ├── objects/                # Object meshes
│   └── textures/
├── policies/                   # Trained models
└── logs/                       # Training logs
```

---

## 🔄 Training Flow

```
┌─────────────────────────────────────────────────────────────────┐
│   1. Compose scene (objects, randomization)                     │
│   2. Define task (success criteria, reward)                     │
│   3. Configure training (algorithm, hyperparams)                │
│   4. Launch training (local or batch)                           │
│   5. Monitor (TensorBoard, success rate)                        │
│   6. Deploy policy (export for robot)                           │
│   7. Track reality gap (sim vs real)                            │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Task Templates

### Pick Task
```python
pick_task = create_task(
    type="pick",
    object="cube",
    success_height=0.1,
    grasp_threshold=0.9
)
```

### Place Task
```python
place_task = create_task(
    type="place",
    object="cube",
    target_position=[0.3, 0.0, 0.05],
    position_tolerance=0.02
)
```

### Insert Task
```python
insert_task = create_task(
    type="insert",
    peg="cylinder",
    hole="socket",
    insertion_depth=0.05,
    alignment_tolerance=0.005
)
```

---

## 🔌 Integration

### With OMEGA Foundry
```python
# Design generates gripper
mjcf = foundry.generate(intent="2-finger gripper for cubes")

# Add to scene
scene.add_gripper(mjcf, mount="arm_ee")

# Train policy
policy = trainer.train(scene, task)
```

### With Reality Bridge
```python
# Validate gripper before training
result = reality_bridge.validate(mjcf)

if result.passed:
    # Safe to train
    policy = trainer.train(scene, task)
```

---

## 🧪 Example Workflow

```python
from core.scene_composer import SceneComposer
from core.task_builder import TaskBuilder
from core.trainer import PolicyTrainer

# 1. Build scene
composer = SceneComposer()
scene = composer.create()
scene.add("table", position=[0, 0, 0])
scene.add("cup", position=[0.2, 0, 0.8], randomize=True)
scene.add("gripper", mount="arm_ee")

# 2. Define task
builder = TaskBuilder()
task = builder.create_pick_task(
    target="cup",
    lift_height=0.15,
    time_limit=5.0
)

# 3. Train policy
trainer = PolicyTrainer()
policy = trainer.train(
    scene=scene,
    task=task,
    algorithm="PPO",
    timesteps=500_000
)

# 4. Evaluate
success_rate = trainer.evaluate(policy, scene, task, episodes=100)
print(f"Success rate: {success_rate:.0%}")

# 5. Export
trainer.export(policy, "pick_cup_policy.onnx")
```

**Note:** The API above is conceptual. In this codebase use:

- **Scene:** `SceneComposer().add_surface()`, `.add_gripper()`, `.add_object()`, `.compose()`, `.save()`
- **Task:** `TaskBuilder.pick_task()`, `.pick_and_place_task()`, `.place_task()`, etc., and `TaskDefinition`
- **Training:** `train_loop(sim, task, method=..., config=...)` from `core.trainer`; `evaluate_policy()` for evaluation
- **Export:** Save policy weights to JSON/ONNX from the app or scripts

---

## 📁 Output Formats

| Format | Use |
|--------|-----|
| **ONNX** | Cross-platform deployment |
| **TorchScript** | PyTorch inference |
| **TensorRT** | NVIDIA optimization |
| **Checkpoint** | Resume training |

---

## 🎯 Use Cases

1. **Policy development**: Train manipulation skills in simulation
2. **Design validation**: Test gripper designs with trained policies
3. **Curriculum learning**: Progressive task difficulty
4. **Batch experiments**: Hyperparameter sweeps overnight
5. **Sim-to-real**: Track and reduce reality gap

---

## 📋 Requirements

```
streamlit>=1.28.0
mujoco>=3.0.0
stable-baselines3>=2.0.0
torch>=2.0.0
tensorboard>=2.14.0
```

*(Optional: stable-baselines3, torch, tensorboard for PPO/SAC/TD3. The app runs with the core requirements in `requirements.txt`.)*

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"Train in simulation. Deploy in reality."*
