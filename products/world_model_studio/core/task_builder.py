"""
Task builder - Define manipulation tasks.
TaskDefinition: name, goal, target_object, success_criteria, max_steps, reward_type.
Built-in: PICK, PLACE, LIFT, PICK_EGG, PICK_BOX, HOLD.
compute_reward(task, obs, info) for sparse/dense.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Any, Optional

PICK = "pick"
PLACE = "place"
LIFT = "lift"
HANDOVER = "handover"
MOVE_TO = "move_to"
PICK_EGG = "pick_egg"
PICK_BOX = "pick_box"
HOLD = "hold"

_TASKS_ROOT = Path(__file__).resolve().parent.parent / "tasks"


@dataclass
class TaskDefinition:
    """Manipulation task definition."""
    name: str
    scene_path: str
    goal: str  # "pick", "place", "lift", "move_to", "handover", "pick_egg", "pick_box", "hold"
    target_object: str
    success_criteria: Dict[str, Any] = field(default_factory=dict)
    max_steps: int = 500
    reward_type: str = "sparse"  # "sparse" | "dense"


def compute_reward(task: TaskDefinition, obs: Dict[str, Any], info: Dict[str, Any]) -> float:
    """
    Compute reward from task and observation.
    sparse: 1.0 if success, 0.0 otherwise.
    dense: shaped reward (distance, grasp, height).
    """
    criteria = getattr(task, "success_criteria", {}) or {}
    reward_type = getattr(task, "reward_type", "sparse")
    qpos = obs.get("qpos")
    if qpos is None or len(qpos) < 3:
        return 0.0
    try:
        import numpy as np
        qpos = np.asarray(qpos)
    except Exception:
        return 0.0
    object_z = float(qpos[2])
    z_min = float(criteria.get("min_height") or criteria.get("object_z_min", 0.0))
    success = object_z >= z_min

    if reward_type == "sparse":
        return 1.0 if success else 0.0
    if reward_type == "dense":
        height_reward = min(1.0, object_z / max(z_min, 1e-6))
        dist_penalty = 0.0
        if "target_pos" in criteria:
            target = np.array(criteria["target_pos"], dtype=np.float64)
            dist = np.linalg.norm(qpos[:3] - target)
            dist_penalty = -0.1 * dist
        return height_reward + dist_penalty + (1.0 if success else 0.0)
    return 1.0 if success else 0.0


class TaskBuilder:
    """Build and save task definitions."""

    @staticmethod
    def pick_task(
        name: str,
        scene_path: str,
        target_object: str = "object_box",
        height_threshold: float = 0.1,
        max_steps: int = 500,
        reward_type: str = "sparse",
    ) -> TaskDefinition:
        """Lift object above height threshold."""
        return TaskDefinition(
            name=name,
            scene_path=scene_path,
            goal=PICK,
            target_object=target_object,
            success_criteria={"object_z_min": height_threshold, "min_height": height_threshold},
            max_steps=max_steps,
            reward_type=reward_type,
        )

    @staticmethod
    def pick_egg_task(scene_path: str, max_steps: int = 500, reward_type: str = "sparse") -> TaskDefinition:
        """Lift egg 5cm above table; success if egg_z > 0.1."""
        return TaskDefinition(
            name="pick_egg",
            scene_path=scene_path,
            goal=PICK_EGG,
            target_object="object_egg",
            success_criteria={"min_height": 0.1, "object_z_min": 0.1},
            max_steps=max_steps,
            reward_type=reward_type,
        )

    @staticmethod
    def pick_box_task(scene_path: str, max_steps: int = 500, reward_type: str = "sparse") -> TaskDefinition:
        """Lift box 5cm above table."""
        return TaskDefinition(
            name="pick_box",
            scene_path=scene_path,
            goal=PICK_BOX,
            target_object="object_box",
            success_criteria={"min_height": 0.1, "object_z_min": 0.1},
            max_steps=max_steps,
            reward_type=reward_type,
        )

    @staticmethod
    def hold_task(scene_path: str, hold_steps: int = 100, max_steps: int = 500, reward_type: str = "sparse") -> TaskDefinition:
        """Grasp and hold object for hold_steps without dropping."""
        return TaskDefinition(
            name="hold",
            scene_path=scene_path,
            goal=HOLD,
            target_object="object_box",
            success_criteria={"hold_steps": hold_steps, "min_height": 0.05},
            max_steps=max_steps,
            reward_type=reward_type,
        )

    @staticmethod
    def place_task(
        name: str,
        scene_path: str,
        target_object: str = "object_box",
        target_pos: Optional[list] = None,
        max_steps: int = 500,
        reward_type: str = "sparse",
    ) -> TaskDefinition:
        """Move object to target position."""
        return TaskDefinition(
            name=name,
            scene_path=scene_path,
            goal=PLACE,
            target_object=target_object,
            success_criteria={"target_pos": target_pos or [0.0, 0.0, 0.15], "tolerance": 0.05},
            max_steps=max_steps,
            reward_type=reward_type,
        )

    @staticmethod
    def handover_task(
        name: str,
        scene_path: str,
        target_object: str = "object_box",
        max_steps: int = 500,
        reward_type: str = "sparse",
    ) -> TaskDefinition:
        """Transfer object between two grippers."""
        return TaskDefinition(
            name=name,
            scene_path=scene_path,
            goal=HANDOVER,
            target_object=target_object,
            success_criteria={"handover_zone": [0.0, 0.0, 0.2]},
            max_steps=max_steps,
            reward_type=reward_type,
        )

    @staticmethod
    def save_task(task: TaskDefinition, filepath: Optional[str] = None) -> str:
        """Save task definition to JSON-like file. Returns path."""
        import json
        _TASKS_ROOT.mkdir(parents=True, exist_ok=True)
        path = Path(filepath) if filepath else _TASKS_ROOT / f"{task.name}.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "name": task.name,
            "scene_path": task.scene_path,
            "goal": task.goal,
            "target_object": task.target_object,
            "success_criteria": task.success_criteria,
            "max_steps": task.max_steps,
            "reward_type": task.reward_type,
        }
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        return str(path)

    @staticmethod
    def load_task(filepath: str) -> TaskDefinition:
        """Load task definition from JSON file."""
        import json
        path = Path(filepath)
        data = json.loads(path.read_text(encoding="utf-8"))
        return TaskDefinition(
            name=data.get("name", "unnamed"),
            scene_path=data["scene_path"],
            goal=data.get("goal", PICK),
            target_object=data.get("target_object", "object_box"),
            success_criteria=data.get("success_criteria", {}),
            max_steps=data.get("max_steps", 500),
            reward_type=data.get("reward_type", "sparse"),
        )
