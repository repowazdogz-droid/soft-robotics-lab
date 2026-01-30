"""
Policies - LinearPolicy, ConstantPolicy, GraspSequencePolicy.
"""

from typing import Any, Dict, List, Optional

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False


class LinearPolicy:
    """Simple linear mapping: action = clip(weights @ obs_vec, -1, 1)."""

    def __init__(self, obs_dim: int, act_dim: int, weights: Optional[Any] = None):
        self.obs_dim = obs_dim
        self.act_dim = act_dim
        if weights is not None:
            self.weights = np.asarray(weights, dtype=np.float64)
            if self.weights.shape != (act_dim, obs_dim):
                self.weights = np.reshape(self.weights, (act_dim, obs_dim))
        else:
            self.weights = np.zeros((act_dim, obs_dim))

    def __call__(self, obs: Dict[str, Any]) -> List[float]:
        if not NUMPY_AVAILABLE:
            return [0.0] * self.act_dim
        qpos = obs.get("qpos", np.array([]))
        qvel = obs.get("qvel", np.array([]))
        vec = np.concatenate([np.asarray(qpos).ravel(), np.asarray(qvel).ravel()])
        if len(vec) < self.obs_dim:
            vec = np.pad(vec, (0, max(0, self.obs_dim - len(vec))))
        vec = vec[: self.obs_dim]
        act = self.weights @ vec
        act = np.clip(act, -1.0, 1.0)
        return act.tolist()

    def get_weights(self) -> Any:
        return self.weights.copy()

    def set_weights(self, w: Any) -> None:
        self.weights = np.asarray(w, dtype=np.float64).reshape(self.act_dim, self.obs_dim)


class ConstantPolicy:
    """Always returns the same actions (for testing)."""

    def __init__(self, actions: List[float]):
        self.actions = list(actions)

    def __call__(self, obs: Dict[str, Any]) -> List[float]:
        return self.actions.copy()


class GraspSequencePolicy:
    """Hardcoded grasp sequence: close gripper then lift (increase z control)."""

    def __init__(self, nu: int = 0, close_steps: int = 50, lift_steps: int = 100):
        self.nu = nu
        self.close_steps = close_steps
        self.lift_steps = lift_steps
        self.step_count = 0

    def __call__(self, obs: Dict[str, Any]) -> List[float]:
        self.step_count += 1
        nu = int(obs.get("nu", self.nu)) or self.nu
        if self.nu == 0 and nu > 0:
            self.nu = nu
        if nu <= 0:
            return []
        action = [0.0] * nu
        if self.step_count <= self.close_steps:
            for i in range(min(nu, 2)):
                action[i] = 0.8
        elif self.step_count <= self.close_steps + self.lift_steps:
            for i in range(min(nu, 2)):
                action[i] = 0.5
            if nu >= 4:
                action[2] = 0.3
                action[3] = 0.3
        return action

    def reset(self) -> None:
        self.step_count = 0
