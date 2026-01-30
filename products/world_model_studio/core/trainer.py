"""
Trainer - Policy evaluation and training.
random_policy(n_actions), evaluate_policy(), random_search(), train_loop().
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Callable, Any, Dict, List, Optional

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

from .task_builder import compute_reward

_OUTPUTS_ROOT = Path(__file__).resolve().parent.parent / "outputs"
_LOGS_ROOT = Path(__file__).resolve().parent.parent / "training_logs"


def random_policy(obs: Dict[str, Any] = None, n_actions: Optional[int] = None) -> Any:
    """Return random actions in [-1, 1]. If n_actions given use it; else use obs['nu']."""
    if n_actions is not None:
        nu = n_actions
    elif obs is not None:
        nu = int(obs.get("nu", 0))
    else:
        nu = 0
    if nu <= 0:
        return []
    if not NUMPY_AVAILABLE:
        return [0.0] * nu
    return np.random.uniform(-1.0, 1.0, size=nu).tolist()


def run_episode(
    simulator: Any,
    policy: Callable[[Dict], Any],
    task: Any,
    render: bool = False,
    max_steps: Optional[int] = None,
    use_task_reward: bool = False,
) -> Dict[str, Any]:
    """
    Run one episode; return total_reward, success, steps, rewards_per_step.
    If use_task_reward, use compute_reward(task, obs, info) instead of sim step reward.
    """
    max_steps = max_steps or getattr(task, "max_steps", 500)
    simulator.set_task(task)
    obs = simulator.reset()
    if hasattr(policy, "reset"):
        policy.reset()
    total_reward = 0.0
    steps = 0
    rewards_list: List[float] = []
    success = False
    for _ in range(max_steps):
        action = policy(obs)
        obs, step_reward, done, info = simulator.step(action)
        if use_task_reward:
            step_reward = compute_reward(task, obs, info)
        total_reward += step_reward
        rewards_list.append(step_reward)
        steps += 1
        if render and hasattr(simulator, "render"):
            simulator.render(mode="rgb_array")
        if done:
            success = True
            break
    criteria = getattr(task, "success_criteria", {}) or {}
    z_min = float(criteria.get("min_height") or criteria.get("object_z_min", 0.0))
    if not success and obs.get("qpos") is not None and len(obs["qpos"]) >= 3:
        try:
            success = float(obs["qpos"][2]) >= z_min
        except Exception:
            pass
    return {"total_reward": total_reward, "success": success, "steps": steps, "rewards": rewards_list}


def evaluate_policy(
    sim: Any,
    task: Any,
    policy: Callable[[Dict], Any],
    n_episodes: int = 10,
    use_task_reward: bool = True,
) -> Dict[str, Any]:
    """Run n episodes; return success_rate, avg_reward, avg_steps."""
    successes = 0
    total_rewards: List[float] = []
    total_steps_list: List[int] = []
    for _ in range(n_episodes):
        out = run_episode(sim, policy, task, render=False, use_task_reward=use_task_reward)
        if out["success"]:
            successes += 1
        total_rewards.append(out["total_reward"])
        total_steps_list.append(out["steps"])
    return {
        "success_rate": successes / n_episodes if n_episodes else 0.0,
        "avg_reward": float(np.mean(total_rewards)) if total_rewards and NUMPY_AVAILABLE else 0.0,
        "avg_steps": float(np.mean(total_steps_list)) if total_steps_list and NUMPY_AVAILABLE else 0.0,
        "n_episodes": n_episodes,
    }


def random_search(
    sim: Any,
    task: Any,
    n_iterations: int = 100,
    n_params: Optional[int] = None,
    n_episodes_per_eval: int = 5,
) -> Dict[str, Any]:
    """
    Simple linear policy: action = weights @ obs.
    Random weight init; keep best. Return best_weights, best_reward, history.
    """
    from .policies import LinearPolicy
    sim.set_task(task)
    obs0 = sim.reset()
    qpos = obs0.get("qpos", np.array([]))
    qvel = obs0.get("qvel", np.array([]))
    obs_dim = len(np.asarray(qpos).ravel()) + len(np.asarray(qvel).ravel())
    act_dim = int(obs0.get("nu", 1)) or 1
    if n_params is not None:
        obs_dim = min(obs_dim, n_params)
    obs_dim = max(1, obs_dim)
    policy = LinearPolicy(obs_dim, act_dim)
    best_reward = float("-inf")
    best_weights = None
    history: List[float] = []
    for it in range(n_iterations):
        weights = np.random.randn(act_dim, obs_dim) * 0.1
        policy.set_weights(weights)
        total = 0.0
        for _ in range(n_episodes_per_eval):
            out = run_episode(sim, policy, task, use_task_reward=True)
            total += out["total_reward"]
        mean_r = total / n_episodes_per_eval
        history.append(mean_r)
        if mean_r > best_reward:
            best_reward = mean_r
            best_weights = policy.get_weights().tolist()
    return {
        "best_weights": best_weights,
        "best_reward": best_reward,
        "history": history,
        "obs_dim": obs_dim,
        "act_dim": act_dim,
    }


def train_loop(
    sim: Any,
    task: Any,
    method: str = "random_search",
    config: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Wrapper for training methods. Logs progress to training_logs/.
    method: "random_search", "evaluate_only".
    Returns training results.
    """
    config = config or {}
    _LOGS_ROOT.mkdir(parents=True, exist_ok=True)
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = _LOGS_ROOT / f"run_{run_id}.json"
    result = {"method": method, "run_id": run_id, "config": config}
    if method == "evaluate_only":
        from .policies import ConstantPolicy
        policy = ConstantPolicy(config.get("actions", [0.0]))
        n_ep = config.get("n_episodes", 10)
        eval_out = evaluate_policy(sim, task, policy, n_episodes=n_ep)
        result["evaluation"] = eval_out
    elif method == "random_search":
        n_iter = config.get("n_iterations", 100)
        n_params = config.get("n_params", 10)
        n_ep_eval = config.get("n_episodes_per_eval", 5)
        search_out = random_search(sim, task, n_iterations=n_iter, n_params=n_params, n_episodes_per_eval=n_ep_eval)
        result["best_reward"] = search_out["best_reward"]
        result["best_weights"] = search_out["best_weights"]
        result["history"] = search_out["history"]
        result["obs_dim"] = search_out["obs_dim"]
        result["act_dim"] = search_out["act_dim"]
    else:
        result["error"] = f"Unknown method: {method}"
    try:
        log_data = {k: v for k, v in result.items() if k != "best_weights" or isinstance(v, (list, dict))}
        if "best_weights" in result and result["best_weights"] is not None:
            log_data["best_weights_shape"] = [len(result["best_weights"]), len(result["best_weights"][0])] if result["best_weights"] else None
        log_path.write_text(json.dumps(log_data, indent=2, default=str), encoding="utf-8")
    except Exception:
        pass
    result["log_path"] = str(log_path)
    return result


def train_random_search(
    simulator: Any,
    task: Any,
    n_episodes: int = 100,
    render_best: bool = False,
    save_video_path: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Legacy: train by random policy baseline. Kept for app compatibility.
    """
    simulator.set_task(task)
    best_reward = float("-inf")
    rewards: List[float] = []
    for ep in range(n_episodes):
        total = run_episode(simulator, lambda o: random_policy(o), task, render=False, use_task_reward=True)["total_reward"]
        rewards.append(total)
        if total > best_reward:
            best_reward = total
    metrics = {
        "n_episodes": n_episodes,
        "mean_reward": float(np.mean(rewards)) if rewards and NUMPY_AVAILABLE else 0.0,
        "max_reward": float(max(rewards)) if rewards else 0.0,
        "min_reward": float(min(rewards)) if rewards else 0.0,
    }
    result = {"best_params": {}, "metrics": metrics, "rewards": rewards}
    if save_video_path and render_best:
        _OUTPUTS_ROOT.mkdir(parents=True, exist_ok=True)
        result["video_path"] = save_video_path
    return result
