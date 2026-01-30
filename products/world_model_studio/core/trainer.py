"""
Trainer - Policy evaluation and training.
random_policy(n_actions), evaluate_policy(), random_search(), train_loop().
Uses shared OMEGA ID: run_id() for training runs.
"""

import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Callable, Any, Dict, List, Optional

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

from .task_builder import compute_reward

_PRODUCTS = Path(__file__).resolve().parent.parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.id_generator import run_id as _run_id

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
    early_stopping_patience: Optional[int] = None,
    checkpoint_path: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Simple linear policy: action = weights @ obs.
    Random weight init; keep best. Early stop if no improvement for patience iters. Save best checkpoint.
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
    no_improve = 0
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
            no_improve = 0
            if checkpoint_path:
                try:
                    Path(checkpoint_path).parent.mkdir(parents=True, exist_ok=True)
                    with open(checkpoint_path, "w") as f:
                        json.dump({"best_weights": best_weights, "obs_dim": obs_dim, "act_dim": act_dim, "best_reward": best_reward}, f)
                except Exception:
                    pass
        else:
            no_improve += 1
        if early_stopping_patience is not None and no_improve >= early_stopping_patience:
            break
    return {
        "best_weights": best_weights,
        "best_reward": best_reward,
        "history": history,
        "obs_dim": obs_dim,
        "act_dim": act_dim,
    }


def grid_search(
    sim: Any,
    task: Any,
    param_grid: Dict[str, List[Any]],
    n_episodes_per_eval: int = 5,
) -> Dict[str, Any]:
    """
    Grid search over param combinations. param_grid e.g. {"scale": [0.05, 0.1, 0.2], "n_params": [5, 10]}.
    Uses random_search with fixed scale for weight init. Returns best_weights, best_reward, all results.
    """
    from .policies import LinearPolicy
    from itertools import product
    sim.set_task(task)
    obs0 = sim.reset()
    qpos = obs0.get("qpos", np.array([]))
    qvel = obs0.get("qvel", np.array([]))
    obs_dim = len(np.asarray(qpos).ravel()) + len(np.asarray(qvel).ravel())
    act_dim = int(obs0.get("nu", 1)) or 1
    keys = list(param_grid.keys())
    values = list(param_grid.values())
    best_reward = float("-inf")
    best_weights = None
    best_params = {}
    all_results: List[Dict] = []
    for combo in product(*values):
        params = dict(zip(keys, combo))
        scale = params.get("scale", 0.1)
        n_params = params.get("n_params", min(obs_dim, 10))
        od = min(obs_dim, n_params)
        od = max(1, od)
        policy = LinearPolicy(od, act_dim)
        total = 0.0
        for _ in range(n_episodes_per_eval):
            policy.set_weights((np.random.randn(act_dim, od) * scale).tolist())
            out = run_episode(sim, policy, task, use_task_reward=True)
            total += out["total_reward"]
        mean_r = total / n_episodes_per_eval
        all_results.append({"params": params, "mean_reward": mean_r})
        if mean_r > best_reward:
            best_reward = mean_r
            best_weights = policy.get_weights().tolist()
            best_params = params
    return {
        "best_weights": best_weights,
        "best_reward": best_reward,
        "best_params": best_params,
        "obs_dim": od,
        "act_dim": act_dim,
        "all_results": all_results,
    }


def simple_rl(
    sim: Any,
    task: Any,
    n_steps: int = 500,
    lr: float = 0.01,
    n_episodes_per_update: int = 5,
) -> Dict[str, Any]:
    """
    Basic policy gradient: linear policy, reward as loss signal, gradient step on weights.
    Simplified REINFORCE-style update.
    """
    from .policies import LinearPolicy
    sim.set_task(task)
    obs0 = sim.reset()
    qpos = obs0.get("qpos", np.array([]))
    qvel = obs0.get("qvel", np.array([]))
    obs_dim = max(1, len(np.asarray(qpos).ravel()) + len(np.asarray(qvel).ravel()))
    act_dim = int(obs0.get("nu", 1)) or 1
    policy = LinearPolicy(obs_dim, act_dim)
    policy.set_weights((np.random.randn(act_dim, obs_dim) * 0.05).tolist())
    history: List[float] = []
    best_reward = float("-inf")
    best_weights = policy.get_weights().copy()
    for step in range(n_steps):
        trajectories: List[tuple] = []
        for _ in range(n_episodes_per_update):
            sim.set_task(task)
            obs = sim.reset()
            if hasattr(policy, "reset"):
                policy.reset()
            rews: List[float] = []
            obss: List[Any] = []
            for _ in range(getattr(task, "max_steps", 500)):
                action = policy(obs)
                obs, r, done, _ = sim.step(action)
                r = compute_reward(task, obs, {})
                rews.append(r)
                obss.append(obs)
                if done:
                    break
            total_r = sum(rews)
            trajectories.append((obss, rews, total_r))
            history.append(total_r)
            if total_r > best_reward:
                best_reward = total_r
                best_weights = policy.get_weights().copy()
        if not NUMPY_AVAILABLE or not trajectories:
            continue
        weights = policy.get_weights()
        for obss, rews, total_r in trajectories:
            for t, o in enumerate(obss):
                vec = np.concatenate([np.asarray(o.get("qpos", [])).ravel(), np.asarray(o.get("qvel", [])).ravel()])[:obs_dim]
                if len(vec) < obs_dim:
                    vec = np.pad(vec, (0, obs_dim - len(vec)))
                g = (total_r - np.mean([tr[2] for tr in trajectories])) * vec
                weights += lr * np.outer(np.clip(policy(o), -1, 1), g)
        policy.set_weights(np.clip(weights, -2, 2))
    return {
        "best_weights": best_weights.tolist(),
        "best_reward": float(best_reward),
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
    method: "random_search", "grid_search", "simple_rl", "evaluate_only".
    config: early_stopping_patience, checkpoint_path, param_grid (for grid_search), etc.
    Returns training results.
    """
    config = config or {}
    _LOGS_ROOT.mkdir(parents=True, exist_ok=True)
    rid = _run_id()
    log_path = _LOGS_ROOT / f"run_{rid.replace('-', '_')}.json"
    result = {"method": method, "run_id": rid, "config": config}
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
        search_out = random_search(
            sim, task,
            n_iterations=n_iter, n_params=n_params, n_episodes_per_eval=n_ep_eval,
            early_stopping_patience=config.get("early_stopping_patience"),
            checkpoint_path=config.get("checkpoint_path"),
        )
        result["best_reward"] = search_out["best_reward"]
        result["best_weights"] = search_out["best_weights"]
        result["history"] = search_out["history"]
        result["obs_dim"] = search_out["obs_dim"]
        result["act_dim"] = search_out["act_dim"]
    elif method == "grid_search":
        param_grid = config.get("param_grid", {"scale": [0.05, 0.1], "n_params": [5, 10]})
        n_ep_eval = config.get("n_episodes_per_eval", 3)
        search_out = grid_search(sim, task, param_grid, n_episodes_per_eval=n_ep_eval)
        result["best_reward"] = search_out["best_reward"]
        result["best_weights"] = search_out["best_weights"]
        result["best_params"] = search_out.get("best_params", {})
        result["history"] = [r["mean_reward"] for r in search_out.get("all_results", [])]
        result["obs_dim"] = search_out["obs_dim"]
        result["act_dim"] = search_out["act_dim"]
    elif method == "simple_rl":
        n_steps = config.get("n_steps", 200)
        lr = config.get("lr", 0.01)
        n_ep_per_update = config.get("n_episodes_per_update", 5)
        search_out = simple_rl(sim, task, n_steps=n_steps, lr=lr, n_episodes_per_update=n_ep_per_update)
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
