"""
Simulator - Run MuJoCo simulations.
Simulator: load_scene, reset, step, render, close.
Observations: joint positions, velocities, object pose.
Actions: actuator controls normalized -1 to 1.
Reward: based on task success criteria.
"""

from pathlib import Path
from typing import Optional, Tuple, Any, Dict

try:
    import mujoco
    import numpy as np
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


class Simulator:
    """
    MuJoCo simulation: load_scene(mjcf_string or path), reset(), step(action), render(), close().
    """

    def __init__(self):
        self._model = None
        self._data = None
        self._mjcf_string: Optional[str] = None
        self._task = None

    def load_scene(self, mjcf_string: Optional[str] = None, path: Optional[str] = None) -> None:
        """Load scene from MJCF string or file path."""
        if not MUJOCO_AVAILABLE:
            raise RuntimeError("MuJoCo not installed")
        if path:
            p = Path(path)
            if p.exists():
                self._model = mujoco.MjModel.from_xml_path(str(p))
                self._mjcf_string = p.read_text(encoding="utf-8", errors="replace")
            else:
                raise FileNotFoundError(path)
        elif mjcf_string:
            self._model = mujoco.MjModel.from_xml_string(mjcf_string)
            self._mjcf_string = mjcf_string
        else:
            raise ValueError("Provide mjcf_string or path")
        self._data = mujoco.MjData(self._model)

    def set_task(self, task: Any) -> None:
        """Set task definition for reward/success."""
        self._task = task

    def reset(self) -> Dict[str, Any]:
        """Reset simulation; return initial observation."""
        if self._data is None:
            raise RuntimeError("Load scene first")
        mujoco.mj_resetData(self._model, self._data)
        return self._get_obs()

    def step(self, action: Any) -> Tuple[Dict[str, Any], float, bool, Dict[str, Any]]:
        """
        Step simulation with actuator control.
        action: array normalized -1 to 1, length nu.
        Returns obs, reward, done, info.
        """
        if self._data is None:
            raise RuntimeError("Load scene first")
        nu = self._model.nu
        if nu > 0 and action is not None:
            act = np.asarray(action, dtype=np.float64).flatten()
            if len(act) >= nu:
                self._data.ctrl[:] = np.clip(act[:nu], -1.0, 1.0)
            elif len(act) > 0:
                self._data.ctrl[: len(act)] = np.clip(act, -1.0, 1.0)
        mujoco.mj_step(self._model, self._data)
        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._is_done()
        info = {"reward": reward, "done": done}
        return obs, reward, done, info

    def render(self, mode: str = "rgb_array", width: int = 640, height: int = 480) -> Any:
        """
        Render frame.
        mode='rgb_array': returns numpy (H, W, 3) uint8.
        mode='human': opens interactive MuJoCo viewer (blocking).
        Default resolution 640x480.
        """
        if not MUJOCO_AVAILABLE or self._model is None or self._data is None:
            return None
        if mode == "rgb_array":
            try:
                renderer = mujoco.Renderer(self._model, height=height, width=width)
                renderer.update_scene(self._data)
                pixels = renderer.render()
                return np.asarray(pixels)
            except Exception:
                return None
        if mode == "human":
            try:
                from .viewer import launch_viewer
                launch_viewer(mjcf_string=self._mjcf_string)
            except Exception:
                pass
            return None
        return None

    def record_episode(
        self,
        policy: Any,
        task: Any,
        filepath: str,
        max_steps: int = 500,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> Dict[str, Any]:
        """
        Run episode while collecting frames; save as MP4 with imageio.
        Returns dict: total_reward, success, video_path.
        """
        if not MUJOCO_AVAILABLE or self._model is None or self._data is None:
            return {"total_reward": 0.0, "success": False, "video_path": None}
        self.set_task(task)
        obs = self.reset()
        frames = []
        total_reward = 0.0
        success = False
        for _ in range(max_steps):
            img = self.render(mode="rgb_array", width=width, height=height)
            if img is not None:
                frames.append(img)
            action = policy(obs) if callable(policy) else None
            obs, reward, done, info = self.step(action)
            total_reward += reward
            if done:
                success = True
                break
        video_path = None
        preview_frames = []
        if frames:
            n = len(frames)
            if n >= 3:
                preview_frames = [frames[0], frames[n // 2], frames[-1]]
            elif n == 2:
                preview_frames = [frames[0], frames[-1]]
            elif n == 1:
                preview_frames = [frames[0]]
            try:
                import imageio
                path = Path(filepath)
                path.parent.mkdir(parents=True, exist_ok=True)
                if path.suffix.lower() != ".mp4":
                    path = path.with_suffix(".mp4")
                imageio.mimsave(str(path), frames, fps=fps, codec="libx264", output_params=["-pix_fmt", "yuv420p"])
                video_path = str(path)
            except Exception:
                try:
                    imageio.mimsave(str(path), frames, fps=fps)
                    video_path = str(path)
                except Exception:
                    pass
        return {"total_reward": total_reward, "success": success, "video_path": video_path, "preview_frames": preview_frames}

    def close(self) -> None:
        """Release resources."""
        self._model = None
        self._data = None
        self._task = None

    def _get_obs(self) -> Dict[str, Any]:
        """Observation: joint positions, velocities, object pose if available."""
        if self._data is None:
            return {}
        obs = {}
        obs["qpos"] = np.copy(self._data.qpos)
        obs["qvel"] = np.copy(self._data.qvel)
        obs["nq"] = self._model.nq
        obs["nv"] = self._model.nv
        obs["nu"] = self._model.nu
        return obs

    def _compute_reward(self) -> float:
        """Reward from task success criteria."""
        if self._task is None:
            return 0.0
        criteria = getattr(self._task, "success_criteria", {}) or {}
        z_min = float(criteria.get("min_height") or criteria.get("object_z_min", 0.0))
        if z_min and self._task.goal in ("pick", "lift", "pick_egg", "pick_box", "hold"):
            qpos = self._data.qpos
            if self._model.nq >= 7:
                object_z = float(qpos[2])
                return 1.0 if object_z >= z_min else 0.0
        if self._task.goal == "place" and "target_pos" in criteria:
            target = np.array(criteria["target_pos"], dtype=np.float64)
            tol = float(criteria.get("tolerance", 0.05))
            if self._model.nq >= 7:
                pos = self._data.qpos[:3]
                dist = np.linalg.norm(pos - target)
                return 1.0 if dist <= tol else -0.01 * dist
        return 0.0

    def _is_done(self) -> bool:
        """Done when task success or max steps (handled by caller)."""
        if self._task is None:
            return False
        criteria = getattr(self._task, "success_criteria", {}) or {}
        z_min = float(criteria.get("min_height") or criteria.get("object_z_min", 0.0))
        if z_min and self._task.goal in ("pick", "lift", "pick_egg", "pick_box", "hold"):
            if self._model.nq >= 7:
                return float(self._data.qpos[2]) >= z_min
        return False

    @property
    def model(self):
        return self._model

    @property
    def data(self):
        return self._data
