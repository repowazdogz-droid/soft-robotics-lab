"""
Viewer - Interactive MuJoCo viewer and controlled viewer.
launch_viewer(mjcf_string or scene_path), launch_controlled_viewer(mjcf_string, control_callback).
"""

from pathlib import Path
from typing import Optional, Callable, Any

try:
    import mujoco
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


def launch_viewer(
    mjcf_string: Optional[str] = None,
    scene_path: Optional[str] = None,
) -> None:
    """
    Open interactive MuJoCo passive viewer. User can rotate, zoom, inspect.
    Provide mjcf_string or scene_path.
    """
    if not MUJOCO_AVAILABLE:
        raise RuntimeError("MuJoCo not installed")
    if scene_path:
        path = Path(scene_path)
        if not path.exists():
            raise FileNotFoundError(scene_path)
        model = mujoco.MjModel.from_xml_path(str(path))
    elif mjcf_string:
        model = mujoco.MjModel.from_xml_string(mjcf_string)
    else:
        raise ValueError("Provide mjcf_string or scene_path")
    data = mujoco.MjData(model)
    try:
        with mujoco.viewer.launch_passive(model, data) as v:
            while v.is_running():
                mujoco.mj_step(model, data)
                v.sync()
    except AttributeError:
        _viewer_loop_fallback(model, data, control_callback=None)


def launch_controlled_viewer(
    mjcf_string: str,
    control_callback: Callable[[Any, Any], None],
) -> None:
    """
    Interactive viewer with live control.
    control_callback(model, data) is called each step before mj_step.
    For manual gripper control testing.
    """
    if not MUJOCO_AVAILABLE:
        raise RuntimeError("MuJoCo not installed")
    if not mjcf_string:
        raise ValueError("mjcf_string required")
    model = mujoco.MjModel.from_xml_string(mjcf_string)
    data = mujoco.MjData(model)
    try:
        with mujoco.viewer.launch_passive(model, data) as v:
            while v.is_running():
                control_callback(model, data)
                mujoco.mj_step(model, data)
                v.sync()
    except AttributeError:
        _viewer_loop_fallback(model, data, control_callback=control_callback)


def _viewer_loop_fallback(
    model: Any,
    data: Any,
    control_callback: Optional[Callable[[Any, Any], None]] = None,
) -> None:
    """Fallback when launch_passive is not available: run a fixed number of steps."""
    for _ in range(10000):
        if control_callback:
            control_callback(model, data)
        mujoco.mj_step(model, data)
