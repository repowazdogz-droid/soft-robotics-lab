"""
Loader - Load MJCF, URDF, detect format.
Returns MuJoCo model for validation.
"""

from pathlib import Path
from typing import Union

try:
    import mujoco
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False


def detect_format(content: Union[str, bytes]) -> str:
    """
    Detect format from XML content.
    Returns "mjcf" | "urdf" | "unknown".
    """
    if isinstance(content, bytes):
        content = content.decode("utf-8", errors="ignore")
    s = (content or "").strip().lower()
    if "<mujoco" in s or 'model="' in s and "<compiler" in s:
        return "mjcf"
    if "<robot" in s or "urdf" in s or "<link " in s:
        return "urdf"
    return "unknown"


def load_mjcf(xml_string: str = None, file_path: str = None):
    """
    Load MuJoCo model from MJCF XML string or file path.
    Returns MjModel or raises.
    """
    if not MUJOCO_AVAILABLE:
        raise RuntimeError("MuJoCo not installed")
    if file_path:
        path = Path(file_path)
        if not path.exists():
            raise FileNotFoundError(f"File not found: {file_path}")
        return mujoco.MjModel.from_xml_path(str(path))
    if xml_string:
        return mujoco.MjModel.from_xml_string(xml_string)
    raise ValueError("Provide xml_string or file_path")


def load_urdf(xml_string: str = None, file_path: str = None):
    """
    Load model from URDF (XML string or file path).
    MuJoCo can parse URDF via from_xml_path/from_xml_string.
    Returns MjModel or raises.
    """
    if not MUJOCO_AVAILABLE:
        raise RuntimeError("MuJoCo not installed")
    if file_path:
        path = Path(file_path)
        if not path.exists():
            raise FileNotFoundError(f"File not found: {file_path}")
        return mujoco.MjModel.from_xml_path(str(path))
    if xml_string:
        return mujoco.MjModel.from_xml_string(xml_string)
    raise ValueError("Provide xml_string or file_path")


def load_model(xml_string: str = None, file_path: str = None, format_hint: str = None):
    """
    Load model from XML (MJCF or URDF). Auto-detect format if not specified.
    Returns (MjModel, format_used).
    """
    content = xml_string
    if file_path:
        path = Path(file_path)
        content = path.read_text(encoding="utf-8", errors="ignore")
    fmt = format_hint or detect_format(content or "")
    if fmt == "mjcf":
        model = load_mjcf(xml_string=content, file_path=file_path)
        return model, "mjcf"
    if fmt == "urdf":
        model = load_urdf(xml_string=content, file_path=file_path)
        return model, "urdf"
    try:
        model = load_mjcf(xml_string=content, file_path=file_path)
        return model, "mjcf"
    except Exception:
        pass
    try:
        model = load_urdf(xml_string=content, file_path=file_path)
        return model, "urdf"
    except Exception:
        raise ValueError("Could not load as MJCF or URDF")
