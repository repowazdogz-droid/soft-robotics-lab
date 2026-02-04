"""
System status for Soft Robotics Lab.
Used for top-of-page banner: Zoo ready, embeddings, MuJoCo validator.
"""

import sys
from pathlib import Path
from typing import Dict, Any

_soft_lab_root = Path(__file__).resolve().parent.parent
if str(_soft_lab_root) not in sys.path:
    sys.path.insert(0, str(_soft_lab_root))


def get_system_status() -> Dict[str, Any]:
    """
    Returns status dict: zoo_ready, embeddings_enabled, mujoco_available.
    Paths are relative to soft_robotics_lab root for portability.
    """
    zoo_path = _soft_lab_root / "gripper_zoo" / "designs" / "index.json"
    zoo_ready = zoo_path.exists()

    embeddings_enabled = False
    try:
        from sentence_transformers import SentenceTransformer
        _ = SentenceTransformer("all-MiniLM-L6-v2")
        embeddings_enabled = True
    except Exception:
        pass

    mujoco_available = False
    try:
        import mujoco
        from workbench.mujoco_validator import MuJoCoValidator
        _ = MuJoCoValidator()
        mujoco_available = True
    except Exception:
        pass

    return {
        "zoo_ready": zoo_ready,
        "embeddings_enabled": embeddings_enabled,
        "mujoco_available": mujoco_available,
    }


def get_lab_os_status() -> bool:
    """Check if OMEGA Lab OS is reachable."""
    try:
        from utils.lab_os_client import health_check
        return health_check()
    except Exception:
        return False


def render_status_banner(st) -> None:
    """Render a slim status banner at top: Zoo | Embeddings | MuJoCo | Lab OS."""
    status = get_system_status()
    zoo = "Zoo ready" if status["zoo_ready"] else "Zoo not generated"
    emb = "Embeddings on" if status["embeddings_enabled"] else "Embeddings off"
    muj = "MuJoCo ready" if status["mujoco_available"] else "MuJoCo unavailable"
    lab_os = get_lab_os_status()

    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.caption(f"**{zoo}**")
    with c2:
        st.caption(f"**{emb}**")
    with c3:
        st.caption(f"**{muj}**")
    with c4:
        st.metric("Lab OS", "Connected" if lab_os else "Offline")
