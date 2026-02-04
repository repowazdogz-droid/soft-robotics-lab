"""
HTTP client for Reality Bridge API.
Uses REALITY_BRIDGE_URL env var (default http://localhost:18000).
"""

import os
from typing import Any, Dict, Optional

import requests

DEFAULT_URL = "http://localhost:18000"


def get_base_url() -> str:
    """Return Reality Bridge base URL from env or default."""
    return os.environ.get("REALITY_BRIDGE_URL", DEFAULT_URL).rstrip("/")


def validate_design(
    xml_string: Optional[str] = None,
    file_path: Optional[str] = None,
    artifact_id: Optional[str] = None,
    timeout: float = 60.0,
) -> Dict[str, Any]:
    """
    POST /validate: validate a single MJCF/URDF design.
    Provide either xml_string or file_path. Returns full API response dict.
    Raises RealityBridgeUnavailable on connection/HTTP errors.
    """
    base = get_base_url()
    url = f"{base}/validate"

    if file_path:
        with open(file_path, "rb") as f:
            files = {"file": (os.path.basename(file_path), f, "application/octet-stream")}
            data = {} if not artifact_id else {"artifact_id": artifact_id}
            resp = requests.post(url, files=files, data=data, timeout=timeout)
    elif xml_string:
        data = {"xml_string": xml_string}
        if artifact_id:
            data["artifact_id"] = artifact_id
        resp = requests.post(url, data=data, timeout=timeout)
    else:
        raise ValueError("Provide xml_string or file_path")

    if resp.status_code == 200:
        return resp.json()

    # Connection/timeout/5xx
    if not resp.ok:
        raise RealityBridgeUnavailable(
            f"Reality Bridge returned {resp.status_code}: {resp.text[:500]}"
        )
    return resp.json()


def health_check(timeout: float = 5.0) -> Dict[str, Any]:
    """GET /health. Returns {"status": "ok", "mujoco_version": "..."} or raises."""
    base = get_base_url()
    resp = requests.get(f"{base}/health", timeout=timeout)
    if not resp.ok:
        raise RealityBridgeUnavailable(f"Health check failed: {resp.status_code}")
    return resp.json()


class RealityBridgeUnavailable(Exception):
    """Raised when Reality Bridge is unreachable or returns an error."""
    pass
