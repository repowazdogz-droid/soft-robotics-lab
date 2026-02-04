"""Client for OMEGA Lab OS (Cognition Server) API."""

import os
import requests
from typing import Optional

# Lab OS URL (env for cross-machine; default local 18002)
LAB_OS_URL = os.environ.get("LAB_OS_URL", "http://localhost:18002")


def health_check() -> bool:
    """Check if Lab OS is running"""
    try:
        r = requests.get(f"{LAB_OS_URL}/health", timeout=2)
        return r.status_code == 200
    except Exception:
        return False


def get_hypotheses() -> list:
    """Get all hypotheses from Lab OS"""
    try:
        r = requests.get(f"{LAB_OS_URL}/hypotheses")
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return []


def create_hypothesis(id: str, claim: str, confidence: float = 0.5) -> Optional[dict]:
    """Create hypothesis in Lab OS"""
    try:
        r = requests.post(
            f"{LAB_OS_URL}/hypotheses",
            json={"id": id, "claim": claim, "confidence": confidence}
        )
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return None


def post_run(bundle: dict) -> Optional[dict]:
    """Submit run bundle to Lab OS"""
    try:
        r = requests.post(f"{LAB_OS_URL}/runs", json=bundle)
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return None


def get_hypothesis_evidence(hypothesis_id: str) -> list:
    """Get evidence for a hypothesis"""
    try:
        r = requests.get(f"{LAB_OS_URL}/hypotheses/{hypothesis_id}/evidence")
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return []


def get_brief() -> Optional[dict]:
    """Get weekly brief data from Lab OS"""
    try:
        r = requests.get(f"{LAB_OS_URL}/brief", timeout=5)
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return None


def get_detailed_brief() -> Optional[dict]:
    """Get detailed brief with charts data"""
    try:
        r = requests.get(f"{LAB_OS_URL}/brief/detailed", timeout=10)
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return None
