"""
Canonical metrics schema for all runs.
Every run should emit these fields (null if not applicable).
"""

from typing import Optional
from pydantic import BaseModel


class CanonicalMetrics(BaseModel):
    """Standard metrics every run should try to emit"""

    # Primary metric (the main thing we're measuring)
    primary_metric: float
    primary_metric_name: str = "slip_rate"

    # Secondary metrics
    slip_rate: Optional[float] = None
    contact_area: Optional[float] = None
    grasp_force: Optional[float] = None
    grasp_success: Optional[bool] = None

    # Simulation metadata
    sim_steps: Optional[int] = None
    contact_frames: Optional[int] = None

    # Failure flags
    failure_flags: list[str] = []

    # Context
    environment: Optional[str] = None
    design_family: Optional[str] = None

    class Config:
        extra = "allow"  # Allow additional metrics
