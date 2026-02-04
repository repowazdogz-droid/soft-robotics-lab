"""Pydantic schemas for OMEGA Lab Cognition Server."""

from datetime import datetime
from typing import Literal, Optional

from pydantic import BaseModel, Field


class RunOutcome(BaseModel):
    """Outcome of a single run (supports/refutes/ambiguous)."""

    direction: Literal["supports", "refutes", "ambiguous"]
    strength: float = Field(ge=0.0, le=1.0)
    rationale: str


class RunBundle(BaseModel):
    """Full run payload submitted to POST /runs."""

    run_id: str
    parent_run_id: Optional[str] = None
    batch_id: Optional[str] = None
    schema_version: str = "1.0"
    created_at: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat() + "Z"
    )
    origin: str = "runner"  # runner | manual | simulation
    timestamp: datetime
    engine: str
    design_id: str
    hypothesis_id: str
    experiment_id: str
    environment: str
    metrics: dict
    outcome: RunOutcome
    artifact_manifest: list[str]
    notes: str


class HypothesisCreate(BaseModel):
    """Request body for creating a hypothesis."""

    id: str
    claim: str
    confidence: float = 0.5
    schema_version: str = "1.0"


class HypothesisResponse(BaseModel):
    """Response for a single hypothesis."""

    id: str
    claim: str
    status: str
    confidence: float
    schema_version: str = "1.0"
    created_at: datetime
    updated_at: datetime


class EvidenceResponse(BaseModel):
    """Response for a single evidence record."""

    id: int
    hypothesis_id: str
    run_id: str
    direction: str
    strength: float
    rationale: str
    timestamp: datetime


class RunResponse(BaseModel):
    """Response for a single run (GET /runs/{run_id})."""

    run_id: str
    hypothesis_id: str
    experiment_id: str
    engine: str
    timestamp: datetime
    environment: str
    metrics_json: str
    artifacts_path: str
    notes: str
