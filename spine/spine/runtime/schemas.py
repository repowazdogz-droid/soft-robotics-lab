"""Pydantic schemas for Spine Decision Runtime input and output."""

from typing import List, Optional, Literal, Dict, Any
from pydantic import BaseModel, Field


class ProblemDefinition(BaseModel):
    """Problem definition from case.yaml"""
    name: str = Field(..., description="Problem name")
    domain: str = Field(..., description="Problem domain")


class CaseInput(BaseModel):
    """Input schema for case.yaml"""
    problem: ProblemDefinition
    constraints: List[str] = Field(default_factory=list, description="List of constraints")
    uncertainties: List[str] = Field(default_factory=list, description="List of uncertainties")
    objectives: List[str] = Field(default_factory=list, description="List of objectives")


class ConstraintCheck(BaseModel):
    """Result of checking a constraint against contracts"""
    constraint: str
    checked: bool
    contract_matched: Optional[str] = None
    violation: Optional[str] = None


class EpistemicWeight(BaseModel):
    """Epistemic weighting metadata for analysis outputs"""
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence level 0.0-1.0")
    evidence_type: Literal["physics_derived", "rule_derived", "heuristic"] = Field(
        ..., description="Type of evidence supporting this output"
    )
    provenance: List[str] = Field(
        default_factory=list, description="List of input sources that led to this output"
    )
    requires_validation: bool = Field(
        default=False, description="Whether this output requires validation"
    )


class FailureMode(BaseModel):
    """Identified failure mode"""
    mode: str = Field(..., description="Failure mode identifier")
    severity: str = Field(..., description="Severity level: low, medium, high, critical")
    mitigation: Optional[str] = Field(None, description="Suggested mitigation")
    epistemic: Optional[EpistemicWeight] = Field(None, description="Epistemic weighting metadata")


class Contradiction(BaseModel):
    """Identified contradiction between constraints/objectives"""
    description: str = Field(..., description="Description of the contradiction")
    epistemic: Optional[EpistemicWeight] = Field(None, description="Epistemic weighting metadata")


class Unknown(BaseModel):
    """Uncertainty that needs resolution"""
    item: str = Field(..., description="Uncertainty identifier")
    impact: str = Field(..., description="Impact level: low, medium, high, critical")
    resolution: Optional[str] = Field(None, description="Suggested resolution approach")
    epistemic: Optional[EpistemicWeight] = Field(None, description="Epistemic weighting metadata")


class RecommendedExperiment(BaseModel):
    """Recommended experiment to resolve uncertainty"""
    name: str = Field(..., description="Experiment name")
    epistemic: Optional[EpistemicWeight] = Field(None, description="Epistemic weighting metadata")


class DecisionMap(BaseModel):
    """Decision map output"""
    constraints_checked: List[ConstraintCheck] = Field(default_factory=list)
    violations: List[str] = Field(default_factory=list)


class DecisionAnalysis(BaseModel):
    """Complete decision analysis output"""
    decision_map: DecisionMap
    failure_modes: List[FailureMode] = Field(default_factory=list)
    contradictions: List[Contradiction] = Field(default_factory=list)
    unknowns: List[Unknown] = Field(default_factory=list)
    recommended_experiments: List[RecommendedExperiment] = Field(default_factory=list)
    trace_graph: Optional[Dict] = Field(None, description="Decision trace graph for reasoning chains")
