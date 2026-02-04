"""Pydantic schemas for Spine Decision Runtime input and output."""

from typing import List, Optional
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


class FailureMode(BaseModel):
    """Identified failure mode"""
    mode: str = Field(..., description="Failure mode identifier")
    severity: str = Field(..., description="Severity level: low, medium, high, critical")
    mitigation: Optional[str] = Field(None, description="Suggested mitigation")


class Contradiction(BaseModel):
    """Identified contradiction between constraints/objectives"""
    description: str = Field(..., description="Description of the contradiction")


class Unknown(BaseModel):
    """Uncertainty that needs resolution"""
    item: str = Field(..., description="Uncertainty identifier")
    impact: str = Field(..., description="Impact level: low, medium, high, critical")
    resolution: Optional[str] = Field(None, description="Suggested resolution approach")


class RecommendedExperiment(BaseModel):
    """Recommended experiment to resolve uncertainty"""
    name: str = Field(..., description="Experiment name")


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
