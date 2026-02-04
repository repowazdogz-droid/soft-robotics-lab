"""Core decision analysis logic for Spine Decision Runtime."""

from typing import List, Optional
from .schemas import (
    CaseInput,
    DecisionAnalysis,
    DecisionMap,
    ConstraintCheck,
    FailureMode,
    Contradiction,
    Unknown,
    RecommendedExperiment
)
from .contracts import ContractLoader


class DecisionAnalyzer:
    """Analyzes case inputs and produces decision analysis."""
    
    def __init__(self, contracts_dir: Optional[str] = None):
        """
        Initialize analyzer with contract loader.
        
        Args:
            contracts_dir: Optional path to contracts directory
        """
        self.contract_loader = ContractLoader(contracts_dir)
    
    def analyze(self, case: CaseInput) -> DecisionAnalysis:
        """
        Analyze a case and produce decision analysis.
        
        Args:
            case: Parsed case input
            
        Returns:
            DecisionAnalysis with all findings
        """
        # Check constraints against contracts
        constraints_checked = self._check_constraints(case.constraints)
        violations = [
            check.violation for check in constraints_checked
            if check.violation is not None
        ]
        
        # Build decision map
        decision_map = DecisionMap(
            constraints_checked=constraints_checked,
            violations=violations
        )
        
        # Identify failure modes
        failure_modes = self._identify_failure_modes(case)
        
        # Find contradictions
        contradictions = self._find_contradictions(case)
        
        # Process uncertainties
        unknowns = self._process_uncertainties(case.uncertainties)
        
        # Generate recommended experiments
        recommended_experiments = self._recommend_experiments(case, unknowns)
        
        return DecisionAnalysis(
            decision_map=decision_map,
            failure_modes=failure_modes,
            contradictions=contradictions,
            unknowns=unknowns,
            recommended_experiments=recommended_experiments
        )
    
    def _check_constraints(self, constraints: List[str]) -> List[ConstraintCheck]:
        """Check each constraint against contracts."""
        results = []
        
        for constraint in constraints:
            relevant_contracts = self.contract_loader.find_relevant_contracts(constraint)
            violation = self.contract_loader.check_constraint_violation(constraint)
            
            contract_matched = relevant_contracts[0] if relevant_contracts else None
            
            results.append(ConstraintCheck(
                constraint=constraint,
                checked=True,
                contract_matched=contract_matched,
                violation=violation
            ))
        
        return results
    
    def _identify_failure_modes(self, case: CaseInput) -> List[FailureMode]:
        """Identify potential failure modes from constraints and objectives."""
        failure_modes = []
        
        # Check for slip-related constraints
        if any('slip' in c.lower() for c in case.constraints):
            failure_modes.append(FailureMode(
                mode="slip_under_load",
                severity="high",
                mitigation="increase_friction_coefficient"
            ))
        
        # Check for force-related constraints
        if any('force' in c.lower() for c in case.constraints):
            failure_modes.append(FailureMode(
                mode="excessive_force_damage",
                severity="medium",
                mitigation="implement_force_limiting_control"
            ))
        
        # Check for biocompatibility constraints
        if any('biocompatible' in c.lower() for c in case.constraints):
            failure_modes.append(FailureMode(
                mode="material_degradation",
                severity="high",
                mitigation="verify_material_certification"
            ))
        
        return failure_modes
    
    def _find_contradictions(self, case: CaseInput) -> List[Contradiction]:
        """Find contradictions between constraints and objectives."""
        contradictions = []
        
        # Check for force vs grip tension contradiction
        has_force_limit = any('force' in c.lower() and 'max' in c.lower() for c in case.constraints)
        has_grip_objective = any('grip' in o.lower() for o in case.objectives)
        
        if has_force_limit and has_grip_objective:
            contradictions.append(Contradiction(
                description="max_force vs adequate_grip tension"
            ))
        
        # Check for safety vs performance contradictions
        has_safety_constraint = any('safe' in c.lower() or 'must_not' in c.lower() for c in case.constraints)
        has_performance_objective = any('adequate' in o.lower() or 'minimal' in o.lower() for o in case.objectives)
        
        if has_safety_constraint and has_performance_objective:
            # This is more of a trade-off than contradiction, but flag it
            pass
        
        return contradictions
    
    def _process_uncertainties(self, uncertainties: List[str]) -> List[Unknown]:
        """Process uncertainties and assess their impact."""
        unknowns = []
        
        for uncertainty in uncertainties:
            impact = "medium"  # Default
            
            # Assess impact based on keywords
            if 'compliance' in uncertainty.lower():
                impact = "critical"
                resolution = "bench_test_required"
            elif 'sterilization' in uncertainty.lower():
                impact = "high"
                resolution = "sterilization_cycle_testing"
            elif 'unknown_range' in uncertainty.lower():
                impact = "high"
                resolution = "characterization_study_required"
            else:
                resolution = "investigation_required"
            
            unknowns.append(Unknown(
                item=uncertainty,
                impact=impact,
                resolution=resolution
            ))
        
        return unknowns
    
    def _recommend_experiments(self, case: CaseInput, unknowns: List[Unknown]) -> List[RecommendedExperiment]:
        """Generate recommended experiments based on case and unknowns."""
        experiments = []
        
        # Force-related experiments
        if any('force' in c.lower() for c in case.constraints):
            experiments.append(RecommendedExperiment(
                name="force_limit_characterization"
            ))
        
        # Sterilization experiments
        if any('sterilization' in u.lower() for u in case.uncertainties):
            experiments.append(RecommendedExperiment(
                name="sterilization_cycle_testing"
            ))
        
        # Tissue compliance experiments
        if any('compliance' in u.lower() for u in case.uncertainties):
            experiments.append(RecommendedExperiment(
                name="tissue_compliance_bench_test"
            ))
        
        return experiments
