"""Core decision analysis logic for Spine Decision Runtime."""

from typing import List, Optional, Dict, Any
from .schemas import (
    CaseInput,
    DecisionAnalysis,
    DecisionMap,
    ConstraintCheck,
    FailureMode,
    Contradiction,
    Unknown,
    RecommendedExperiment,
    EpistemicWeight
)
from .contracts import ContractLoader
from .trace import DecisionTraceGraph


class DecisionAnalyzer:
    """Analyzes case inputs and produces decision analysis."""
    
    def __init__(self, contracts_dir: Optional[str] = None):
        """
        Initialize analyzer with contract loader.
        
        Args:
            contracts_dir: Optional path to contracts directory
        """
        self.contract_loader = ContractLoader(contracts_dir)
    
    def analyze(self, case: CaseInput, context: Optional[Dict[str, Any]] = None) -> DecisionAnalysis:
        """
        Analyze a case and produce decision analysis.
        
        Args:
            case: Parsed case input
            context: Optional context dict containing physics results or other derived data
            
        Returns:
            DecisionAnalysis with all findings
        """
        # Initialize trace graph
        trace = DecisionTraceGraph()
        
        # Add input nodes
        problem_node_id = trace.add_node(
            "input",
            {"type": "problem", "name": case.problem.name, "domain": case.problem.domain}
        )
        
        constraint_node_ids = []
        for constraint in case.constraints:
            node_id = trace.add_node(
                "input",
                {"type": "constraint", "value": constraint},
                parents=[problem_node_id]
            )
            constraint_node_ids.append(node_id)
        
        uncertainty_node_ids = []
        for uncertainty in case.uncertainties:
            node_id = trace.add_node(
                "input",
                {"type": "uncertainty", "value": uncertainty},
                parents=[problem_node_id]
            )
            uncertainty_node_ids.append(node_id)
        
        objective_node_ids = []
        for objective in case.objectives:
            node_id = trace.add_node(
                "input",
                {"type": "objective", "value": objective},
                parents=[problem_node_id]
            )
            objective_node_ids.append(node_id)
        
        # Check constraints against contracts
        constraints_checked = self._check_constraints(case.constraints)
        violations = [
            check.violation for check in constraints_checked
            if check.violation is not None
        ]
        
        # Add inference nodes for constraint checks
        for i, check in enumerate(constraints_checked):
            trace.add_node(
                "inference",
                {
                    "type": "constraint_check",
                    "constraint": check.constraint,
                    "checked": check.checked,
                    "contract_matched": check.contract_matched,
                    "violation": check.violation
                },
                parents=[constraint_node_ids[i]] if i < len(constraint_node_ids) else []
            )
        
        # Build decision map
        decision_map = DecisionMap(
            constraints_checked=constraints_checked,
            violations=violations
        )
        
        # Check if physics results are available in context
        has_physics_results = context is not None and context.get("physics_results") is not None
        
        # Identify failure modes
        failure_modes = self._identify_failure_modes(case, has_physics_results, trace, constraint_node_ids)
        
        # Find contradictions
        contradictions = self._find_contradictions(case, has_physics_results, trace, constraint_node_ids, objective_node_ids)
        
        # Process uncertainties
        unknowns = self._process_uncertainties(case.uncertainties, has_physics_results, trace, uncertainty_node_ids)
        
        # Generate recommended experiments
        recommended_experiments = self._recommend_experiments(case, unknowns, has_physics_results, trace, constraint_node_ids, uncertainty_node_ids)
        
        return DecisionAnalysis(
            decision_map=decision_map,
            failure_modes=failure_modes,
            contradictions=contradictions,
            unknowns=unknowns,
            recommended_experiments=recommended_experiments,
            trace_graph=trace.to_dict()
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
    
    def _identify_failure_modes(self, case: CaseInput, has_physics_results: bool = False, trace: DecisionTraceGraph = None, constraint_node_ids: List[str] = None) -> List[FailureMode]:
        """Identify potential failure modes from constraints and objectives."""
        failure_modes = []
        constraint_node_ids = constraint_node_ids or []
        
        # Check for slip-related constraints
        slip_constraints = [c for c in case.constraints if 'slip' in c.lower()]
        if slip_constraints:
            slip_indices = [i for i, c in enumerate(case.constraints) if 'slip' in c.lower()]
            slip_parent_ids = [constraint_node_ids[i] for i in slip_indices if i < len(constraint_node_ids)]
            
            failure_mode = FailureMode(
                mode="slip_under_load",
                severity="high",
                mitigation="increase_friction_coefficient",
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "heuristic",
                    provenance=[f"constraint: {c}" for c in slip_constraints],
                    requires_validation=not has_physics_results
                )
            )
            failure_modes.append(failure_mode)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "failure_mode",
                        "mode": failure_mode.mode,
                        "severity": failure_mode.severity,
                        "mitigation": failure_mode.mitigation
                    },
                    parents=slip_parent_ids,
                    metadata={"epistemic": failure_mode.epistemic.dict() if failure_mode.epistemic else None}
                )
        
        # Check for force-related constraints
        force_constraints = [c for c in case.constraints if 'force' in c.lower()]
        if force_constraints:
            force_indices = [i for i, c in enumerate(case.constraints) if 'force' in c.lower()]
            force_parent_ids = [constraint_node_ids[i] for i in force_indices if i < len(constraint_node_ids)]
            
            failure_mode = FailureMode(
                mode="excessive_force_damage",
                severity="medium",
                mitigation="implement_force_limiting_control",
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "heuristic",
                    provenance=[f"constraint: {c}" for c in force_constraints],
                    requires_validation=not has_physics_results
                )
            )
            failure_modes.append(failure_mode)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "failure_mode",
                        "mode": failure_mode.mode,
                        "severity": failure_mode.severity,
                        "mitigation": failure_mode.mitigation
                    },
                    parents=force_parent_ids,
                    metadata={"epistemic": failure_mode.epistemic.dict() if failure_mode.epistemic else None}
                )
        
        # Check for biocompatibility constraints
        biocompat_constraints = [c for c in case.constraints if 'biocompatible' in c.lower()]
        if biocompat_constraints:
            biocompat_indices = [i for i, c in enumerate(case.constraints) if 'biocompatible' in c.lower()]
            biocompat_parent_ids = [constraint_node_ids[i] for i in biocompat_indices if i < len(constraint_node_ids)]
            
            failure_mode = FailureMode(
                mode="material_degradation",
                severity="high",
                mitigation="verify_material_certification",
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "heuristic",
                    provenance=[f"constraint: {c}" for c in biocompat_constraints],
                    requires_validation=not has_physics_results
                )
            )
            failure_modes.append(failure_mode)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "failure_mode",
                        "mode": failure_mode.mode,
                        "severity": failure_mode.severity,
                        "mitigation": failure_mode.mitigation
                    },
                    parents=biocompat_parent_ids,
                    metadata={"epistemic": failure_mode.epistemic.dict() if failure_mode.epistemic else None}
                )
        
        return failure_modes
    
    def _find_contradictions(self, case: CaseInput, has_physics_results: bool = False, trace: DecisionTraceGraph = None, constraint_node_ids: List[str] = None, objective_node_ids: List[str] = None) -> List[Contradiction]:
        """Find contradictions between constraints and objectives."""
        contradictions = []
        constraint_node_ids = constraint_node_ids or []
        objective_node_ids = objective_node_ids or []
        provenance_items = []
        
        # Check for force vs grip tension contradiction
        force_limit_constraints = [c for c in case.constraints if 'force' in c.lower() and 'max' in c.lower()]
        grip_objectives = [o for o in case.objectives if 'grip' in o.lower()]
        
        if force_limit_constraints and grip_objectives:
            provenance_items = [f"constraint: {c}" for c in force_limit_constraints]
            provenance_items.extend([f"objective: {o}" for o in grip_objectives])
            
            force_indices = [i for i, c in enumerate(case.constraints) if 'force' in c.lower() and 'max' in c.lower()]
            grip_indices = [i for i, o in enumerate(case.objectives) if 'grip' in o.lower()]
            parent_ids = [constraint_node_ids[i] for i in force_indices if i < len(constraint_node_ids)]
            parent_ids.extend([objective_node_ids[i] for i in grip_indices if i < len(objective_node_ids)])
            
            contradiction = Contradiction(
                description="max_force vs adequate_grip tension",
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "rule_derived",
                    provenance=provenance_items,
                    requires_validation=not has_physics_results
                )
            )
            contradictions.append(contradiction)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "contradiction",
                        "description": contradiction.description
                    },
                    parents=parent_ids,
                    metadata={"epistemic": contradiction.epistemic.dict() if contradiction.epistemic else None}
                )
        
        # Check for safety vs performance contradictions
        has_safety_constraint = any('safe' in c.lower() or 'must_not' in c.lower() for c in case.constraints)
        has_performance_objective = any('adequate' in o.lower() or 'minimal' in o.lower() for o in case.objectives)
        
        if has_safety_constraint and has_performance_objective:
            # This is more of a trade-off than contradiction, but flag it
            pass
        
        return contradictions
    
    def _process_uncertainties(self, uncertainties: List[str], has_physics_results: bool = False, trace: DecisionTraceGraph = None, uncertainty_node_ids: List[str] = None) -> List[Unknown]:
        """Process uncertainties and assess their impact."""
        unknowns = []
        uncertainty_node_ids = uncertainty_node_ids or []
        
        for idx, uncertainty in enumerate(uncertainties):
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
            
            parent_id = uncertainty_node_ids[idx] if idx < len(uncertainty_node_ids) else None
            parent_ids = [parent_id] if parent_id else []
            
            unknown = Unknown(
                item=uncertainty,
                impact=impact,
                resolution=resolution,
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "heuristic",
                    provenance=[f"uncertainty: {uncertainty}"],
                    requires_validation=not has_physics_results
                )
            )
            unknowns.append(unknown)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "unknown",
                        "item": unknown.item,
                        "impact": unknown.impact,
                        "resolution": unknown.resolution
                    },
                    parents=parent_ids,
                    metadata={"epistemic": unknown.epistemic.dict() if unknown.epistemic else None}
                )
        
        return unknowns
    
    def _recommend_experiments(self, case: CaseInput, unknowns: List[Unknown], has_physics_results: bool = False, trace: DecisionTraceGraph = None, constraint_node_ids: List[str] = None, uncertainty_node_ids: List[str] = None) -> List[RecommendedExperiment]:
        """Generate recommended experiments based on case and unknowns."""
        experiments = []
        constraint_node_ids = constraint_node_ids or []
        uncertainty_node_ids = uncertainty_node_ids or []
        provenance_items = []
        
        # Force-related experiments
        force_constraints = [c for c in case.constraints if 'force' in c.lower()]
        if force_constraints:
            provenance_items = [f"constraint: {c}" for c in force_constraints]
            force_indices = [i for i, c in enumerate(case.constraints) if 'force' in c.lower()]
            parent_ids = [constraint_node_ids[i] for i in force_indices if i < len(constraint_node_ids)]
            
            experiment = RecommendedExperiment(
                name="force_limit_characterization",
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "rule_derived",
                    provenance=provenance_items,
                    requires_validation=not has_physics_results
                )
            )
            experiments.append(experiment)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "recommended_experiment",
                        "name": experiment.name
                    },
                    parents=parent_ids,
                    metadata={"epistemic": experiment.epistemic.dict() if experiment.epistemic else None}
                )
        
        # Sterilization experiments
        sterilization_uncertainties = [u for u in case.uncertainties if 'sterilization' in u.lower()]
        if sterilization_uncertainties:
            provenance_items = [f"uncertainty: {u}" for u in sterilization_uncertainties]
            sterilization_indices = [i for i, u in enumerate(case.uncertainties) if 'sterilization' in u.lower()]
            parent_ids = [uncertainty_node_ids[i] for i in sterilization_indices if i < len(uncertainty_node_ids)]
            
            experiment = RecommendedExperiment(
                name="sterilization_cycle_testing",
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "rule_derived",
                    provenance=provenance_items,
                    requires_validation=not has_physics_results
                )
            )
            experiments.append(experiment)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "recommended_experiment",
                        "name": experiment.name
                    },
                    parents=parent_ids,
                    metadata={"epistemic": experiment.epistemic.dict() if experiment.epistemic else None}
                )
        
        # Tissue compliance experiments
        compliance_uncertainties = [u for u in case.uncertainties if 'compliance' in u.lower()]
        if compliance_uncertainties:
            provenance_items = [f"uncertainty: {u}" for u in compliance_uncertainties]
            compliance_indices = [i for i, u in enumerate(case.uncertainties) if 'compliance' in u.lower()]
            parent_ids = [uncertainty_node_ids[i] for i in compliance_indices if i < len(uncertainty_node_ids)]
            
            experiment = RecommendedExperiment(
                name="tissue_compliance_bench_test",
                epistemic=EpistemicWeight(
                    confidence=0.8 if has_physics_results else 0.4,
                    evidence_type="physics_derived" if has_physics_results else "rule_derived",
                    provenance=provenance_items,
                    requires_validation=not has_physics_results
                )
            )
            experiments.append(experiment)
            
            if trace:
                trace.add_node(
                    "output",
                    {
                        "type": "recommended_experiment",
                        "name": experiment.name
                    },
                    parents=parent_ids,
                    metadata={"epistemic": experiment.epistemic.dict() if experiment.epistemic else None}
                )
        
        return experiments
