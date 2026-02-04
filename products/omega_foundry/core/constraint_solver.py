"""
Constraint Solver - Enforce hard constraints on designs

Given constraints (size, force, environment), generate designs that satisfy all.
Reports "impossible" with explanation if constraints conflict.
"""
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any, Tuple
from enum import Enum


class ConstraintType(Enum):
    MIN = "min"
    MAX = "max"
    EXACT = "exact"
    RANGE = "range"
    ENUM = "enum"  # Must be one of specific values
    BOOL = "bool"


@dataclass
class Constraint:
    """A single design constraint."""
    name: str
    parameter: str  # e.g., "finger_length", "max_force"
    constraint_type: ConstraintType
    value: Any  # Depends on type: number, tuple for range, list for enum
    unit: str = ""
    priority: int = 1  # 1 = must satisfy, 2 = should satisfy, 3 = nice to have
    reason: str = ""

    def check(self, actual_value: Any) -> Tuple[bool, str]:
        """Check if actual value satisfies constraint."""
        if actual_value is None:
            return False, f"{self.name}: No value provided"

        if self.constraint_type == ConstraintType.MIN:
            if actual_value >= self.value:
                return True, ""
            return False, f"{self.name}: {actual_value} < {self.value} {self.unit} (minimum)"

        elif self.constraint_type == ConstraintType.MAX:
            if actual_value <= self.value:
                return True, ""
            return False, f"{self.name}: {actual_value} > {self.value} {self.unit} (maximum)"

        elif self.constraint_type == ConstraintType.EXACT:
            if actual_value == self.value:
                return True, ""
            return False, f"{self.name}: {actual_value} != {self.value} {self.unit} (exact)"

        elif self.constraint_type == ConstraintType.RANGE:
            low, high = self.value
            if low <= actual_value <= high:
                return True, ""
            return False, f"{self.name}: {actual_value} not in [{low}, {high}] {self.unit}"

        elif self.constraint_type == ConstraintType.ENUM:
            if actual_value in self.value:
                return True, ""
            return False, f"{self.name}: {actual_value} not in {self.value}"

        elif self.constraint_type == ConstraintType.BOOL:
            if actual_value == self.value:
                return True, ""
            return False, f"{self.name}: expected {self.value}, got {actual_value}"

        return True, ""


@dataclass
class ConstraintSet:
    """A set of constraints for a design."""
    name: str
    constraints: List[Constraint] = field(default_factory=list)

    def add(self, constraint: Constraint):
        self.constraints.append(constraint)

    def add_min(self, name: str, parameter: str, value: float, unit: str = "", priority: int = 1, reason: str = ""):
        self.add(Constraint(name, parameter, ConstraintType.MIN, value, unit, priority, reason))

    def add_max(self, name: str, parameter: str, value: float, unit: str = "", priority: int = 1, reason: str = ""):
        self.add(Constraint(name, parameter, ConstraintType.MAX, value, unit, priority, reason))

    def add_range(self, name: str, parameter: str, low: float, high: float, unit: str = "", priority: int = 1, reason: str = ""):
        self.add(Constraint(name, parameter, ConstraintType.RANGE, (low, high), unit, priority, reason))

    def add_enum(self, name: str, parameter: str, values: List[Any], priority: int = 1, reason: str = ""):
        self.add(Constraint(name, parameter, ConstraintType.ENUM, values, "", priority, reason))

    def add_bool(self, name: str, parameter: str, value: bool, priority: int = 1, reason: str = ""):
        self.add(Constraint(name, parameter, ConstraintType.BOOL, value, "", priority, reason))


@dataclass
class ConstraintResult:
    """Result of constraint checking."""
    satisfied: bool
    violations: List[str]
    warnings: List[str]  # Priority 2-3 violations
    parameters_checked: int
    parameters_passed: int
    feasible_ranges: Dict[str, Tuple[float, float]]  # Computed feasible ranges

    def to_dict(self) -> dict:
        return {
            "satisfied": self.satisfied,
            "violations": self.violations,
            "warnings": self.warnings,
            "parameters_checked": self.parameters_checked,
            "parameters_passed": self.parameters_passed,
            "feasible_ranges": self.feasible_ranges
        }


@dataclass
class ImpossibilityReport:
    """Report when constraints are impossible to satisfy."""
    is_impossible: bool
    conflicts: List[Dict[str, Any]]
    suggestions: List[str]

    def to_dict(self) -> dict:
        return {
            "is_impossible": self.is_impossible,
            "conflicts": self.conflicts,
            "suggestions": self.suggestions
        }


# Design parameter bounds (physical limits)
PARAMETER_BOUNDS = {
    "finger_length": (0.01, 0.3),  # 1cm to 30cm
    "finger_width": (0.005, 0.05),  # 5mm to 5cm
    "finger_count": (2, 8),
    "palm_size": (0.02, 0.2),  # 2cm to 20cm
    "max_force": (0.5, 100),  # 0.5N to 100N
    "max_pressure": (10, 300),  # 10kPa to 300kPa
    "stiffness": (0.001, 10),  # MPa
    "mass": (0.01, 2.0),  # 10g to 2kg
    "response_time": (0.01, 5.0),  # 10ms to 5s
}

# Material properties
MATERIAL_PROPERTIES = {
    "ecoflex_00-30": {"stiffness": 0.07, "density": 1070, "max_strain": 9.0, "food_safe": True, "sterilizable": False},
    "ecoflex_00-50": {"stiffness": 0.08, "density": 1070, "max_strain": 8.0, "food_safe": True, "sterilizable": False},
    "dragon_skin_10": {"stiffness": 0.15, "density": 1080, "max_strain": 10.0, "food_safe": True, "sterilizable": True},
    "dragon_skin_30": {"stiffness": 0.59, "density": 1080, "max_strain": 4.0, "food_safe": True, "sterilizable": True},
    "elastosil": {"stiffness": 0.3, "density": 1100, "max_strain": 6.0, "food_safe": True, "sterilizable": True},
    "tpu": {"stiffness": 10, "density": 1200, "max_strain": 5.0, "food_safe": False, "sterilizable": True},
}


class ConstraintSolver:
    """Solve design constraints and find feasible parameter ranges."""

    def __init__(self):
        self.parameter_bounds = PARAMETER_BOUNDS.copy()
        self.materials = MATERIAL_PROPERTIES.copy()

    def check_constraints(
        self,
        design_params: Dict[str, Any],
        constraint_set: ConstraintSet
    ) -> ConstraintResult:
        """
        Check if design parameters satisfy constraints.

        Args:
            design_params: Current design parameters
            constraint_set: Constraints to check

        Returns:
            ConstraintResult with violations and warnings
        """
        violations = []
        warnings = []
        passed = 0

        for constraint in constraint_set.constraints:
            param_value = design_params.get(constraint.parameter)

            satisfied, message = constraint.check(param_value)

            if not satisfied:
                if constraint.priority == 1:
                    violations.append(message)
                else:
                    warnings.append(message)
            else:
                passed += 1

        # Compute feasible ranges based on constraints
        feasible = self._compute_feasible_ranges(constraint_set)

        return ConstraintResult(
            satisfied=len(violations) == 0,
            violations=violations,
            warnings=warnings,
            parameters_checked=len(constraint_set.constraints),
            parameters_passed=passed,
            feasible_ranges=feasible
        )

    def _compute_feasible_ranges(
        self,
        constraint_set: ConstraintSet
    ) -> Dict[str, Tuple[float, float]]:
        """Compute feasible ranges for each parameter given constraints."""
        ranges = {}

        # Group constraints by parameter
        param_constraints = {}
        for c in constraint_set.constraints:
            if c.parameter not in param_constraints:
                param_constraints[c.parameter] = []
            param_constraints[c.parameter].append(c)

        # Compute range for each parameter
        for param, constraints in param_constraints.items():
            # Start with physical bounds
            if param in self.parameter_bounds:
                low, high = self.parameter_bounds[param]
            else:
                low, high = float('-inf'), float('inf')

            # Apply constraints
            for c in constraints:
                if c.constraint_type == ConstraintType.MIN:
                    low = max(low, c.value)
                elif c.constraint_type == ConstraintType.MAX:
                    high = min(high, c.value)
                elif c.constraint_type == ConstraintType.RANGE:
                    low = max(low, c.value[0])
                    high = min(high, c.value[1])
                elif c.constraint_type == ConstraintType.EXACT:
                    low = high = c.value

            if low <= high:
                ranges[param] = (low, high)
            else:
                ranges[param] = None  # Infeasible

        return ranges

    def check_impossibility(
        self,
        constraint_set: ConstraintSet
    ) -> ImpossibilityReport:
        """
        Check if constraints are impossible to satisfy.

        Returns:
            ImpossibilityReport with conflicts and suggestions
        """
        conflicts = []
        suggestions = []

        # Check for direct conflicts
        feasible = self._compute_feasible_ranges(constraint_set)

        for param, range_val in feasible.items():
            if range_val is None:
                # Find the conflicting constraints
                param_constraints = [c for c in constraint_set.constraints if c.parameter == param]

                conflict = {
                    "parameter": param,
                    "constraints": [
                        {
                            "name": c.name,
                            "type": c.constraint_type.value,
                            "value": c.value,
                            "reason": c.reason
                        }
                        for c in param_constraints
                    ]
                }
                conflicts.append(conflict)

                # Generate suggestions
                mins = [c for c in param_constraints if c.constraint_type == ConstraintType.MIN]
                maxs = [c for c in param_constraints if c.constraint_type == ConstraintType.MAX]

                if mins and maxs:
                    min_val = max(c.value for c in mins)
                    max_val = min(c.value for c in maxs)
                    suggestions.append(
                        f"Relax {param}: min is {min_val}, max is {max_val}. "
                        f"Either increase max to >{min_val} or decrease min to <{max_val}"
                    )

        # Check for derived conflicts (e.g., force requires size)
        derived_conflicts = self._check_derived_conflicts(constraint_set)
        conflicts.extend(derived_conflicts["conflicts"])
        suggestions.extend(derived_conflicts["suggestions"])

        return ImpossibilityReport(
            is_impossible=len(conflicts) > 0,
            conflicts=conflicts,
            suggestions=suggestions
        )

    def _check_derived_conflicts(
        self,
        constraint_set: ConstraintSet
    ) -> Dict[str, List]:
        """Check for conflicts that arise from physical relationships."""
        conflicts = []
        suggestions = []

        # Get constraint values
        constraints_by_param = {}
        for c in constraint_set.constraints:
            constraints_by_param[c.parameter] = c

        # Check: high force requires larger fingers
        if "max_force" in constraints_by_param and "finger_length" in constraints_by_param:
            force_c = constraints_by_param["max_force"]
            length_c = constraints_by_param["finger_length"]

            if force_c.constraint_type == ConstraintType.MIN:
                required_force = force_c.value
                # Rough approximation: need ~1cm per 5N
                min_length_for_force = required_force * 0.002  # 2mm per N

                if length_c.constraint_type == ConstraintType.MAX and length_c.value < min_length_for_force:
                    conflicts.append({
                        "type": "derived",
                        "parameters": ["max_force", "finger_length"],
                        "explanation": f"Force of {required_force}N requires finger length ≥{min_length_for_force*1000:.0f}mm, but max is {length_c.value*1000:.0f}mm"
                    })
                    suggestions.append(
                        f"Either reduce required force below {length_c.value / 0.002:.0f}N or increase max finger length above {min_length_for_force*1000:.0f}mm"
                    )

        # Check: small size limits force
        if "palm_size" in constraints_by_param and "max_force" in constraints_by_param:
            size_c = constraints_by_param["palm_size"]
            force_c = constraints_by_param["max_force"]

            if size_c.constraint_type == ConstraintType.MAX and force_c.constraint_type == ConstraintType.MIN:
                max_size = size_c.value
                required_force = force_c.value
                # Rough: max force ≈ palm_size * 500N/m
                max_achievable_force = max_size * 500

                if required_force > max_achievable_force:
                    conflicts.append({
                        "type": "derived",
                        "parameters": ["palm_size", "max_force"],
                        "explanation": f"Palm size {max_size*1000:.0f}mm can only achieve ~{max_achievable_force:.0f}N, but {required_force}N required"
                    })
                    suggestions.append(
                        f"Either increase palm size to ≥{required_force/500*1000:.0f}mm or reduce force requirement to ≤{max_achievable_force:.0f}N"
                    )

        return {"conflicts": conflicts, "suggestions": suggestions}

    def solve_for_feasible_design(
        self,
        constraint_set: ConstraintSet,
        base_design: Dict[str, Any] = None
    ) -> Tuple[Optional[Dict[str, Any]], ConstraintResult]:
        """
        Find a feasible design that satisfies all constraints.

        Args:
            constraint_set: Constraints to satisfy
            base_design: Starting design to modify (optional)

        Returns:
            (feasible_design, constraint_result) or (None, result with violations)
        """
        # Check for impossibility first
        impossibility = self.check_impossibility(constraint_set)
        if impossibility.is_impossible:
            return None, ConstraintResult(
                satisfied=False,
                violations=[f"Impossible: {c.get('explanation', str(c))}" for c in impossibility.conflicts],
                warnings=[],
                parameters_checked=len(constraint_set.constraints),
                parameters_passed=0,
                feasible_ranges={}
            )

        # Compute feasible ranges
        feasible = self._compute_feasible_ranges(constraint_set)

        # Start with base design or empty
        design = (base_design or {}).copy()

        # Set each parameter to middle of feasible range
        for param, range_val in feasible.items():
            if range_val is not None:
                low, high = range_val
                if low == high:
                    design[param] = low
                else:
                    # Use middle of range, biased toward lower end for safety
                    design[param] = low + (high - low) * 0.4

        # Handle enum constraints
        for c in constraint_set.constraints:
            if c.constraint_type == ConstraintType.ENUM:
                design[c.parameter] = c.value[0]  # Use first allowed value
            elif c.constraint_type == ConstraintType.BOOL:
                design[c.parameter] = c.value

        # Verify result
        result = self.check_constraints(design, constraint_set)

        if result.satisfied:
            return design, result
        else:
            return None, result

    def suggest_material(
        self,
        constraint_set: ConstraintSet
    ) -> List[Tuple[str, float]]:
        """
        Suggest materials that satisfy constraints.

        Returns:
            List of (material_name, compatibility_score) sorted by score
        """
        suggestions = []

        # Get relevant constraints
        stiffness_c = None
        food_safe_c = None
        sterilizable_c = None

        for c in constraint_set.constraints:
            if c.parameter == "stiffness":
                stiffness_c = c
            elif c.parameter == "food_safe":
                food_safe_c = c
            elif c.parameter == "sterilizable":
                sterilizable_c = c

        for name, props in self.materials.items():
            score = 1.0

            # Check stiffness
            if stiffness_c:
                satisfied, _ = stiffness_c.check(props["stiffness"])
                if not satisfied:
                    score *= 0.5

            # Check food safe
            if food_safe_c and food_safe_c.value:
                if not props["food_safe"]:
                    score *= 0.3

            # Check sterilizable
            if sterilizable_c and sterilizable_c.value:
                if not props["sterilizable"]:
                    score *= 0.3

            suggestions.append((name, score))

        return sorted(suggestions, key=lambda x: x[1], reverse=True)
