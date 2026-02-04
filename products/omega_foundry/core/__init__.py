"""
Omega Foundry - Design from natural language intent.
"""

from .intent_parser import IntentParser, DesignSpec
from .design_engine import DesignEngine, GeneratedDesign
from .validator import PhysicsValidator, ValidationResult
from .exporter import DesignExporter

try:
    from .constraint_solver import (
        ConstraintSolver,
        ConstraintSet,
        Constraint,
        ConstraintType,
        ConstraintResult,
        ImpossibilityReport
    )
    _constraint_exports = [
        "ConstraintSolver", "ConstraintSet", "Constraint", "ConstraintType",
        "ConstraintResult", "ImpossibilityReport"
    ]
except ImportError:
    _constraint_exports = []

try:
    from .design_evolver import (
        DesignEvolver,
        DesignVersion,
        EvolutionRule,
        EvolutionResult
    )
    _evolver_exports = ["DesignEvolver", "DesignVersion", "EvolutionRule", "EvolutionResult"]
except ImportError:
    _evolver_exports = []

__all__ = [
    "IntentParser",
    "DesignSpec",
    "DesignEngine",
    "GeneratedDesign",
    "PhysicsValidator",
    "ValidationResult",
    "DesignExporter",
] + _constraint_exports + _evolver_exports
