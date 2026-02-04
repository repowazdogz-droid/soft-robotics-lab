"""
Design Evolver - Evolve designs based on validation failures

Takes validation failures from Reality Bridge and evolves design parameters.
Tracks design lineage (v1 → v2 → v3).
"""
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any, Tuple
from datetime import datetime
from pathlib import Path
import json
import hashlib


@dataclass
class DesignVersion:
    """A version of a design."""
    version_id: str
    parent_id: Optional[str]
    design_params: Dict[str, Any]
    created_at: str
    changes_from_parent: List[Dict[str, Any]]
    validation_result: Optional[Dict[str, Any]] = None
    validation_passed: bool = False

    def to_dict(self) -> dict:
        return {
            "version_id": self.version_id,
            "parent_id": self.parent_id,
            "design_params": self.design_params,
            "created_at": self.created_at,
            "changes_from_parent": self.changes_from_parent,
            "validation_result": self.validation_result,
            "validation_passed": self.validation_passed
        }


@dataclass
class EvolutionRule:
    """Rule for evolving design based on failure type."""
    failure_pattern: str  # Pattern to match in failure message
    parameter: str
    adjustment: str  # "increase", "decrease", "multiply"
    factor: float
    max_applications: int = 3
    reasoning: str = ""


@dataclass
class EvolutionResult:
    """Result of design evolution."""
    original_version: str
    new_version: str
    changes_made: List[Dict[str, Any]]
    rules_applied: List[str]
    expected_improvement: str

    def to_dict(self) -> dict:
        return {
            "original_version": self.original_version,
            "new_version": self.new_version,
            "changes_made": self.changes_made,
            "rules_applied": self.rules_applied,
            "expected_improvement": self.expected_improvement
        }


# Evolution rules based on common failure patterns
EVOLUTION_RULES = [
    # Stability failures
    EvolutionRule(
        failure_pattern="stability",
        parameter="base_width",
        adjustment="multiply",
        factor=1.3,
        reasoning="Wider base improves stability"
    ),
    EvolutionRule(
        failure_pattern="tip over",
        parameter="palm_height",
        adjustment="multiply",
        factor=0.8,
        reasoning="Lower center of mass prevents tip-over"
    ),

    # Reach/length failures
    EvolutionRule(
        failure_pattern="too short",
        parameter="finger_length",
        adjustment="multiply",
        factor=1.2,
        reasoning="Longer fingers increase reach"
    ),
    EvolutionRule(
        failure_pattern="reach",
        parameter="finger_length",
        adjustment="multiply",
        factor=1.15,
        reasoning="Increase length for better reach"
    ),

    # Force failures
    EvolutionRule(
        failure_pattern="grip force",
        parameter="actuator_force",
        adjustment="multiply",
        factor=1.3,
        reasoning="Increase actuator force for stronger grip"
    ),
    EvolutionRule(
        failure_pattern="weak grip",
        parameter="finger_width",
        adjustment="multiply",
        factor=1.2,
        reasoning="Wider fingers provide more contact area"
    ),

    # Collision failures
    EvolutionRule(
        failure_pattern="collision",
        parameter="finger_spacing",
        adjustment="multiply",
        factor=1.2,
        reasoning="More spacing prevents self-collision"
    ),
    EvolutionRule(
        failure_pattern="self-collision",
        parameter="spread_angle",
        adjustment="increase",
        factor=10,  # degrees
        reasoning="Wider spread angle prevents collision"
    ),

    # Dynamics failures
    EvolutionRule(
        failure_pattern="oscillation",
        parameter="damping",
        adjustment="multiply",
        factor=1.5,
        reasoning="Higher damping reduces oscillation"
    ),
    EvolutionRule(
        failure_pattern="slow response",
        parameter="actuator_gain",
        adjustment="multiply",
        factor=1.3,
        reasoning="Higher gain improves response speed"
    ),
    EvolutionRule(
        failure_pattern="overshoot",
        parameter="damping",
        adjustment="multiply",
        factor=1.3,
        reasoning="Higher damping reduces overshoot"
    ),

    # Mass failures
    EvolutionRule(
        failure_pattern="too heavy",
        parameter="wall_thickness",
        adjustment="multiply",
        factor=0.8,
        reasoning="Thinner walls reduce mass"
    ),
    EvolutionRule(
        failure_pattern="mass",
        parameter="density",
        adjustment="multiply",
        factor=0.85,
        reasoning="Lower density material reduces mass"
    ),

    # Stiffness failures
    EvolutionRule(
        failure_pattern="too stiff",
        parameter="stiffness",
        adjustment="multiply",
        factor=0.7,
        reasoning="Reduce stiffness for more compliance"
    ),
    EvolutionRule(
        failure_pattern="too soft",
        parameter="stiffness",
        adjustment="multiply",
        factor=1.4,
        reasoning="Increase stiffness for more rigidity"
    ),
]


class DesignEvolver:
    """Evolve designs based on validation feedback."""

    def __init__(self, history_dir: Path = None):
        """
        Initialize evolver with history storage.

        Args:
            history_dir: Directory to store design history
        """
        self.history_dir = history_dir or Path(__file__).parent.parent / "data" / "evolution_history"
        self.history_dir.mkdir(parents=True, exist_ok=True)
        self.rules = EVOLUTION_RULES.copy()
        self._design_history: Dict[str, List[DesignVersion]] = {}

    def _generate_version_id(self, design_params: Dict) -> str:
        """Generate unique version ID."""
        content = json.dumps(design_params, sort_keys=True)
        hash_val = hashlib.md5(content.encode()).hexdigest()[:8]
        return f"v_{hash_val}"

    def _load_history(self, design_name: str) -> List[DesignVersion]:
        """Load history for a design."""
        if design_name in self._design_history:
            return self._design_history[design_name]

        history_file = self.history_dir / f"{design_name}_history.json"
        if history_file.exists():
            try:
                data = json.loads(history_file.read_text())
                versions = [DesignVersion(**v) for v in data.get("versions", [])]
                self._design_history[design_name] = versions
                return versions
            except Exception:
                pass

        return []

    def _save_history(self, design_name: str, versions: List[DesignVersion]):
        """Save history for a design."""
        self._design_history[design_name] = versions

        history_file = self.history_dir / f"{design_name}_history.json"
        data = {
            "design_name": design_name,
            "versions": [v.to_dict() for v in versions],
            "updated_at": datetime.now().isoformat()
        }
        history_file.write_text(json.dumps(data, indent=2))

    def create_initial_version(
        self,
        design_name: str,
        design_params: Dict[str, Any]
    ) -> DesignVersion:
        """Create initial version of a design."""
        version = DesignVersion(
            version_id=self._generate_version_id(design_params),
            parent_id=None,
            design_params=design_params.copy(),
            created_at=datetime.now().isoformat(),
            changes_from_parent=[]
        )

        history = self._load_history(design_name)
        history.append(version)
        self._save_history(design_name, history)

        return version

    def record_validation(
        self,
        design_name: str,
        version_id: str,
        validation_result: Dict[str, Any],
        passed: bool
    ):
        """Record validation result for a version."""
        history = self._load_history(design_name)

        for version in history:
            if version.version_id == version_id:
                version.validation_result = validation_result
                version.validation_passed = passed
                break

        self._save_history(design_name, history)

    def evolve_from_failure(
        self,
        design_name: str,
        current_version_id: str,
        failure_messages: List[str]
    ) -> Tuple[Optional[DesignVersion], EvolutionResult]:
        """
        Evolve design based on validation failures.

        Args:
            design_name: Name of the design
            current_version_id: Current version that failed
            failure_messages: List of failure messages from validation

        Returns:
            (new_version, evolution_result) or (None, result) if no changes possible
        """
        history = self._load_history(design_name)

        # Find current version
        current_version = None
        for v in history:
            if v.version_id == current_version_id:
                current_version = v
                break

        if current_version is None:
            return None, EvolutionResult(
                original_version=current_version_id,
                new_version="",
                changes_made=[],
                rules_applied=[],
                expected_improvement="Version not found"
            )

        # Find applicable rules
        new_params = current_version.design_params.copy()
        changes = []
        rules_applied = []

        failure_text = " ".join(failure_messages).lower()

        for rule in self.rules:
            if rule.failure_pattern.lower() in failure_text:
                # Check how many times this rule has been applied in history
                applications = sum(
                    1 for v in history
                    for change in v.changes_from_parent
                    if change.get("rule") == rule.failure_pattern
                )

                if applications >= rule.max_applications:
                    continue

                # Apply the rule
                param = rule.parameter
                if param in new_params:
                    old_value = new_params[param]

                    if rule.adjustment == "multiply":
                        new_value = old_value * rule.factor
                    elif rule.adjustment == "increase":
                        new_value = old_value + rule.factor
                    elif rule.adjustment == "decrease":
                        new_value = old_value - rule.factor
                    else:
                        continue

                    new_params[param] = new_value
                    changes.append({
                        "parameter": param,
                        "old_value": old_value,
                        "new_value": new_value,
                        "rule": rule.failure_pattern,
                        "reasoning": rule.reasoning
                    })
                    rules_applied.append(rule.failure_pattern)

        if not changes:
            return None, EvolutionResult(
                original_version=current_version_id,
                new_version="",
                changes_made=[],
                rules_applied=[],
                expected_improvement="No applicable evolution rules found for these failures"
            )

        # Create new version
        new_version = DesignVersion(
            version_id=self._generate_version_id(new_params),
            parent_id=current_version_id,
            design_params=new_params,
            created_at=datetime.now().isoformat(),
            changes_from_parent=changes
        )

        history.append(new_version)
        self._save_history(design_name, history)

        # Generate expected improvement description
        improvements = [f"{c['parameter']}: {c['reasoning']}" for c in changes]

        return new_version, EvolutionResult(
            original_version=current_version_id,
            new_version=new_version.version_id,
            changes_made=changes,
            rules_applied=rules_applied,
            expected_improvement="; ".join(improvements)
        )

    def get_design_lineage(self, design_name: str) -> List[DesignVersion]:
        """Get full lineage of a design."""
        return self._load_history(design_name)

    def get_version(self, design_name: str, version_id: str) -> Optional[DesignVersion]:
        """Get a specific version."""
        history = self._load_history(design_name)
        for v in history:
            if v.version_id == version_id:
                return v
        return None

    def get_latest_version(self, design_name: str) -> Optional[DesignVersion]:
        """Get the latest version of a design."""
        history = self._load_history(design_name)
        if history:
            return history[-1]
        return None

    def get_evolution_summary(self, design_name: str) -> Dict[str, Any]:
        """Get summary of design evolution."""
        history = self._load_history(design_name)

        if not history:
            return {"design_name": design_name, "versions": 0}

        passed_versions = [v for v in history if v.validation_passed]
        failed_versions = [v for v in history if v.validation_result and not v.validation_passed]

        all_rules = []
        for v in history:
            for change in v.changes_from_parent:
                if "rule" in change:
                    all_rules.append(change["rule"])

        return {
            "design_name": design_name,
            "versions": len(history),
            "passed": len(passed_versions),
            "failed": len(failed_versions),
            "untested": len(history) - len(passed_versions) - len(failed_versions),
            "rules_applied": list(set(all_rules)),
            "latest_version": history[-1].version_id,
            "latest_passed": passed_versions[-1].version_id if passed_versions else None
        }
