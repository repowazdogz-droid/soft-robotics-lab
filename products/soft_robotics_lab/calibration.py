"""
Sim-to-Real Calibration Module

Enables the digital twin to "learn" from physical experiment data,
automatically refining simulation parameters to match real-world behavior.
"""

import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict

_DATA_DIR = Path(__file__).resolve().parent / "data" / "calibration"
_DATA_DIR.mkdir(parents=True, exist_ok=True)


@dataclass
class PhysicalTrial:
    """Record of a physical experiment"""
    trial_id: str
    timestamp: str
    gripper_id: str
    material: str

    # Test conditions
    object_type: str
    object_mass: float  # kg
    approach_angle: float  # degrees
    grip_force: float  # N

    # Results
    grasp_success: bool
    slip_detected: bool
    deformation_mm: float

    # Notes
    failure_mode: Optional[str] = None
    notes: Optional[str] = None


@dataclass
class CalibrationUpdate:
    """Parameter update based on physical trials"""
    parameter: str
    old_value: float
    new_value: float
    confidence: float
    trials_used: int
    timestamp: str


class Calibrator:
    """
    Calibration engine for sim-to-real parameter refinement.

    Workflow:
    1. Run simulation with current parameters
    2. Conduct physical trial
    3. Log trial results
    4. Calibrator computes parameter adjustments
    5. Update simulation, repeat
    """

    def __init__(self, gripper_id: str):
        self.gripper_id = gripper_id
        self.trials_path = _DATA_DIR / f"{gripper_id}_trials.json"
        self.calibration_path = _DATA_DIR / f"{gripper_id}_calibration.json"
        self.trials: List[PhysicalTrial] = self._load_trials()
        self.calibration_history: List[CalibrationUpdate] = self._load_calibration()

    def _load_trials(self) -> List[PhysicalTrial]:
        if self.trials_path.exists():
            data = json.loads(self.trials_path.read_text(encoding="utf-8"))
            return [PhysicalTrial(**t) for t in data]
        return []

    def _load_calibration(self) -> List[CalibrationUpdate]:
        if self.calibration_path.exists():
            data = json.loads(self.calibration_path.read_text(encoding="utf-8"))
            return [CalibrationUpdate(**c) for c in data]
        return []

    def _save(self) -> None:
        self.trials_path.write_text(
            json.dumps([asdict(t) for t in self.trials], indent=2),
            encoding="utf-8",
        )
        self.calibration_path.write_text(
            json.dumps([asdict(c) for c in self.calibration_history], indent=2),
            encoding="utf-8",
        )

    def log_trial(self, trial: PhysicalTrial) -> None:
        """Log a physical experiment trial"""
        self.trials.append(trial)
        self._save()

    def compute_calibration(self, sim_results: Dict) -> Dict[str, CalibrationUpdate]:
        """
        Compare simulation predictions with physical trials
        and compute parameter adjustments.

        Args:
            sim_results: Dict with predicted outcomes from simulation

        Returns:
            Dict of parameter updates
        """
        if len(self.trials) < 3:
            return {}  # Need minimum trials for statistical confidence

        updates: Dict[str, CalibrationUpdate] = {}

        # Compare grasp success rate
        physical_success_rate = sum(1 for t in self.trials if t.grasp_success) / len(self.trials)
        sim_success_rate = sim_results.get("success_rate", 0.5)

        if abs(physical_success_rate - sim_success_rate) > 0.1:
            # Significant discrepancy - adjust friction
            direction = 1 if physical_success_rate > sim_success_rate else -1
            old_friction = sim_results.get("friction", 0.9)
            new_friction = old_friction + direction * 0.05

            update = CalibrationUpdate(
                parameter="friction",
                old_value=old_friction,
                new_value=new_friction,
                confidence=min(len(self.trials) / 10, 1.0),
                trials_used=len(self.trials),
                timestamp=datetime.now().isoformat(),
            )
            updates["friction"] = update
            self.calibration_history.append(update)

        self._save()
        return updates

    def get_current_parameters(self) -> Dict:
        """Get the most recent calibrated parameters"""
        params: Dict[str, float] = {}
        for update in self.calibration_history:
            params[update.parameter] = update.new_value
        return params

    def get_confidence(self) -> float:
        """Overall calibration confidence based on trial count and consistency"""
        if not self.trials:
            return 0.0

        # Base confidence on trial count
        trial_confidence = min(len(self.trials) / 20, 1.0)

        # Reduce if recent trials show inconsistency
        if len(self.trials) >= 5:
            recent = self.trials[-5:]
            recent_success = sum(1 for t in recent if t.grasp_success) / 5
            consistency = 1 - abs(recent_success - 0.5) * 0.5  # Penalize extreme variance
        else:
            consistency = 0.5

        return trial_confidence * consistency
