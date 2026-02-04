"""
Reality Gap Tracking - Measure and reduce sim-to-real gap

Track:
- Simulation metrics vs real-world metrics
- Gap per gripper/task combination
- Gap reduction over time
- Calibration recommendations
"""
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any
from datetime import datetime
from pathlib import Path
import json

# Data storage
_DATA_DIR = Path(__file__).parent.parent / "data"
_GAP_FILE = _DATA_DIR / "reality_gap.json"


@dataclass
class SimMetrics:
    """Metrics from simulation."""
    success_rate: float
    avg_execution_time: float
    avg_grip_force: float
    avg_position_error: float
    episodes: int
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

    def to_dict(self) -> dict:
        return {
            "success_rate": self.success_rate,
            "avg_execution_time": self.avg_execution_time,
            "avg_grip_force": self.avg_grip_force,
            "avg_position_error": self.avg_position_error,
            "episodes": self.episodes,
            "timestamp": self.timestamp
        }


@dataclass
class RealMetrics:
    """Metrics from real-world testing."""
    success_rate: float
    avg_execution_time: float
    avg_grip_force: Optional[float] = None  # May not have force sensor
    avg_position_error: Optional[float] = None  # May not have precise tracking
    trials: int = 0
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    notes: str = ""

    def to_dict(self) -> dict:
        return {
            "success_rate": self.success_rate,
            "avg_execution_time": self.avg_execution_time,
            "avg_grip_force": self.avg_grip_force,
            "avg_position_error": self.avg_position_error,
            "trials": self.trials,
            "timestamp": self.timestamp,
            "notes": self.notes
        }


@dataclass
class GapMetrics:
    """Computed gap between sim and real."""
    success_rate_gap: float  # sim - real (positive = sim overestimates)
    execution_time_gap: float  # real - sim (positive = real is slower)
    grip_force_gap: Optional[float] = None
    position_error_gap: Optional[float] = None
    overall_gap: float = 0.0
    severity: str = "low"  # "low", "medium", "high", "critical"

    def to_dict(self) -> dict:
        return {
            "success_rate_gap": self.success_rate_gap,
            "execution_time_gap": self.execution_time_gap,
            "grip_force_gap": self.grip_force_gap,
            "position_error_gap": self.position_error_gap,
            "overall_gap": self.overall_gap,
            "severity": self.severity
        }


@dataclass
class GapRecord:
    """Single record of sim vs real comparison."""
    id: str
    gripper_id: str
    task_id: str
    policy_id: str
    sim_metrics: SimMetrics
    real_metrics: RealMetrics
    gap_metrics: GapMetrics
    calibration_applied: List[str] = field(default_factory=list)
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "gripper_id": self.gripper_id,
            "task_id": self.task_id,
            "policy_id": self.policy_id,
            "sim_metrics": self.sim_metrics.to_dict(),
            "real_metrics": self.real_metrics.to_dict(),
            "gap_metrics": self.gap_metrics.to_dict(),
            "calibration_applied": self.calibration_applied,
            "created_at": self.created_at
        }


@dataclass
class CalibrationRecommendation:
    """Recommendation to reduce gap."""
    parameter: str
    current_value: Any
    recommended_value: Any
    expected_gap_reduction: float
    confidence: float
    reasoning: str
    priority: int  # 1 = highest


def _rec_success_rate_high(sim: SimMetrics, real: RealMetrics) -> CalibrationRecommendation:
    return CalibrationRecommendation(
        parameter="domain_randomization.position",
        current_value=0.02,
        recommended_value=0.05,
        expected_gap_reduction=0.1,
        confidence=0.7,
        reasoning="High success rate gap suggests sim is too easy. Increase position randomization.",
        priority=1
    )


def _rec_friction(sim: SimMetrics, real: RealMetrics) -> CalibrationRecommendation:
    return CalibrationRecommendation(
        parameter="friction_coefficient",
        current_value=1.0,
        recommended_value=0.7,
        expected_gap_reduction=0.08,
        confidence=0.6,
        reasoning="Reduce friction coefficient to better match real-world grip difficulty.",
        priority=2
    )


def _rec_actuator_delay(sim: SimMetrics, real: RealMetrics) -> CalibrationRecommendation:
    return CalibrationRecommendation(
        parameter="actuator_delay",
        current_value=0.0,
        recommended_value=0.1,
        expected_gap_reduction=0.3,
        confidence=0.8,
        reasoning="Real execution slower. Add actuator delay to simulation.",
        priority=1
    )


def _rec_control_frequency(sim: SimMetrics, real: RealMetrics) -> CalibrationRecommendation:
    return CalibrationRecommendation(
        parameter="control_frequency",
        current_value=100,
        recommended_value=50,
        expected_gap_reduction=0.15,
        confidence=0.6,
        reasoning="Reduce control frequency to match real-world controller.",
        priority=2
    )


def _rec_actuator_gain(sim: SimMetrics, real: RealMetrics) -> CalibrationRecommendation:
    # Use sim vs real to decide direction: reduce gain if sim force > real
    rec_val = 0.8 if (real.avg_grip_force is not None and sim.avg_grip_force > real.avg_grip_force) else 1.2
    return CalibrationRecommendation(
        parameter="actuator_gain",
        current_value=1.0,
        recommended_value=rec_val,
        expected_gap_reduction=0.5,
        confidence=0.7,
        reasoning="Adjust actuator gain to match real force output.",
        priority=1
    )


def _rec_joint_damping(sim: SimMetrics, real: RealMetrics) -> CalibrationRecommendation:
    return CalibrationRecommendation(
        parameter="joint_damping",
        current_value=0.1,
        recommended_value=0.2,
        expected_gap_reduction=0.005,
        confidence=0.6,
        reasoning="Increase joint damping to reduce oscillation and improve position accuracy.",
        priority=2
    )


# Calibration rules based on gap patterns
CALIBRATION_RULES = {
    "success_rate_gap_high": [
        {"condition": lambda gap: gap.success_rate_gap > 0.15, "recommendation": _rec_success_rate_high},
        {"condition": lambda gap: gap.success_rate_gap > 0.1, "recommendation": _rec_friction},
    ],
    "execution_time_gap_high": [
        {"condition": lambda gap: gap.execution_time_gap > 0.5, "recommendation": _rec_actuator_delay},
        {"condition": lambda gap: gap.execution_time_gap > 0.3, "recommendation": _rec_control_frequency},
    ],
    "grip_force_gap_high": [
        {"condition": lambda gap: gap.grip_force_gap is not None and abs(gap.grip_force_gap) > 2.0, "recommendation": _rec_actuator_gain},
    ],
    "position_error_gap_high": [
        {"condition": lambda gap: gap.position_error_gap is not None and gap.position_error_gap > 0.01, "recommendation": _rec_joint_damping},
    ],
}


def compute_gap(sim: SimMetrics, real: RealMetrics) -> GapMetrics:
    """Compute gap metrics between sim and real."""
    success_gap = sim.success_rate - real.success_rate
    time_gap = real.avg_execution_time - sim.avg_execution_time

    force_gap = None
    if real.avg_grip_force is not None:
        force_gap = sim.avg_grip_force - real.avg_grip_force

    position_gap = None
    if real.avg_position_error is not None:
        position_gap = real.avg_position_error - sim.avg_position_error

    # Compute overall gap (weighted)
    gaps = [abs(success_gap) * 2]  # Success rate weighted higher
    gaps.append(abs(time_gap) / 2)  # Normalize time gap
    if force_gap is not None:
        gaps.append(abs(force_gap) / 10)  # Normalize force gap
    if position_gap is not None:
        gaps.append(abs(position_gap) * 100)  # Normalize position gap

    overall = sum(gaps) / len(gaps)

    # Determine severity
    if overall < 0.1:
        severity = "low"
    elif overall < 0.2:
        severity = "medium"
    elif overall < 0.35:
        severity = "high"
    else:
        severity = "critical"

    return GapMetrics(
        success_rate_gap=success_gap,
        execution_time_gap=time_gap,
        grip_force_gap=force_gap,
        position_error_gap=position_gap,
        overall_gap=overall,
        severity=severity
    )


def generate_calibration_recommendations(
    gap: GapMetrics,
    sim: SimMetrics,
    real: RealMetrics
) -> List[CalibrationRecommendation]:
    """Generate recommendations to reduce the gap."""
    recommendations = []

    for rule_category, rules in CALIBRATION_RULES.items():
        for rule in rules:
            try:
                if rule["condition"](gap):
                    rec = rule["recommendation"](sim, real)
                    recommendations.append(rec)
            except Exception:
                pass

    # Sort by priority
    recommendations.sort(key=lambda r: r.priority)

    return recommendations


class RealityGapTracker:
    """Track and manage reality gap over time."""

    def __init__(self):
        self.records: List[GapRecord] = []
        self._load()

    def _load(self) -> None:
        """Load existing records."""
        _DATA_DIR.mkdir(parents=True, exist_ok=True)
        if _GAP_FILE.exists():
            try:
                data = json.loads(_GAP_FILE.read_text(encoding="utf-8"))
                for r in data.get("records", []):
                    sim = SimMetrics(**r["sim_metrics"])
                    real = RealMetrics(**r["real_metrics"])
                    gap = GapMetrics(**r["gap_metrics"])
                    record = GapRecord(
                        id=r["id"],
                        gripper_id=r["gripper_id"],
                        task_id=r["task_id"],
                        policy_id=r["policy_id"],
                        sim_metrics=sim,
                        real_metrics=real,
                        gap_metrics=gap,
                        calibration_applied=r.get("calibration_applied", []),
                        created_at=r.get("created_at", datetime.now().isoformat())
                    )
                    self.records.append(record)
            except Exception:
                pass

    def _save(self) -> None:
        """Save records."""
        _DATA_DIR.mkdir(parents=True, exist_ok=True)
        data = {
            "records": [r.to_dict() for r in self.records],
            "updated_at": datetime.now().isoformat()
        }
        _GAP_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def record_gap(
        self,
        gripper_id: str,
        task_id: str,
        policy_id: str,
        sim_metrics: SimMetrics,
        real_metrics: RealMetrics,
        calibration_applied: Optional[List[str]] = None
    ) -> GapRecord:
        """Record a new sim vs real comparison."""
        import hashlib

        gap_metrics = compute_gap(sim_metrics, real_metrics)

        record_id = hashlib.md5(
            f"{gripper_id}{task_id}{policy_id}{datetime.now().isoformat()}".encode()
        ).hexdigest()[:12]

        record = GapRecord(
            id=f"GAP-{record_id.upper()}",
            gripper_id=gripper_id,
            task_id=task_id,
            policy_id=policy_id,
            sim_metrics=sim_metrics,
            real_metrics=real_metrics,
            gap_metrics=gap_metrics,
            calibration_applied=calibration_applied or []
        )

        self.records.append(record)
        self._save()

        return record

    def get_gap_history(
        self,
        gripper_id: Optional[str] = None,
        task_id: Optional[str] = None,
        policy_id: Optional[str] = None
    ) -> List[GapRecord]:
        """Get gap history with optional filters."""
        results = self.records

        if gripper_id:
            results = [r for r in results if r.gripper_id == gripper_id]
        if task_id:
            results = [r for r in results if r.task_id == task_id]
        if policy_id:
            results = [r for r in results if r.policy_id == policy_id]

        return sorted(results, key=lambda r: r.created_at, reverse=True)

    def get_gap_trend(
        self,
        gripper_id: str,
        task_id: str
    ) -> Dict[str, Any]:
        """Get gap trend over time for a gripper/task pair."""
        history = self.get_gap_history(gripper_id=gripper_id, task_id=task_id)

        if not history:
            return {"trend": "no_data", "records": 0}

        if len(history) < 2:
            return {
                "trend": "insufficient_data",
                "records": len(history),
                "latest_gap": history[0].gap_metrics.overall_gap
            }

        # Compare latest to oldest
        latest = history[0].gap_metrics.overall_gap
        oldest = history[-1].gap_metrics.overall_gap

        if latest < oldest * 0.8:
            trend = "improving"
        elif latest > oldest * 1.2:
            trend = "degrading"
        else:
            trend = "stable"

        return {
            "trend": trend,
            "records": len(history),
            "latest_gap": latest,
            "oldest_gap": oldest,
            "improvement": (oldest - latest) / oldest if oldest > 0 else 0,
            "timeline": [
                {"date": r.created_at, "gap": r.gap_metrics.overall_gap}
                for r in reversed(history)
            ]
        }

    def get_recommendations(
        self,
        gripper_id: Optional[str] = None,
        task_id: Optional[str] = None
    ) -> List[CalibrationRecommendation]:
        """Get calibration recommendations based on latest gaps."""
        history = self.get_gap_history(gripper_id=gripper_id, task_id=task_id)

        if not history:
            return []

        latest = history[0]
        return generate_calibration_recommendations(
            latest.gap_metrics,
            latest.sim_metrics,
            latest.real_metrics
        )

    def get_summary(self) -> Dict[str, Any]:
        """Get overall summary of reality gap tracking."""
        if not self.records:
            return {
                "total_records": 0,
                "grippers_tracked": 0,
                "tasks_tracked": 0,
                "avg_gap": None,
                "worst_gap": None,
                "best_gap": None,
                "severity_distribution": {"low": 0, "medium": 0, "high": 0, "critical": 0}
            }

        grippers = set(r.gripper_id for r in self.records)
        tasks = set(r.task_id for r in self.records)
        gaps = [r.gap_metrics.overall_gap for r in self.records]

        return {
            "total_records": len(self.records),
            "grippers_tracked": len(grippers),
            "tasks_tracked": len(tasks),
            "avg_gap": sum(gaps) / len(gaps),
            "worst_gap": max(gaps),
            "best_gap": min(gaps),
            "severity_distribution": {
                "low": sum(1 for r in self.records if r.gap_metrics.severity == "low"),
                "medium": sum(1 for r in self.records if r.gap_metrics.severity == "medium"),
                "high": sum(1 for r in self.records if r.gap_metrics.severity == "high"),
                "critical": sum(1 for r in self.records if r.gap_metrics.severity == "critical")
            }
        }
