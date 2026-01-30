"""
OMEGA Trust Score
Single metric (0-100) measuring system trustworthiness.
"""
from dataclasses import dataclass
from typing import Dict
import json
from pathlib import Path


@dataclass
class TrustMetrics:
    first_run_success: float = 75.0
    zero_traceback_rate: float = 90.0
    reproducibility_rate: float = 85.0
    failure_clarity_rate: float = 80.0
    uptime_rate: float = 95.0

    def calculate_trust_score(self) -> float:
        score = (
            0.3 * self.first_run_success
            + 0.2 * self.zero_traceback_rate
            + 0.2 * self.reproducibility_rate
            + 0.2 * self.failure_clarity_rate
            + 0.1 * self.uptime_rate
        )
        return round(score, 1)

    def to_dict(self) -> Dict:
        return {
            "first_run_success": self.first_run_success,
            "zero_traceback_rate": self.zero_traceback_rate,
            "reproducibility_rate": self.reproducibility_rate,
            "failure_clarity_rate": self.failure_clarity_rate,
            "uptime_rate": self.uptime_rate,
            "trust_score": self.calculate_trust_score(),
        }


class TrustScoreTracker:
    def __init__(self, data_path: Path = None):
        if data_path is None:
            data_path = Path(__file__).parent / "trust_data.json"
        self.data_path = Path(data_path)
        self.metrics = self._load()

    def _load(self) -> TrustMetrics:
        if self.data_path.exists():
            try:
                data = json.loads(self.data_path.read_text(encoding="utf-8"))
                return TrustMetrics(**{k: v for k, v in data.items() if k != "trust_score"})
            except Exception:
                pass
        return TrustMetrics()

    def save(self):
        self.data_path.parent.mkdir(parents=True, exist_ok=True)
        self.data_path.write_text(json.dumps(self.metrics.to_dict(), indent=2), encoding="utf-8")

    def get_score(self) -> float:
        return self.metrics.calculate_trust_score()

    def record_run(self, had_traceback: bool):
        current = self.metrics.zero_traceback_rate
        new_val = 0.0 if had_traceback else 100.0
        self.metrics.zero_traceback_rate = round((current * 0.9) + (new_val * 0.1), 1)
        self.save()


def get_trust_score() -> float:
    return TrustScoreTracker().get_score()


def get_trust_metrics() -> Dict:
    return TrustScoreTracker().metrics.to_dict()
