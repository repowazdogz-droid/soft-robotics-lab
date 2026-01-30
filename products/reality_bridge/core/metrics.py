"""
Reality Bridge — Performance metrics (validation time, counts, trends).
Uses database.get_stats(), get_recent_validations(), get_failure_distribution().
Expose for /stats and dashboard.
"""

from typing import Any, Dict, List

from . import database


def get_metrics() -> Dict[str, Any]:
    """Return full metrics dict: total_validations, pass_rate, avg_validation_ms, today, this_week, failure_distribution."""
    return database.get_stats()


def get_recent_validations(limit: int = 20) -> List[Dict[str, Any]]:
    """Recent validations for dashboard."""
    return database.get_recent_validations(limit=limit)


def get_failure_distribution(days: int = 7) -> Dict[str, float]:
    """Failure type distribution for the last N days."""
    return database.get_failure_distribution(limit_days=days)


def get_validation_history(design_id: str, limit: int = 50) -> List[Dict[str, Any]]:
    """All validations for a design (by artifact_id or design_hash)."""
    return database.get_validation_history(design_id, limit=limit)
