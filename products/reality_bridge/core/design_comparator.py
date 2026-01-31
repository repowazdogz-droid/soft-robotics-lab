"""
Design Comparator - Compare two designs for a given task.

Input: two designs + task.
Output: which is better and why.
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from enum import Enum


class ComparisonMetric(Enum):
    STABILITY = "stability"
    KINEMATICS = "kinematics"
    DYNAMICS = "dynamics"
    MASS = "mass"
    GRIP_FORCE = "grip_force"
    REACH = "reach"
    SPEED = "speed"
    ROBUSTNESS = "robustness"


@dataclass
class MetricComparison:
    metric: ComparisonMetric
    design_a_score: float
    design_b_score: float
    winner: str
    difference: float
    significance: str
    notes: str


@dataclass
class ComparisonReport:
    design_a_id: str
    design_b_id: str
    task: str
    overall_winner: str
    confidence: float
    metric_comparisons: List[MetricComparison]
    design_a_strengths: List[str]
    design_b_strengths: List[str]
    recommendation: str
    detailed_analysis: str


# Map Reality Bridge test names to comparison metrics
_TEST_TO_METRIC = {
    "STABILITY_TEST": "stability",
    "KINEMATICS_TEST": "kinematics",
    "DYNAMICS_TEST": "dynamics",
    "MASS_PROPERTIES_TEST": "mass",
    "SELF_COLLISION_TEST": "robustness",
    "LOAD_TEST": "robustness",
}


def extract_metrics(validation_result: Dict[str, Any]) -> Dict[str, float]:
    """Extract normalized metrics from validation result (Reality Bridge to_dict or API payload)."""
    metrics: Dict[str, float] = {}
    overall = validation_result.get("score", 0.5)
    tests = validation_result.get("tests", {})

    for test_name, test_result in tests.items():
        if not isinstance(test_result, dict):
            continue
        passed = test_result.get("passed", False)
        score = 1.0 if passed else 0.0
        key = _TEST_TO_METRIC.get(test_name)
        if key:
            metrics[key] = score

    details = validation_result.get("metrics", {}) or {}
    if details.get("mass_kg") is not None:
        mass = float(details["mass_kg"])
        metrics["mass"] = min(mass / 10.0, 1.0) if metrics.get("mass") is None else metrics["mass"]
    if "grip_force" in details:
        metrics["grip_force"] = min(float(details["grip_force"]) / 20.0, 1.0)
    if "reach" in details:
        metrics["reach"] = min(float(details["reach"]) / 0.5, 1.0)
    if "response_time" in details:
        metrics["speed"] = max(0, 1 - float(details["response_time"]))

    for m in ComparisonMetric:
        if m.value not in metrics:
            metrics[m.value] = overall
    return metrics


def compare_metric(
    metric: ComparisonMetric, score_a: float, score_b: float
) -> MetricComparison:
    """Compare a single metric between two designs."""
    diff = score_a - score_b
    abs_diff = abs(diff)
    if abs_diff < 0.05:
        winner = "TIE"
    elif diff > 0:
        winner = "A"
    else:
        winner = "B"
    if abs_diff < 0.05:
        significance = "negligible"
    elif abs_diff < 0.15:
        significance = "marginal"
    else:
        significance = "significant"
    pct_diff = abs_diff * 100
    if winner == "TIE":
        notes = f"Both designs perform similarly on {metric.value}"
    else:
        better = "A" if winner == "A" else "B"
        notes = f"Design {better} is {significance}ly better at {metric.value} ({pct_diff:.0f}% difference)"
    return MetricComparison(
        metric=metric,
        design_a_score=score_a,
        design_b_score=score_b,
        winner=winner,
        difference=pct_diff,
        significance=significance,
        notes=notes,
    )


def get_task_weights(task: str) -> Dict[str, float]:
    """Get metric weights based on task."""
    weights = {
        "stability": 1.0,
        "kinematics": 1.0,
        "dynamics": 1.0,
        "mass": 1.0,
        "grip_force": 1.0,
        "reach": 1.0,
        "speed": 1.0,
        "robustness": 1.0,
    }
    task_lower = task.lower()
    if "egg" in task_lower or "delicate" in task_lower or "fragile" in task_lower:
        weights["grip_force"] = 0.5
        weights["dynamics"] = 2.0
        weights["robustness"] = 1.5
    elif "heavy" in task_lower or "lift" in task_lower:
        weights["grip_force"] = 2.0
        weights["stability"] = 2.0
        weights["mass"] = 0.5
    elif "fast" in task_lower or "speed" in task_lower:
        weights["speed"] = 2.0
        weights["dynamics"] = 1.5
    elif "reach" in task_lower or "far" in task_lower:
        weights["reach"] = 2.0
        weights["kinematics"] = 1.5
    elif "surgical" in task_lower or "precision" in task_lower:
        weights["kinematics"] = 2.0
        weights["dynamics"] = 2.0
        weights["robustness"] = 2.0
    return weights


def generate_recommendation(
    winner: str,
    design_a_id: str,
    design_b_id: str,
    strengths_a: List[str],
    strengths_b: List[str],
    task: str,
) -> str:
    """Generate a recommendation based on comparison."""
    if winner == "TIE":
        return (
            f"Both designs are comparable for {task}. "
            f"Choose based on other factors: "
            f"{design_a_id} excels at {', '.join(strengths_a) if strengths_a else 'nothing specific'}, "
            f"while {design_b_id} excels at {', '.join(strengths_b) if strengths_b else 'nothing specific'}."
        )
    winner_id = design_a_id if winner == "A" else design_b_id
    loser_id = design_b_id if winner == "A" else design_a_id
    winner_strengths = strengths_a if winner == "A" else strengths_b
    return (
        f"**{winner_id}** is recommended for {task}. "
        f"Key advantages: {', '.join(winner_strengths) if winner_strengths else 'overall performance'}. "
        f"Consider {loser_id} if other factors (cost, manufacturing) are prioritized."
    )


def generate_detailed_analysis(
    comparisons: List[MetricComparison],
    task: str,
    design_a_id: str,
    design_b_id: str,
) -> str:
    """Generate detailed analysis text."""
    lines = [f"## Detailed Comparison for {task}\n"]
    significant = [c for c in comparisons if c.significance == "significant"]
    if significant:
        lines.append("### Significant Differences\n")
        for c in significant:
            lines.append(f"- **{c.metric.value.title()}**: {c.notes}")
        lines.append("")
    marginal = [c for c in comparisons if c.significance == "marginal"]
    if marginal:
        lines.append("### Marginal Differences\n")
        for c in marginal:
            lines.append(f"- **{c.metric.value.title()}**: {c.notes}")
        lines.append("")
    similar = [c for c in comparisons if c.significance == "negligible"]
    if similar:
        lines.append("### Similar Performance\n")
        lines.append(f"Both designs perform similarly on: {', '.join(c.metric.value for c in similar)}")
    return "\n".join(lines)


def compare_designs(
    design_a: Dict[str, Any],
    design_b: Dict[str, Any],
    validation_a: Dict[str, Any],
    validation_b: Dict[str, Any],
    task: str = "general",
) -> ComparisonReport:
    """
    Compare two designs based on their validation results.

    Args:
        design_a: First design dict (id, mjcf, etc.).
        design_b: Second design dict.
        validation_a: Validation result for design A (dict from to_dict or API).
        validation_b: Validation result for design B.
        task: Task context (e.g., "pick_egg", "heavy_lift").

    Returns:
        ComparisonReport with detailed analysis.
    """
    design_a_id = design_a.get("id", "Design A")
    design_b_id = design_b.get("id", "Design B")
    metrics_a = extract_metrics(validation_a)
    metrics_b = extract_metrics(validation_b)
    metric_comparisons: List[MetricComparison] = []
    for metric in ComparisonMetric:
        score_a = metrics_a.get(metric.value, 0.5)
        score_b = metrics_b.get(metric.value, 0.5)
        comparison = compare_metric(metric, score_a, score_b)
        metric_comparisons.append(comparison)

    weights = get_task_weights(task)
    weighted_a = sum(metrics_a.get(m.value, 0.5) * weights.get(m.value, 1.0) for m in ComparisonMetric)
    weighted_b = sum(metrics_b.get(m.value, 0.5) * weights.get(m.value, 1.0) for m in ComparisonMetric)
    total_weight = sum(weights.values())
    normalized_a = weighted_a / total_weight if total_weight else 0.5
    normalized_b = weighted_b / total_weight if total_weight else 0.5

    diff = abs(normalized_a - normalized_b)
    if diff < 0.05:
        overall_winner = "TIE"
        confidence = 0.5
    elif normalized_a > normalized_b:
        overall_winner = "A"
        confidence = 0.5 + (diff * 2)
    else:
        overall_winner = "B"
        confidence = 0.5 + (diff * 2)
    confidence = min(confidence, 0.95)

    design_a_strengths = [
        m.metric.value for m in metric_comparisons if m.winner == "A" and m.significance != "negligible"
    ]
    design_b_strengths = [
        m.metric.value for m in metric_comparisons if m.winner == "B" and m.significance != "negligible"
    ]
    recommendation = generate_recommendation(
        overall_winner, design_a_id, design_b_id, design_a_strengths, design_b_strengths, task
    )
    detailed_analysis = generate_detailed_analysis(
        metric_comparisons, task, design_a_id, design_b_id
    )

    return ComparisonReport(
        design_a_id=design_a_id,
        design_b_id=design_b_id,
        task=task,
        overall_winner=overall_winner,
        confidence=confidence,
        metric_comparisons=metric_comparisons,
        design_a_strengths=design_a_strengths,
        design_b_strengths=design_b_strengths,
        recommendation=recommendation,
        detailed_analysis=detailed_analysis,
    )


def format_comparison_table(comparisons: List[MetricComparison]) -> str:
    """Format comparisons as a markdown table."""
    lines = [
        "| Metric | Design A | Design B | Winner | Difference |",
        "|--------|----------|----------|--------|------------|",
    ]
    for c in comparisons:
        winner_str = c.winner if c.winner != "TIE" else "—"
        lines.append(
            f"| {c.metric.value.title()} | {c.design_a_score:.0%} | {c.design_b_score:.0%} | {winner_str} | {c.difference:.0f}% |"
        )
    return "\n".join(lines)
