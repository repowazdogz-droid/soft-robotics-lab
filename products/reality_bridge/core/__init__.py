"""
Reality Bridge - Physics validation API.
"""

from .loader import load_mjcf, load_urdf, detect_format
from .validator import PhysicsValidator, ValidationResult, TestResult
from .analyzer import find_weak_points, estimate_failure_modes, suggest_improvements
from .reporter import to_dict, to_markdown, to_html
from .prescriptive_fixer import (
    FixType,
    PrescriptiveFix,
    FixReport,
    generate_prescriptive_fixes,
    format_fix_for_display,
    generate_mjcf_patch,
)
from .design_comparator import (
    ComparisonMetric,
    MetricComparison,
    ComparisonReport,
    compare_designs,
    extract_metrics,
    format_comparison_table,
)

__all__ = [
    "load_mjcf",
    "load_urdf",
    "detect_format",
    "PhysicsValidator",
    "ValidationResult",
    "TestResult",
    "find_weak_points",
    "estimate_failure_modes",
    "suggest_improvements",
    "to_dict",
    "to_markdown",
    "to_html",
    "FixType",
    "PrescriptiveFix",
    "FixReport",
    "generate_prescriptive_fixes",
    "format_fix_for_display",
    "generate_mjcf_patch",
    "ComparisonMetric",
    "MetricComparison",
    "ComparisonReport",
    "compare_designs",
    "extract_metrics",
    "format_comparison_table",
]
