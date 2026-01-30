"""
OMEGA Failure Taxonomy

12 canonical failure types with severity levels.
Rule: No raw exceptions reach the user.
"""
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from shared.id_generator import error_id


class Severity(Enum):
    INFO = "info"  # Show suggestion, continue
    WARN = "warn"  # Show warning, allow proceed with confirmation
    BLOCK = "block"  # Hard stop, must fix before continuing


class FailureCode(Enum):
    # Physics/Simulation
    PHYSICS_INSTABILITY = "PHYSICS_INSTABILITY"
    INSUFFICIENT_RESOLUTION = "INSUFFICIENT_RESOLUTION"

    # Geometry/Design
    GEOMETRY_SELF_INTERSECTION = "GEOMETRY_SELF_INTERSECTION"
    MATERIAL_OUT_OF_RANGE = "MATERIAL_OUT_OF_RANGE"

    # Validation
    UNSUPPORTED_ASSUMPTION = "UNSUPPORTED_ASSUMPTION"
    INVALID_INPUT_UNITS = "INVALID_INPUT_UNITS"
    GRASP_FAILURE = "GRASP_FAILURE"
    VALIDATION_TIMEOUT = "VALIDATION_TIMEOUT"

    # System
    MISSING_PARAMS = "MISSING_PARAMS"
    API_UNAVAILABLE = "API_UNAVAILABLE"
    PERMISSION_DENIED = "PERMISSION_DENIED"

    # Catch-all
    UNKNOWN = "UNKNOWN"


# Severity mapping
FAILURE_SEVERITY = {
    FailureCode.PHYSICS_INSTABILITY: Severity.BLOCK,
    FailureCode.MATERIAL_OUT_OF_RANGE: Severity.BLOCK,
    FailureCode.GEOMETRY_SELF_INTERSECTION: Severity.BLOCK,
    FailureCode.UNSUPPORTED_ASSUMPTION: Severity.BLOCK,
    FailureCode.INSUFFICIENT_RESOLUTION: Severity.WARN,
    FailureCode.INVALID_INPUT_UNITS: Severity.BLOCK,
    FailureCode.GRASP_FAILURE: Severity.WARN,
    FailureCode.VALIDATION_TIMEOUT: Severity.WARN,
    FailureCode.MISSING_PARAMS: Severity.BLOCK,
    FailureCode.API_UNAVAILABLE: Severity.WARN,
    FailureCode.PERMISSION_DENIED: Severity.INFO,
    FailureCode.UNKNOWN: Severity.WARN,
}

# Human-readable messages
FAILURE_MESSAGES = {
    FailureCode.PHYSICS_INSTABILITY: "Simulation became unstable (exploded or collapsed)",
    FailureCode.MATERIAL_OUT_OF_RANGE: "Material properties are outside valid bounds",
    FailureCode.GEOMETRY_SELF_INTERSECTION: "Design has parts that intersect themselves",
    FailureCode.UNSUPPORTED_ASSUMPTION: "A required assumption was violated",
    FailureCode.INSUFFICIENT_RESOLUTION: "Mesh or timestep is too coarse for accurate results",
    FailureCode.INVALID_INPUT_UNITS: "Input units don't match expected SI units",
    FailureCode.GRASP_FAILURE: "Object was not successfully grasped",
    FailureCode.VALIDATION_TIMEOUT: "Validation took too long and was stopped",
    FailureCode.MISSING_PARAMS: "Required parameters were not provided",
    FailureCode.API_UNAVAILABLE: "External service is not responding",
    FailureCode.PERMISSION_DENIED: "This action is not allowed in the current mode",
    FailureCode.UNKNOWN: "An unexpected error occurred",
}

# Suggested next actions
FAILURE_ACTIONS = {
    FailureCode.PHYSICS_INSTABILITY: [
        "Reduce timestep",
        "Check for unrealistic forces or masses",
        "Simplify the design",
    ],
    FailureCode.MATERIAL_OUT_OF_RANGE: [
        "Use a material from the calibrated database",
        "Check stiffness and damping values",
    ],
    FailureCode.GEOMETRY_SELF_INTERSECTION: [
        "Increase spacing between components",
        "Check joint limits",
    ],
    FailureCode.UNSUPPORTED_ASSUMPTION: [
        "Check input format matches contract",
        "Review assumptions in component contract",
    ],
    FailureCode.INSUFFICIENT_RESOLUTION: [
        "Increase mesh density",
        "Reduce timestep",
        "Results may still be usable for rough estimates",
    ],
    FailureCode.INVALID_INPUT_UNITS: [
        "Ensure all values are in SI units (meters, kg, seconds)",
        "Check for mm vs m confusion",
    ],
    FailureCode.GRASP_FAILURE: [
        "Adjust gripper size relative to object",
        "Try different grasp approach angle",
        "Increase grip force",
    ],
    FailureCode.VALIDATION_TIMEOUT: [
        "Simplify the design",
        "Reduce simulation duration",
        "Try again - may be temporary",
    ],
    FailureCode.MISSING_PARAMS: [
        "Check required fields in the request",
        "See API documentation",
    ],
    FailureCode.API_UNAVAILABLE: [
        "Check if the service is running",
        "Try again in a moment",
        "Check network connection",
    ],
    FailureCode.PERMISSION_DENIED: [
        "Switch to the appropriate mode",
        "Check user permissions",
    ],
    FailureCode.UNKNOWN: [
        "Check logs for details",
        "Try again",
        "Report this issue if it persists",
    ],
}

# Tutor topic links (for auto-link to learning)
FAILURE_TUTOR_TOPICS = {
    FailureCode.PHYSICS_INSTABILITY: "physics simulation stability",
    FailureCode.MATERIAL_OUT_OF_RANGE: "material properties soft robotics",
    FailureCode.GEOMETRY_SELF_INTERSECTION: "collision detection robotics",
    FailureCode.GRASP_FAILURE: "grasp planning manipulation",
    FailureCode.INVALID_INPUT_UNITS: "SI units engineering",
}


@dataclass
class Failure:
    """Structured failure object"""

    code: FailureCode
    error_id: str
    message: str
    severity: Severity
    actions: List[str]
    tutor_topic: Optional[str] = None
    details: Optional[str] = None  # Technical details (not shown to user by default)

    def to_dict(self):
        from shared.tutor_links import get_tutor_link

        tutor_link = get_tutor_link(self.tutor_topic) if self.tutor_topic else None
        return {
            "error_id": self.error_id,
            "code": self.code.value,
            "message": self.message,
            "severity": self.severity.value,
            "actions": self.actions,
            "tutor_topic": self.tutor_topic,
            "tutor_link": tutor_link,
        }

    def to_user_message(self) -> str:
        """Format for user display"""
        lines = [
            f"❌ {self.message}",
            "",
            "What you can do:",
        ]
        for i, action in enumerate(self.actions, 1):
            lines.append(f"  {i}. {action}")

        if self.tutor_topic:
            lines.append("")
            lines.append(f"📚 Want to learn more? Ask Tutor about: {self.tutor_topic}")

        return "\n".join(lines)


def create_failure(code: FailureCode, details: str = None) -> Failure:
    """Create a structured failure from a failure code"""
    return Failure(
        code=code,
        error_id=error_id(code.value),
        message=FAILURE_MESSAGES.get(code, "An error occurred"),
        severity=FAILURE_SEVERITY.get(code, Severity.WARN),
        actions=FAILURE_ACTIONS.get(code, ["Try again"]),
        tutor_topic=FAILURE_TUTOR_TOPICS.get(code),
        details=details,
    )


def handle_exception(e: Exception, context: str = "") -> Failure:
    """Convert a raw exception to a structured Failure"""
    # Map common exceptions to failure codes
    error_str = str(e).lower()

    if "timeout" in error_str:
        code = FailureCode.VALIDATION_TIMEOUT
    elif "nan" in error_str or "inf" in error_str or "unstable" in error_str:
        code = FailureCode.PHYSICS_INSTABILITY
    elif "collision" in error_str or "intersect" in error_str:
        code = FailureCode.GEOMETRY_SELF_INTERSECTION
    elif "material" in error_str or "stiffness" in error_str:
        code = FailureCode.MATERIAL_OUT_OF_RANGE
    elif "missing" in error_str or "required" in error_str:
        code = FailureCode.MISSING_PARAMS
    elif "connection" in error_str or "unavailable" in error_str:
        code = FailureCode.API_UNAVAILABLE
    elif "permission" in error_str or "denied" in error_str:
        code = FailureCode.PERMISSION_DENIED
    else:
        code = FailureCode.UNKNOWN

    return create_failure(code, details=f"{context}: {str(e)}" if context else str(e))
