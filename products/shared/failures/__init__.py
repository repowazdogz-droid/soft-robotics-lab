"""OMEGA failure taxonomy - 12 canonical failure types, no raw tracebacks to users."""
from .taxonomy import (
    Severity,
    FailureCode,
    Failure,
    create_failure,
    handle_exception,
    FAILURE_SEVERITY,
    FAILURE_MESSAGES,
    FAILURE_ACTIONS,
    FAILURE_TUTOR_TOPICS,
)

__all__ = [
    "Severity",
    "FailureCode",
    "Failure",
    "create_failure",
    "handle_exception",
    "FAILURE_SEVERITY",
    "FAILURE_MESSAGES",
    "FAILURE_ACTIONS",
    "FAILURE_TUTOR_TOPICS",
]
