"""
Audit bundle generation. Every session creates a bundle; immutable once closed.
"""
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from ..schemas.bundle import AuditBundle
from ..schemas.action import ActionRequest, ActionDecision, ActionOutcome


def create_bundle(agent_id: str, session_id: str, metadata: Optional[Dict[str, Any]] = None) -> AuditBundle:
    """Create a new audit bundle for a session."""
    return AuditBundle(
        agent_id=agent_id,
        session_id=session_id,
        started_at=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        metadata=metadata or {},
    )


def record_action(bundle: AuditBundle, request: ActionRequest) -> None:
    """Record an action request in the bundle."""
    bundle.add_action(request.to_dict())


def record_decision(bundle: AuditBundle, decision: ActionDecision) -> None:
    """Record a governor decision in the bundle."""
    bundle.add_decision(decision.to_dict())


def record_outcome(bundle: AuditBundle, outcome: ActionOutcome) -> None:
    """Record an action outcome in the bundle."""
    bundle.add_outcome(outcome.to_dict())


def close_bundle(bundle: AuditBundle) -> None:
    """Close the bundle (immutable from then on)."""
    bundle.close()
