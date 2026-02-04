"""
Agent action models for Guardian Runtime.
"""
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Dict, Optional


class Decision(Enum):
    """Governor decision for an action."""
    ALLOW = "ALLOW"
    DENY = "DENY"
    REQUIRE_APPROVAL = "REQUIRE_APPROVAL"


@dataclass
class ActionRequest:
    """Request from a wrapped agent to perform an action."""
    action: str
    params: Dict[str, Any]
    agent_id: str
    session_id: str
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "params": self.params,
            "agent_id": self.agent_id,
            "session_id": self.session_id,
            "timestamp": self.timestamp,
        }


@dataclass
class ActionDecision:
    """Governor decision for a single action."""
    action: str
    params: Dict[str, Any]
    decision: Decision
    reason: Optional[str] = None
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "params": self.params,
            "decision": self.decision.value,
            "reason": self.reason,
            "timestamp": self.timestamp,
        }


@dataclass
class ActionOutcome:
    """Result of executing an action (after Governor approval)."""
    action: str
    params: Dict[str, Any]
    success: bool
    result: Optional[Any] = None
    error: Optional[str] = None
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "params": self.params,
            "success": self.success,
            "result": self.result,
            "error": self.error,
            "timestamp": self.timestamp,
        }
