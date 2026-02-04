"""
Audit bundle Pydantic-compatible models (using dataclasses for minimal deps).
Immutable once closed.
"""
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional


@dataclass(frozen=False)
class AuditBundle:
    """Portable trust artifact: session + all actions, decisions, outcomes."""
    agent_id: str
    session_id: str
    started_at: str
    closed_at: Optional[str] = None
    actions: List[Dict[str, Any]] = field(default_factory=list)
    decisions: List[Dict[str, Any]] = field(default_factory=list)
    outcomes: List[Dict[str, Any]] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    _closed: bool = field(default=False, repr=False)

    def add_action(self, action: Dict[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("Audit bundle is closed; cannot add entries")
        self.actions.append(action)

    def add_decision(self, decision: Dict[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("Audit bundle is closed; cannot add entries")
        self.decisions.append(decision)

    def add_outcome(self, outcome: Dict[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("Audit bundle is closed; cannot add entries")
        self.outcomes.append(outcome)

    def close(self) -> None:
        self._closed = True
        object.__setattr__(self, "closed_at", datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"))

    @property
    def is_closed(self) -> bool:
        return self._closed

    def to_dict(self) -> Dict[str, Any]:
        return {
            "agent_id": self.agent_id,
            "session_id": self.session_id,
            "started_at": self.started_at,
            "closed_at": self.closed_at,
            "actions": list(self.actions),
            "decisions": list(self.decisions),
            "outcomes": list(self.outcomes),
            "metadata": dict(self.metadata),
        }

    def json(self, indent: Optional[int] = 2) -> str:
        import json
        return json.dumps(self.to_dict(), indent=indent)
