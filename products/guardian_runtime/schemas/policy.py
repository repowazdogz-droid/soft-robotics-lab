"""
Policy models for Guardian Runtime.
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class PolicyType(Enum):
    """Type of policy rule."""
    ALLOW_LIST = "ALLOW_LIST"
    DENY_LIST = "DENY_LIST"
    REQUIRE_APPROVAL = "REQUIRE_APPROVAL"
    RATE_LIMIT = "RATE_LIMIT"


@dataclass
class PolicyRule:
    """Single rule: action -> decision."""
    action: str
    decision: str  # ALLOW, DENY, REQUIRE_APPROVAL
    condition: Optional[Dict[str, Any]] = None  # Optional filter (e.g. path prefix)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action": self.action,
            "decision": self.decision,
            "condition": self.condition,
        }


@dataclass
class Policy:
    """Named policy with a list of rules."""
    name: str
    rules: List[Dict[str, Any]]  # [{"action": "read_file", "decision": "ALLOW"}, ...]
    policy_type: PolicyType = PolicyType.ALLOW_LIST
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "rules": self.rules,
            "policy_type": self.policy_type.value,
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Policy":
        pt = data.get("policy_type", "ALLOW_LIST")
        if isinstance(pt, str):
            pt = PolicyType(pt) if pt in [e.value for e in PolicyType] else PolicyType.ALLOW_LIST
        return cls(
            name=data["name"],
            rules=data.get("rules", []),
            policy_type=pt,
            metadata=data.get("metadata", {}),
        )
