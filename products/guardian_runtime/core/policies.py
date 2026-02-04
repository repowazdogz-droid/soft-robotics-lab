"""
Policy definitions and rule engine (no LLM for MVP).
Policy types: ALLOW_LIST, DENY_LIST, REQUIRE_APPROVAL, RATE_LIMIT.
Load from YAML or Python dict.
"""
from typing import Any, Dict, List, Optional

from ..schemas.policy import Policy, PolicyType
from ..schemas.action import Decision


def load_policies_from_dict(data: Dict[str, Any]) -> List[Policy]:
    """Load policies from a Python dict (e.g. from YAML)."""
    policies = []
    for item in data.get("policies", [data] if "name" in data else []):
        policies.append(Policy.from_dict(item))
    return policies


def load_policies_from_yaml(path: str) -> List[Policy]:
    """Load policies from a YAML file."""
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return load_policies_from_dict(data or {})


class PolicyEngine:
    """Simple rule engine: match action against policy rules, return decision."""

    def __init__(self, policies: Optional[List[Policy]] = None):
        self.policies = list(policies or [])

    def add_policy(self, policy: Policy) -> None:
        self.policies.append(policy)

    def evaluate(self, action: str, params: Optional[Dict[str, Any]] = None) -> Decision:
        """
        Evaluate action against all policies. First matching rule wins.
        Order: DENY > REQUIRE_APPROVAL > ALLOW. Default if no match: DENY (safe default).
        """
        params = params or {}
        decision = Decision.DENY  # default: deny if no rule matches

        for policy in self.policies:
            for rule in policy.rules:
                rule_action = rule.get("action")
                if rule_action != action:
                    continue
                rule_decision = rule.get("decision", "DENY").upper()
                condition = rule.get("condition")
                if condition and not _match_condition(params, condition):
                    continue
                if rule_decision == "DENY":
                    return Decision.DENY
                if rule_decision == "REQUIRE_APPROVAL":
                    decision = Decision.REQUIRE_APPROVAL
                if rule_decision == "ALLOW":
                    decision = Decision.ALLOW
        return decision


def _match_condition(params: Dict[str, Any], condition: Dict[str, Any]) -> bool:
    """Match params against condition (e.g. path startswith)."""
    for key, value in condition.items():
        if key not in params:
            return False
        pv = params[key]
        if isinstance(value, str) and isinstance(pv, str):
            if key == "path_prefix" and not pv.startswith(value):
                return False
            if key == "path" and pv != value:
                return False
        elif pv != value:
            return False
    return True
