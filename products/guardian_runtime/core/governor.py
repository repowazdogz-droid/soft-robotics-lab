"""
Governor: policy enforcement engine. Receives action requests, checks policies,
returns ALLOW / DENY / REQUIRE_APPROVAL. All decisions logged to audit bundle.
"""
from typing import Any, Dict, List, Optional

from ..schemas.action import ActionRequest, ActionDecision, Decision
from ..schemas.bundle import AuditBundle
from ..schemas.policy import Policy
from .policies import PolicyEngine
from .audit import record_decision


class Governor:
    """Policy enforcement engine. All decisions logged to the provided audit bundle."""

    def __init__(self, policies: Optional[List[Policy]] = None, audit_bundle: Optional[AuditBundle] = None):
        self.engine = PolicyEngine(policies or [])
        self.audit_bundle = audit_bundle

    def set_audit_bundle(self, bundle: AuditBundle) -> None:
        self.audit_bundle = bundle

    def add_policy(self, policy: Policy) -> None:
        self.engine.add_policy(policy)

    def request(self, action: str, params: Optional[Dict[str, Any]] = None) -> Decision:
        """Check action against policies; return decision. Log to audit bundle if set."""
        params = params or {}
        decision = self.engine.evaluate(action, params)
        reason = f"Policy rule: {decision.value}"

        ad = ActionDecision(action=action, params=params, decision=decision, reason=reason)
        if self.audit_bundle:
            record_decision(self.audit_bundle, ad)
        return decision

    def check_request(self, request: ActionRequest) -> Decision:
        """Check an ActionRequest; log and return decision."""
        return self.request(request.action, request.params)
