"""
Base wrapper: any agent inherits from this. Intercepts tool/action calls,
routes through Governor before execution, logs outcome after.
"""
from typing import Any, Callable, Dict, List, Optional

from ..schemas.policy import Policy
from ..schemas.action import ActionRequest, ActionOutcome, Decision
from ..schemas.bundle import AuditBundle
from ..core.governor import Governor
from ..core.audit import create_bundle, record_action, record_outcome, close_bundle


class GuardianWrapper:
    """Base class for agents that run under Guardian. All actions go through Governor."""

    def __init__(
        self,
        agent_id: str,
        session_id: Optional[str] = None,
        policies: Optional[List[Policy]] = None,
        audit_bundle: Optional[AuditBundle] = None,
    ):
        import uuid
        self.agent_id = agent_id
        self.session_id = session_id or str(uuid.uuid4())
        self._policies = list(policies or [])
        self._bundle = audit_bundle or create_bundle(agent_id, self.session_id)
        self._governor = Governor(policies=self._policies, audit_bundle=self._bundle)

    def request_action(self, action: str, params: Optional[Dict[str, Any]] = None) -> Any:
        """
        Request an action. Governor checks policy; if ALLOW, execute and return result;
        if DENY, raise; if REQUIRE_APPROVAL, raise (MVP: no approval flow, treat as deny for auto).
        """
        params = params or {}
        req = ActionRequest(action=action, params=params, agent_id=self.agent_id, session_id=self.session_id)
        record_action(self._bundle, req)

        decision = self._governor.check_request(req)
        if decision == Decision.DENY:
            record_outcome(
                self._bundle,
                ActionOutcome(action=action, params=params, success=False, error="DENIED by policy"),
            )
            raise PermissionError(f"Action {action} denied by policy")
        if decision == Decision.REQUIRE_APPROVAL:
            record_outcome(
                self._bundle,
                ActionOutcome(action=action, params=params, success=False, error="REQUIRE_APPROVAL (not granted in MVP)"),
            )
            raise PermissionError(f"Action {action} requires approval (not implemented in MVP)")

        # ALLOW: execute via subclass hook
        try:
            result = self._execute_action(action, params)
            record_outcome(self._bundle, ActionOutcome(action=action, params=params, success=True, result=result))
            return result
        except Exception as e:
            record_outcome(
                self._bundle,
                ActionOutcome(action=action, params=params, success=False, error=str(e)),
            )
            raise

    def _execute_action(self, action: str, params: Dict[str, Any]) -> Any:
        """Override in subclass to perform the actual action. Default: no-op, return None."""
        return None

    def export_audit_bundle(self, close: bool = True) -> AuditBundle:
        """Export the audit bundle (optionally close it first)."""
        if close and not self._bundle.is_closed:
            close_bundle(self._bundle)
        return self._bundle
