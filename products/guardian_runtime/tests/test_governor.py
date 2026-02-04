"""Policy enforcement works correctly."""
import pytest

from guardian_runtime.core.governor import Governor
from guardian_runtime.schemas.policy import Policy
from guardian_runtime.schemas.action import Decision
from guardian_runtime.core.audit import create_bundle


def test_governor_allow():
    policy = Policy(
        name="file_access",
        rules=[
            {"action": "read_file", "decision": "ALLOW"},
            {"action": "write_file", "decision": "REQUIRE_APPROVAL"},
            {"action": "delete_file", "decision": "DENY"},
        ],
    )
    gov = Governor(policies=[policy])
    assert gov.request("read_file", {"path": "/a.txt"}) == Decision.ALLOW


def test_governor_deny():
    policy = Policy(
        name="file_access",
        rules=[
            {"action": "read_file", "decision": "ALLOW"},
            {"action": "delete_file", "decision": "DENY"},
        ],
    )
    gov = Governor(policies=[policy])
    assert gov.request("delete_file", {"path": "/a.txt"}) == Decision.DENY


def test_governor_require_approval():
    policy = Policy(
        name="file_access",
        rules=[{"action": "write_file", "decision": "REQUIRE_APPROVAL"}],
    )
    gov = Governor(policies=[policy])
    assert gov.request("write_file", {"path": "/out.txt"}) == Decision.REQUIRE_APPROVAL


def test_governor_default_deny():
    policy = Policy(name="file_access", rules=[{"action": "read_file", "decision": "ALLOW"}])
    gov = Governor(policies=[policy])
    assert gov.request("unknown_action", {}) == Decision.DENY


def test_governor_logs_to_audit_bundle():
    bundle = create_bundle("agent1", "session1")
    policy = Policy(name="file_access", rules=[{"action": "read_file", "decision": "ALLOW"}])
    gov = Governor(policies=[policy], audit_bundle=bundle)
    gov.request("read_file", {"path": "/a.txt"})
    assert len(bundle.decisions) == 1
    assert bundle.decisions[0]["action"] == "read_file"
    assert bundle.decisions[0]["decision"] == "ALLOW"
