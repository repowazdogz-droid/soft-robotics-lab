"""Wrapped agent actions route through Governor."""
import pytest

from guardian_runtime.wrappers.example import ExampleAgent
from guardian_runtime.schemas.policy import Policy


def test_wrapper_allow_read_write():
    policy = Policy(
        name="file_access",
        rules=[
            {"action": "read_file", "decision": "ALLOW"},
            {"action": "write_file", "decision": "ALLOW"},
        ],
    )
    agent = ExampleAgent(policies=[policy])
    agent.request_action("read_file", {"path": "/data/input.txt"})
    result = agent.request_action("write_file", {"path": "/data/output.txt", "content": "hello"})
    assert result is not None
    assert result.get("path") == "/data/output.txt"
    bundle = agent.export_audit_bundle()
    assert len(bundle.actions) == 2
    assert len(bundle.decisions) == 2
    assert len(bundle.outcomes) == 2


def test_wrapper_deny_raises():
    policy = Policy(
        name="file_access",
        rules=[
            {"action": "read_file", "decision": "ALLOW"},
            {"action": "delete_file", "decision": "DENY"},
        ],
    )
    agent = ExampleAgent(policies=[policy])
    agent.request_action("read_file", {"path": "/a.txt"})
    with pytest.raises(PermissionError, match="denied"):
        agent.request_action("delete_file", {"path": "/a.txt"})
    bundle = agent.export_audit_bundle()
    assert len(bundle.outcomes) == 2
    assert bundle.outcomes[1].get("success") is False
    assert "DENIED" in str(bundle.outcomes[1].get("error", ""))


def test_wrapper_require_approval_raises():
    policy = Policy(name="file_access", rules=[{"action": "write_file", "decision": "REQUIRE_APPROVAL"}])
    agent = ExampleAgent(policies=[policy])
    with pytest.raises(PermissionError, match="approval"):
        agent.request_action("write_file", {"path": "/out.txt", "content": "x"})
