"""Bundles generate, are immutable, export properly."""
import pytest

from guardian_runtime.core.audit import create_bundle, record_action, record_decision, record_outcome, close_bundle
from guardian_runtime.schemas.action import ActionRequest, ActionDecision, ActionOutcome, Decision


def test_bundle_creation():
    bundle = create_bundle("agent1", "session1")
    assert bundle.agent_id == "agent1"
    assert bundle.session_id == "session1"
    assert bundle.started_at
    assert bundle.closed_at is None
    assert bundle.actions == []
    assert bundle.decisions == []
    assert bundle.outcomes == []


def test_bundle_records():
    bundle = create_bundle("agent1", "session1")
    req = ActionRequest(action="read_file", params={"path": "/a.txt"}, agent_id="agent1", session_id="session1")
    record_action(bundle, req)
    record_decision(bundle, ActionDecision(action="read_file", params={"path": "/a.txt"}, decision=Decision.ALLOW))
    record_outcome(bundle, ActionOutcome(action="read_file", params={"path": "/a.txt"}, success=True, result={}))
    assert len(bundle.actions) == 1
    assert len(bundle.decisions) == 1
    assert len(bundle.outcomes) == 1


def test_bundle_immutable_after_close():
    bundle = create_bundle("agent1", "session1")
    close_bundle(bundle)
    assert bundle.is_closed
    assert bundle.closed_at is not None
    with pytest.raises(RuntimeError, match="closed"):
        bundle.add_action({"action": "read_file"})


def test_bundle_export_json():
    bundle = create_bundle("agent1", "session1")
    bundle.add_action({"action": "read_file", "params": {}})
    close_bundle(bundle)
    s = bundle.json()
    assert "agent_id" in s
    assert "session1" in s
    assert "read_file" in s
    d = bundle.to_dict()
    assert d["agent_id"] == "agent1"
    assert len(d["actions"]) == 1
