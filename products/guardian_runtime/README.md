# OMEGA Guardian Runtime

Lightweight runtime that wraps any OMEGA agent, enforcing policies and emitting audit bundles. Every tool call, decision, and output is governed and logged.

## Principles

- **Deterministic core** — policy engine is rule-based (no LLM for MVP)
- **Human-in-the-loop** — zero autonomous action without policy check
- **Local-first** — audit bundles stored locally, exportable as JSON
- **Audit bundles** — portable trust artifacts (agent_id, session_id, actions, decisions, outcomes)

## Install

```bash
pip install -r requirements.txt
```

From `products/guardian_runtime` (so `guardian_runtime` is importable from `products/`):

```bash
cd products/guardian_runtime
PYTHONPATH=.. pytest tests/ -v
```

## Usage

### Define a policy

```python
from guardian_runtime import Policy

policy = Policy(
    name="file_access",
    rules=[
        {"action": "read_file", "decision": "ALLOW"},
        {"action": "write_file", "decision": "REQUIRE_APPROVAL"},
        {"action": "delete_file", "decision": "DENY"},
    ],
)
```

### Wrap an agent

```python
from guardian_runtime import GuardianWrapper, Policy

class MyAgent(GuardianWrapper):
    def _execute_action(self, action: str, params: dict):
        if action == "read_file":
            path = params.get("path", "")
            return {"content": open(path).read(), "path": path}
        if action == "write_file":
            path = params.get("path", "")
            content = params.get("content", "")
            open(path, "w").write(content)
            return {"path": path, "bytes": len(content)}
        return None

    def perform_task(self):
        self.request_action("read_file", {"path": "/data/input.txt"})
        self.request_action("write_file", {"path": "/data/output.txt", "content": "output"})
```

### Run with governance

```python
agent = MyAgent(policies=[policy])
agent.perform_task()

# Export audit bundle
bundle = agent.export_audit_bundle()
print(bundle.json())
```

### Governor only (no wrapper)

```python
from guardian_runtime.core import Governor, create_bundle
from guardian_runtime.schemas import Policy

policy = Policy(name="file_access", rules=[{"action": "read_file", "decision": "ALLOW"}])
bundle = create_bundle("agent1", "session1")
gov = Governor(policies=[policy], audit_bundle=bundle)

decision = gov.request("read_file", {"path": "/a.txt"})  # ALLOW / DENY / REQUIRE_APPROVAL
```

## Structure

- **core/governor.py** — policy enforcement (ALLOW / DENY / REQUIRE_APPROVAL)
- **core/audit.py** — audit bundle creation and recording
- **core/policies.py** — rule engine; load from YAML or dict
- **schemas/** — action, policy, bundle models
- **wrappers/base.py** — base class for wrapped agents
- **utils/** — logging, local storage

## Tests

```bash
# From products/guardian_runtime with PYTHONPATH to products
PYTHONPATH=.. pytest tests/ -v
```

All tests should pass: governor, audit bundle generation/immutability/export, and wrapped agent routing through Governor.
