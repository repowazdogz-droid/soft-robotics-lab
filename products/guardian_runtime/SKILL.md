# Guardian Runtime — SKILL for Claude/Cursor

Use this when building or modifying OMEGA Guardian Runtime or any agent that must run under governance.

## What Guardian Is

- **Governance layer** for OMEGA agents. Every action goes through a **Governor** that enforces **policies** (ALLOW / DENY / REQUIRE_APPROVAL).
- **Audit bundles** record every session: actions requested, decisions made, outcomes. Bundles are immutable once closed and exportable as JSON (portable trust artifacts).
- **Deterministic core**: policy engine is rule-based; no LLM required for MVP. Policies can be loaded from YAML or Python dicts.

## Key Concepts

1. **Governor** (`core/governor.py`) — Receives action requests, evaluates against policies, returns Decision (ALLOW/DENY/REQUIRE_APPROVAL). Optionally logs every decision to an audit bundle.
2. **Audit bundle** (`core/audit.py`, `schemas/bundle.py`) — One per session. Contains agent_id, session_id, started_at, closed_at, actions[], decisions[], outcomes[]. Immutable after `close_bundle()`.
3. **Policies** (`core/policies.py`, `schemas/policy.py`) — Named list of rules: `{"action": "read_file", "decision": "ALLOW"}`. Policy types: ALLOW_LIST, DENY_LIST, REQUIRE_APPROVAL, RATE_LIMIT. First matching rule wins; default is DENY.
4. **GuardianWrapper** (`wrappers/base.py`) — Base class for agents. Subclass and implement `_execute_action(action, params)`. Call `request_action(action, params)` to route through Governor; DENY/REQUIRE_APPROVAL raise PermissionError.

## File Layout

- `core/governor.py` — Governor class
- `core/audit.py` — create_bundle, record_*, close_bundle
- `core/policies.py` — PolicyEngine, load_policies_from_dict / from_yaml
- `schemas/action.py` — ActionRequest, ActionDecision, ActionOutcome, Decision
- `schemas/policy.py` — Policy, PolicyRule, PolicyType
- `schemas/bundle.py` — AuditBundle (add_action, add_decision, add_outcome, close, json())
- `wrappers/base.py` — GuardianWrapper (request_action, export_audit_bundle)
- `wrappers/example.py` — ExampleAgent (read_file, write_file simulated)
- `utils/logging.py` — structured_log; `utils/storage.py` — save_bundle, load_bundle

## Coding Conventions

- Use dataclasses for schemas (no Pydantic required for MVP).
- Timestamps: `datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")`.
- Default decision when no rule matches: DENY.
- When adding features, keep policy engine free of LLM calls unless explicitly extending for “optional LLM enrichment.”

## Tests

- `tests/test_governor.py` — Policy enforcement (allow, deny, require_approval, default deny, logging to bundle).
- `tests/test_audit.py` — Bundle creation, recording, immutability after close, export JSON.
- `tests/test_wrapper.py` — Wrapped agent: allow read/write, deny raises, require_approval raises.

Run: `PYTHONPATH=.. pytest tests/ -v` from `products/guardian_runtime`.

## Example Prompt for Extensions

“Add a RATE_LIMIT policy type to Guardian: max N actions per session per action name. Implement in PolicyEngine and add a test in test_governor.py.”
