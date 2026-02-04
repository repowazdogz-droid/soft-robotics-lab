# AGENTS.md — OMEGA Agent Template

Copy this file into your agent or product repo and fill in the sections. Use one AGENTS.md per logical agent (or one per product if it has a single agent).

---

## Purpose

**Agent name:**  
**Product:**  
**One-line description:**  
What this agent does and what problem it solves. Example: "Orchestrates design generation from natural language intent and exports MJCF/URDF; all file writes go through Guardian."

---

## Guardian Integration

- **Wrapper:** Does this agent use `GuardianWrapper` (or equivalent)? Yes / No.
- **Governor:** How are policies loaded? (e.g. from `products/guardian_runtime`, from product-local YAML, from env.)
- **Audit bundle:** When is a bundle created and closed? (e.g. one per user session, one per design run.) Where are bundles stored? (e.g. `data/audit/`, exported to Substrate.)

If the agent does not yet use Guardian, state why and what actions would need to be governed (e.g. file write, API call, model export).

---

## Policies

List the policies that apply to this agent. For each:

- **Policy name:**  
- **Rules:** (e.g. `read_file` → ALLOW, `write_file` → REQUIRE_APPROVAL, `delete_file` → DENY.)  
- **Source:** (e.g. `guardian_runtime` default, product `policies/file_access.yaml`.)

Example:

| Action        | Decision           |
|---------------|--------------------|
| read_file     | ALLOW              |
| write_file    | REQUIRE_APPROVAL   |
| delete_file   | DENY               |

---

## Audit Events

What gets recorded in the audit bundle?

- **Actions:** (e.g. every `request_action` call: action name + params.)
- **Decisions:** (e.g. every Governor decision: ALLOW/DENY/REQUIRE_APPROVAL.)
- **Outcomes:** (e.g. success/failure and result or error after execution.)

Any PII or secrets must be redacted before persistence or export.

---

## Dependencies

- **Python:** (e.g. 3.11+.)
- **OMEGA:** Guardian Runtime, Substrate (if used), other products (e.g. Reality Bridge for validation).
- **External:** (e.g. OpenAI, MuJoCo, Streamlit.) Prefer listing in `requirements.txt` with minimum versions.

---

*Template: [products/templates/AGENTS.md](AGENTS.md)*
