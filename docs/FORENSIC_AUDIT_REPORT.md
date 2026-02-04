# OMEGA Stack — Read-Only Forensic Audit Report

**Mode:** Audit only. No code changes. No branches, commits, or patches.  
**Scope:** Determinism, audit integrity, silent failure, boundary enforcement, cross-product brittleness.  
**Evidence:** File paths and line numbers cited. Uncertainties stated where applicable.

---

## 1. SYSTEM SNAPSHOT

### Repo structure summary

- **Root:** `OmegaStack/` — README, Makefile, `requirements.txt`, `scripts/`, `docs/`, `products/`.
- **Products:** Multiple Python products under `products/` (breakthrough_engine, enterprise/, frontline, guardian_runtime, omega_console, omega_foundry, omega_scientist, omega_tutor, reality_bridge, shared/, soft_robotics_lab, templates/, world_model_studio). Frontline includes a Vite/React/TS UI under `products/frontline/ui/`.
- **Shared:** `products/shared/` — substrate (temporal, cross_connector, vector_store, knowledge_graph, lineage), id_generator, audit (bundle), contracts, trust (score), tutor_links.
- **Config / tests:** `pytest.ini` at root (referenced in README); product-level `requirements.txt` and tests under `products/*/tests/` or `*_test*.py`.

### Core products and responsibilities

| Product | Responsibility |
|--------|-----------------|
| **Guardian Runtime** | Policy enforcement (Governor), audit bundles (in-memory + JSON save/load), wrappers for agents. |
| **Breakthrough Engine** | Hypothesis ledger (SQLite), evidence/history, decay, substrate integration (optional). |
| **Reality Bridge** | MJCF validation, webhooks, database (validation results), prescriptive fixes. |
| **OMEGA Foundry** | Design from intent, templates, constraint solver, design evolver, history. |
| **World Model Studio** | Simulation, grasp, reality-gap tracking, batch jobs, scene composition. |
| **OMEGA Scientist** | Paper parsing, cross-domain, contradiction miner, hypothesis ranker, ledger integration. |
| **OMEGA Tutor** | Curriculum, sessions, quizzes, knowledge base, spaced repetition. |
| **Decision Brief** | Enterprise decision briefs, domains, substrate integration. |
| **Frontline** | Therapy reflection app (UI: Ollama, crisis detection, localStorage); Python schemas/integration (guardian, substrate). |
| **Soft Robotics Lab** | Gripper zoo, calibration/experiments DB, research memory, workbench (MuJoCo validator, MJCF generator). |

### Shared infrastructure overview

- **ID generation:** `products/shared/id_generator.py` — `hypothesis_id()` (uuid4-based), `generate_id(prefix, suffix)` (date + uuid4 or suffix).
- **Audit:** Two notions: (1) Guardian’s in-memory `AuditBundle` (schemas/bundle.py, core/audit.py) — actions/decisions/outcomes, close makes add_* raise; (2) shared zip/artifact bundle in `products/shared/audit/bundle.py` — metadata, artifact, contract, validation, logs, lineage; save/load as zip, no integrity verification on load.
- **Substrate:** Optional vector store, knowledge graph, lineage, cross_connector, temporal query engine; components built with try/except and silent fallback (e.g. `products/shared/substrate/__init__.py`).
- **Cross-product imports:** Frontline and others use `from products.guardian_runtime ...` or `from products.shared ...`; breakthrough_engine uses `from shared.id_generator import hypothesis_id` (path hack). Weekly review imports `from products.breakthrough_engine.hypothesis_ledger import ...`. Tests use `from products.frontline...` etc. Running from different CWDs can break these unless PYTHONPATH or path inserts are set.

---

## 2. FINDINGS TABLE

| ID | Severity | Description | Evidence (file:line) | Failure mode | Why this matters |
|----|----------|-------------|------------------------|--------------|------------------|
| **D1** | High | **Time-based evidence/history IDs** — Evidence IDs `E-{YYYYMMDDHHMMSS}` and history IDs `LOG-{YYYYMMDDHHMMSSffffff}`. Two evidence or two history rows in the same second (or same microsecond for history) can collide. | `products/breakthrough_engine/hypothesis_ledger.py:611` (Evidence), `:931` (history) | PK collision on INSERT; duplicate key or overwrite depending on DB. | Determinism/replay and uniqueness; audit ordering by ID breaks. |
| **D2** | Medium | **ID generator mixes date + unseeded randomness** — `generate_id()` uses `datetime.now().strftime("%Y%m%d")` + `uuid.uuid4().hex[:5]`. `hypothesis_id()` is uuid4 only (no time). Unseeded randomness is intentional for uniqueness but makes runs non-reproducible. | `products/shared/id_generator.py:24-29`, `:45-46` | Same logical event in two runs gets different IDs. | Reproducibility of traces and tests; ordering by ID is not stable. |
| **D3** | Low | **Reality-gap record ID** — `GAP-{MD5(gripper_id+task_id+policy_id+datetime.now().isoformat())[:12]}`. Time is part of input; same (gripper, task, policy) in same moment could theoretically collide. | `products/world_model_studio/core/reality_gap.py:339-341` | Collision possible in same microsecond. | Lower likelihood than D1; still time-dependent IDs. |
| **A1** | Critical | **Audit bundle not tamper-evident** — Guardian `AuditBundle` is `frozen=False`; `actions`/`decisions`/`outcomes` are mutable lists. After `close()`, `add_*` raise but existing list contents can be mutated in place by anyone holding a reference. No hash/signature. | `products/guardian_runtime/schemas/bundle.py:10-26` | In-memory tampering; no integrity check on serialization. | Claim of “portable trust artifact” and “immutable once closed” is only append-restricted, not content-integrity. |
| **A2** | High | **Load paths trust input** — Guardian `load_bundle(path)` and shared audit `Bundle.load(zip_path)` parse JSON/zip with no signature, checksum, or schema-strict validation. Corrupt or modified file is accepted. | `products/guardian_runtime/utils/storage.py:23-26`; `products/shared/audit/bundle.py:100-134` | Tampered or corrupted bundle loads as valid. | Audit trail integrity cannot be verified on load. |
| **A3** | Medium | **Shared audit bundle load** — Same as A2 for zip loader; no verification that metadata/artifact/contract haven’t been altered. | `products/shared/audit/bundle.py:100-134` | Same as A2 for zip bundles. | Same as A2. |
| **S1** | High | **Ollama failure returns template fallback** — On any error (network, parse, non-2xx), `generateReflection()` returns `TEMPLATE_FALLBACK`; `generateSummary()` returns a fixed string. Caller cannot distinguish “real AI output” from “failure fallback”. | `products/frontline/ui/src/services/ollama.ts:93-95`, `:117-119` | User sees plausible-looking reflection when Ollama failed or returned invalid JSON. | Silent correctness failure; looks like success. |
| **S2** | High | **Webhook config load** — `_load_webhooks()` on any exception returns `{"webhooks": []}`. Corrupt or invalid `webhooks.json` yields empty list with no error. | `products/reality_bridge/core/webhooks.py:24-26` | All webhooks disappear from perspective of caller. | Silent failure; notifications silently disabled. |
| **S3** | High | **Substrate component init** — Multiple `except Exception: pass` when building VectorStore, KnowledgeGraph, LineageGraph, CrossProductConnector, TemporalQueryEngine. Failures leave missing components with no error to caller. | `products/shared/substrate/__init__.py:64-99` | Substrate appears to work but parts are missing. | Silent degradation; cross-product calls may get None or wrong behavior. |
| **S4** | Medium | **Ledger substrate write** — After saving hypothesis, `record_to_substrate(h)` is in try/except with `pass`. Substrate write failure is invisible. | `products/breakthrough_engine/hypothesis_ledger.py:910-913` | Ledger and substrate can diverge with no signal. | Audit/provenance gap. |
| **S5** | Medium | **Frontline Guardian log** — `log_safeguarding_escalation()` catches all exceptions and passes. If Guardian import or `record_action`/`record_outcome` fails, escalation is not logged and caller is not informed. | `products/frontline/integration/guardian.py:79-80` | Safeguarding escalations can be missing from audit. | Safety-critical audit gap. |
| **S6** | Medium | **Temporal belief load** — On any exception during JSON load of beliefs file, `except Exception: pass`; belief store can remain empty or partially loaded with no error. | `products/shared/substrate/temporal.py:107-108` | Belief history lost or inconsistent. | Silent data loss or inconsistency. |
| **S7** | Medium | **Template loader** — Multiple `except Exception: continue` or `return None` when loading template JSON; bad or malformed template is skipped with no log or raise. | `products/omega_foundry/core/template_loader.py:35-36`, `:56`, `:98-99` | Invalid templates silently ignored. | Misconfiguration or corruption hidden. |
| **S8** | Low | **Reality Bridge validator** — Loader init uses `except Exception: pass`; if loader is missing, `_loader` is None and validation continues with fallback path. Some other `except Exception` branches return structured ValidationResult with error message (not silent). | `products/reality_bridge/core/validator.py:60-61` | Loader failure is silent; validation errors are surfaced. | Partial silent failure at init. |
| **B1** | Medium | **Policy evaluation order** — Docstring says “First matching rule wins” and “Order: DENY > REQUIRE_APPROVAL > ALLOW”. Implementation: DENY returns immediately; REQUIRE_APPROVAL and ALLOW overwrite `decision` and loop continues. So *last* matching rule (for ALLOW/REQUIRE_APPROVAL) wins. | `products/guardian_runtime/core/policies.py:37-59` | Policy order and rule order change outcome; doc and code disagree. | Boundary enforcement and audit interpretation wrong if readers trust “first match”. |
| **B2** | Low | **Guardian bundle “immutable once closed”** — Only appends are blocked; list/dict contents are still mutable (see A1). Docstring overstates immutability. | `products/guardian_runtime/schemas/bundle.py:23-26`, docstring “Immutable once closed” | Same as A1. | Claim vs reality. |
| **E1** | High | **Lambda fallbacks on ImportError** — Breakthrough engine substitutes `record_to_substrate`, `find_similar_hypotheses`, etc. with lambdas that return None or [] on ImportError. Ledger behaves as if substrate is empty rather than failing. | `products/breakthrough_engine/hypothesis_ledger.py:60-65` | Substrate unavailability is invisible; lineage and similarity are no-ops. | Cross-product brittleness; “optional” dependency hides total failure of integration. |
| **E2** | Medium | **Cross-product import paths** — Frontline, weekly_review, tests use `from products.*` or `from products.breakthrough_engine.*`. Depends on repo root (or correct path insert) being on PYTHONPATH. Running a product from its own dir without PYTHONPATH can break imports. | `products/frontline/integration/guardian.py:20,73-74`; `products/breakthrough_engine/weekly_review.py:38`; `products/frontline/tests/*.py` | ImportError at runtime when CWD/path differs. | Brittleness in deployment and local runs. |
| **E3** | Medium | **Shared audit bundle** — Uses `sys.path.insert` and `from shared.id_generator`; `from shared.substrate` with `except Exception`. Different from Guardian’s bundle; two “audit bundle” concepts. | `products/shared/audit/bundle.py:14-20` | Confusion and duplicate code; load path assumes path hacks. | Consistency and maintainability. |

---

## 3. CLAIMS VS REALITY

| Claim / Doc | Location | Reality |
|-------------|----------|---------|
| “Deterministic Core” / “Safety-critical logic is rule-based, inspectable, testable” | docs/OMEGA_MANIFEST.md (P1) | IDs use unseeded uuid4 and/or wall-clock time; policy evaluation is order-dependent; many code paths swallow exceptions. |
| “Deep Traceability” / “Outputs traceable to rules, assumptions, sources” | docs/OMEGA_MANIFEST.md (P6) | Audit bundles are not tamper-evident; load does not verify integrity; substrate write failures are silent (S4). |
| “Immutable once closed” (audit bundle) | guardian_runtime/schemas/bundle.py | Only appends are blocked; in-memory list/dict contents can still be mutated (A1, B2). |
| “Portable trust artifact” | guardian_runtime/schemas/bundle.py | No signature or checksum; load trusts file content (A2). |
| “First matching rule wins” / “Order: DENY > REQUIRE_APPROVAL > ALLOW” | guardian_runtime/core/policies.py | Last matching ALLOW/REQUIRE_APPROVAL wins; DENY is first-match (B1). |
| Graceful degradation (e.g. substrate optional) | breakthrough_engine, shared/substrate | Implemented via broad except and lambdas that return empty/None; failures are silent (E1, S3). |

---

## 4. RISK REGISTER

Even if “everything seems to work” in a single run:

1. **Replay / reproducibility** — Same inputs in another run produce different IDs and possibly different ordering; time-based IDs can collide under load or in tests.
2. **Audit compromise** — Bundles can be modified in memory before serialization; saved bundles can be edited and reloaded with no detection.
3. **Silent divergence** — Substrate, webhooks, or Guardian logging can fail while the rest of the app succeeds; ledger/substrate/audit state can diverge without any alert.
4. **Policy surprises** — Adding or reordering policies can change outcomes (last ALLOW/REQUIRE_APPROVAL wins); operators may not expect order-sensitivity.
5. **Frontline UX** — User may believe reflection/summary came from Ollama when it was the fallback; crisis or clinical use could rely on “real” output.
6. **Deployment** — Different CWD or missing PYTHONPATH can cause ImportErrors for `products.*` or `shared.*`, especially in containers or scripts.
7. **Corrupt or malicious files** — Corrupt webhooks.json, belief store, or audit zip is accepted; no integrity check on load.

---

## 5. RECOMMENDED REMEDIATION (NO CODE)

- **Safety-critical (high impact on trust / safety)**  
  1. **A1/A2:** Define “tamper-evident” for audit artifacts: e.g. hash or sign bundle content on close/save; on load, verify before use.  
  2. **S1:** In Frontline Ollama service, do not return template fallback as success; return a distinct error type or UI state so caller can show “generation failed” instead of fake content.  
  3. **S5:** In Frontline Guardian integration, do not swallow exceptions in `log_safeguarding_escalation`; at minimum log failure and/or re-raise or return a boolean so caller can handle.  
  4. **S2:** When webhook config load fails, log error and/or surface to operator; avoid returning empty list as if “no webhooks” without distinguishing “failed to load”.

- **Non-breaking (hygiene, no API/behavior change)**  
  5. **D1:** Use composite or UUID-based IDs for evidence and history (e.g. hypothesis_id + uuid4 or monotonic suffix) so same-second inserts cannot collide.  
  6. **S3, S6, S7:** Replace broad `except Exception: pass` with specific exceptions and either log + re-raise or return a clear “degraded” result and log.  
  7. **S4:** Log substrate write failures in the ledger (and optionally retry or queue) instead of silent pass.  
  8. **B1:** Align policy engine with doc: either implement true “first matching rule wins” or update doc to “last matching rule wins for ALLOW/REQUIRE_APPROVAL”.  
  9. **A1/B2:** Document that “immutable once closed” means “no new entries” only, and that full tamper-evidence requires hashing/signing (then implement per item 1).

- **Breaking or structural**  
  10. **A2/A3:** Add optional integrity check on bundle load (e.g. expected hash/signature); make it required for “trusted” mode.  
  11. **E1:** Consider making substrate integration explicit: if configured, require it to be available and fail fast on write/read failure instead of lambda no-ops.  
  12. **E2/E3:** Standardize how products resolve `products.*` and `shared.*` (e.g. install as package, or single entrypoint that sets path); document and test from repo root and from product dirs.

Order: 1–4 are safety-critical; 5–9 are non-breaking hygiene; 10–12 are breaking or structural and need design/rollout decisions.
