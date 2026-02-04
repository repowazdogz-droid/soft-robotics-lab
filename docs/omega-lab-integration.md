# omega-lab integration with OMEGA Stack

omega-lab is the R&D and testing code at **repo root** (agents/, sim/, memory/, pipelines/, runs/, omega_main.py, omega_lab.py, etc.). It runs alongside **products/** (Guardian Runtime, Substrate, etc.) as a sibling. Lab code can **import from products**; shared infrastructure is wired where built.

---

## What’s connected

| Area | Status | Details |
|------|--------|---------|
| **Lab → products imports** | ✅ | Repo root is added to `sys.path` in lab entrypoints so `from products.<name> import ...` works. |
| **Guardian Runtime** | ✅ | Lab can import and use `GuardianWrapper`, `Policy`, `Governor`, `create_bundle`. Agents are not wrapped yet (deterministic/LLM-only). |
| **Substrate** | ✅ | Lab can import `products.shared.substrate` (VectorStore, get_substrate, etc.). |
| **Memory → Substrate** | ✅ | Optional: `memory.substrate_backend.publish_trace_to_substrate()` and `substrate_available()`; local memory remains primary. |
| **Guardian-ready pattern** | ✅ | `agents/__init__.py` documents how to wrap lab agents with Guardian when they gain tool/API actions. |

---

## Import patterns (lab code)

Run lab entrypoints from **repo root** (e.g. `python run_chain.py`, `python omega_main.py`). Repo root is on `sys.path`, so these imports work:

```python
# Guardian Runtime
from products.guardian_runtime import GuardianWrapper, Policy, Governor, create_bundle

# Substrate (vector store, knowledge graph, lineage, etc.)
from products.shared.substrate import get_substrate, VectorStore

# Optional: publish lab traces to Substrate
from memory.substrate_backend import publish_trace_to_substrate, substrate_available
if substrate_available():
    publish_trace_to_substrate("research", {"topic": "soft_robotics"}, collection="lab_traces")
```

Entrypoints that add repo root to path: `omega_main.py`, `omega_shell.py`, `run_chain.py`, `omega_lab.py`. Any script run from root that does `sys.path.insert(0, str(Path(__file__).resolve().parent))` (or equivalent) can use the same imports.

---

## Path setup (how it works)

- **products/__init__.py** — Makes `products` a package so `from products.guardian_runtime import ...` works.
- **Lab entrypoints** — At top: `_root = Path(__file__).resolve().parent`; `sys.path.insert(0, str(_root))` so `products` is resolvable when run from repo root.
- **tests/test_integration.py** — Same path setup; verifies `from products.guardian_runtime import GuardianWrapper` and `from products.shared.substrate import get_substrate` (and optional memory.substrate_backend).

---

## Pending / optional

| Item | Note |
|------|------|
| **Guardian for LLM agents** | Not required yet (no tool/API actions). When lab agents do file write or API call, wrap with `GuardianWrapper` and policies; see docstring in `agents/__init__.py`. |
| **Substrate as primary memory** | Lab memory stays local (traces.jsonl, snapshots, Chroma under memory/). Substrate is optional secondary via `memory.substrate_backend`. |
| **Cross-product queries** | Products use Substrate; lab can publish to Substrate so run summaries/experiments are searchable from products (e.g. Breakthrough Engine, Omega Scientist) when wired. |

---

## Where things live

- **Lab entrypoints and code:** Repo root — `run_chain.py`, `omega_main.py`, `omega_shell.py`, `agents/`, `sim/`, `memory/`, `pipelines/`, `runs/`, `omega_lab.py`.
- **Product code:** `products/<name>/` — e.g. `products/guardian_runtime/`, `products/shared/substrate/`.
- **Integration test:** `tests/test_integration.py` — imports from products, GuardianWrapper instantiation, Substrate get_substrate, optional memory.substrate_backend.
- **Docs:** [docs/omega-lab/](omega-lab/) (README, PRODUCT.md). This file: integration and import patterns.

---

## System profile note

If you keep a **system_profile.md**, add an “omega-lab integration” subsection that:

- States omega-lab lives at OmegaStack repo root; lab can import from `products`.
- Lists entrypoints and path setup (omega_main, run_chain, omega_shell, omega_lab).
- Points to this doc and [CODEBASE_MAP.md](../CODEBASE_MAP.md).
