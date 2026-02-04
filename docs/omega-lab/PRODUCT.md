# PRODUCT.md — omega-lab

omega-lab is the R&D and testing sibling of OMEGA Stack. Its code lives at **repo root** (agents/, sim/, memory/, pipelines/, runs/, omega_main.py, omega_lab.py, etc.), not under products/.

---

## Overview

**Product name:** omega-lab  
**Purpose:** R&D and experiment layer for OMEGA Stack — pipelines, agents, world models, memory, run tracking, and experiment definitions. Used for simulations, topic-based runs (soft_robotics, synthetic_biology), and experiment variants (e.g. bimanual contact physics).  
**Port(s):** None (CLI and Python entrypoints only).

omega-lab provides deterministic pipelines (research, learning, synthesis, coalitions), agents that use LLM + pipeline context (worldgen, task design, loop), world model snapshots in sim/worlds/, memory (traces, snapshots, Chroma), run records in runs/, and experiment tracking via omega_lab.py. It integrates with OMEGA Stack products as a sibling: when ready, Substrate for vectors; Guardian when agents perform tool/API actions.

---

## Components

| Component   | Path / module              | Responsibility                                      |
|------------|----------------------------|-----------------------------------------------------|
| Agents     | `agents/`                  | omega_loop, omega_worldgen, omega_world_agent, omega_task_agent, omega_delta, executor, tool_registry |
| Sim        | `sim/`                     | sim_core, world_model_builder, twin_builder, control_loop; worlds/*.json |
| Memory     | `memory/`                  | traces.jsonl, snapshots, recall, semantic_memory, Chroma storage |
| Pipelines  | `pipelines/`               | research, learning, synthesis, coalitions          |
| Runs       | `runs/`                    | Pipeline run records (input/output per run)         |
| Experiments| `omega_lab.py` (root)      | Experiment, OmegaLab.create_experiment — variants, success_metric |
| Entrypoints| root                       | omega_main.py, omega_shell.py, run_chain.py, loop.py, planner.py |

---

## Guardian Policies

- **Does this product use Guardian Runtime?** No (not yet).
- **Reason:** Lab agents are currently deterministic (pipelines) or LLM-only (no tool/API actions that write files or call external APIs). When lab agents perform tool/API actions, they should be wrapped with Guardian Runtime and policies applied.
- **Planned:** If agents gain file or API access, add Guardian wrapper and audit bundles; document policies in this PRODUCT.md.

---

## API (if any)

- **Type:** CLI and Python. No HTTP API.
- **Entrypoints:** See [CODEBASE_MAP.md](../CODEBASE_MAP.md) — `run_chain.py`, `omega_main.py`, `omega_shell.py`, `agents/omega_loop.py`, etc.
- **Auth:** N/A.

---

## Testing

- **How to run tests:** From repo root, run product tests (e.g. `pytest products/guardian_runtime/tests/`). Lab-specific tests: `tests/test_*.py` at root (e.g. `test_experiment.py`, `test_memory.py`) and `agents/test_agents.py`. Use root `pytest.ini` when running from root.
- **Required env:** LLM client (e.g. local or API key) for agents; optional Chroma for memory.
- **Coverage:** Core pipeline and run recording are exercised by run_chain; experiment creation by omega_lab.py usage.

---

## Deployment

- **Local:** Run entrypoints from repo root (e.g. `python run_chain.py --topic soft_robotics`, `python omega_shell.py --topic synthetic_biology`).
- **Env vars:** Any required by `llm_client` or memory backends (e.g. API keys).
- **Data directories:** `runs/`, `outputs/`, `sim/worlds/`, `memory/snapshots/`, `memory/storage/` — ensure writable; optionally back up runs/ and outputs/.

---

*Template: [products/templates/PRODUCT.md](../../products/templates/PRODUCT.md)*
