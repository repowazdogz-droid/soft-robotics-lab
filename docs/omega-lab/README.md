# omega-lab

R&D and testing sibling of OMEGA Stack. Pipelines, agents, world models, memory, run tracking, and experiments — code at **repo root** (not under products/).

---

## Quick start: run a simulation / pipeline

From OMEGA Stack **repo root**:

```bash
# Run the full pipeline chain (research → learning → synthesis → coalitions)
python run_chain.py --topic soft_robotics

# Interactive shell with LLM (pipelines + context)
python omega_shell.py --topic synthetic_biology

# Single shot: pipelines + LLM reply
python omega_main.py "What are the key bottlenecks in soft robotics?" --topic soft_robotics

# Generate world model snapshot
python agents/omega_worldgen.py synthetic_biology

# Full agent loop (worldgen → delta → project)
python agents/omega_loop.py "build a soft gripper prototype" --topic soft_robotics
```

Run records appear in `runs/` (e.g. `runs/research_*/`, `runs/learning_*/`). World snapshots in `sim/worlds/`. Memory traces in `memory/traces.jsonl`.

---

## Dependencies

- Python 3.10+ (3.13 supported).
- Root `requirements.txt` and any product deps you use.
- LLM: `llm_client` (local or API) for agents.
- Optional: Chroma for semantic memory (`memory/storage/chroma/`).

Install from repo root:

```bash
pip install -r requirements.txt
```

---

## Experiments

Experiments are defined and tracked via `omega_lab.py`:

```python
from omega_lab import OmegaLab, Experiment

lab = OmegaLab()
exp = lab.create_experiment(
    name="Bimanual contact",
    hypothesis="Contact stiffness affects stability",
    domain="soft_robotics",
    variants=[{"stiffness": 0.1}, {"stiffness": 0.5}],
    success_metric="stability_score",
)
# exp.id, exp.variants, etc.
```

Experiment runners (e.g. `run_contact_physics_experiment.py`, `run_bimanual_contact_experiment.py`) use the same codebase.

---

## Documentation

- **[CODEBASE_MAP.md](../../CODEBASE_MAP.md)** — Full map of repo (lab + products), entrypoints, capabilities, integration points.
- **[PRODUCT.md](PRODUCT.md)** — omega-lab product overview, components, Guardian (future), API, testing, deployment.
- **[omega-lab integration](../../docs/omega-lab-integration.md)** — How omega-lab integrates with OMEGA Stack products.

---

*omega-lab: R&D sibling of OMEGA Stack.*
