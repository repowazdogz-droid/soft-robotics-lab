# 🚀 OMEGA Stack

**The Translation Layer Between Research and Reality**

A complete research operating system: discover, validate, track, teach, and build.

---

## 🎯 What is OMEGA?

OMEGA is not a collection of tools. It's a **unified research platform** where:

- **OMEGA Scientist** discovers hypotheses from literature
- **Hypothesis Ledger** tracks them to breakthrough
- **Reality Bridge** validates designs in physics
- **OMEGA Foundry** generates designs from intent
- **World Model Studio** trains manipulation policies
- **OMEGA Tutor** teaches anyone anything
- **Decision Brief** structures strategic decisions
- **Soft Robotics Lab** provides domain-specific tools
- **Substrate** connects and remembers everything

---

## 💡 The Core Insight

Most research tools ask: *"What do the papers say?"*

OMEGA asks: *"What will actually work?"*

This is the **Translation Trinity**:

| Compiler | Question | Result |
|----------|----------|--------|
| **SRFC** | Can it work physically? | GREEN / AMBER / RED |
| **TSRFC** | What workflow does it replace? | GREEN / AMBER / RED |
| **VRFC** | Will it survive reality? | GREEN / AMBER / RED |

Every hypothesis, design, and decision runs through this filter.

---

## 📦 Products

### Discovery & Validation

| Product | Purpose | Port |
|---------|---------|------|
| [OMEGA Scientist](products/omega_scientist/) | Find discoveries in literature | 8506 |
| [Breakthrough Engine](products/breakthrough_engine/) | Track hypotheses to breakthrough | 8502 |
| [Reality Bridge](products/reality_bridge/) | Physics validation for designs | 8000/8501 |

### Design & Training

| Product | Purpose | Port |
|---------|---------|------|
| [OMEGA Foundry](products/omega_foundry/) | Design from intent | 8504 |
| [World Model Studio](products/world_model_studio/) | Train manipulation policies | 8505 |
| [Soft Robotics Lab](products/soft_robotics_lab/) | Domain-specific design tools | 8501 |

### Decision & Learning

| Product | Purpose | Port |
|---------|---------|------|
| [Decision Brief](products/enterprise/decision_brief/) | Strategic assessment | 8507 |
| [OMEGA Tutor](products/omega_tutor/) | Adaptive teaching | 8503 |

### Infrastructure & Governance

| Product | Purpose |
|---------|---------|
| [Guardian Runtime](products/guardian_runtime/) | Policy enforcement & audit bundles for all agents |
| [Substrate](products/shared/substrate/) | Memory & knowledge layer |
| Shared Components | Audit bundles, contracts, trust scores |

### Planned

| Product | Purpose |
|---------|---------|
| Frontline | TBD |
| Audit Bundle (shared) | Portable trust artifacts |
| Additional products | Per roadmap |

### R&D: omega-lab (repo root)

omega-lab is the R&D and testing sibling at **repo root** (not under products/). Pipelines, agents, world models, memory, runs, and experiments.

- **[CODEBASE_MAP.md](CODEBASE_MAP.md)** — Full map of lab + products, entrypoints, capabilities.
- **[docs/omega-lab/README.md](docs/omega-lab/README.md)** — Quick start: run pipeline, shell, experiments.
- **[docs/omega-lab/PRODUCT.md](docs/omega-lab/PRODUCT.md)** — omega-lab product overview (Guardian not required yet).
- **[docs/omega-lab-integration.md](docs/omega-lab-integration.md)** — Integration with OMEGA Stack products (Substrate, Guardian when ready).

---

## 📐 Development Standards

All products build consistently under OMEGA Stack standards:

- **[.cursor/rules](.cursor/)** — Naming, Guardian wrapper, Pydantic, pytest, local-first, human-in-the-loop.
- **[AGENTS.md template](products/templates/AGENTS.md)** — Purpose, Guardian integration, policies, audit events, dependencies.
- **[PRODUCT.md template](products/templates/PRODUCT.md)** — Overview, components, Guardian policies, API, testing, deployment.
- **[pytest.ini](pytest.ini)** — Shared pytest config at repo root.

**Quick start for a new product:**

1. Copy [products/templates/PRODUCT.md](products/templates/PRODUCT.md) into your product folder; fill Overview, Components, Guardian Policies, API, Testing, Deployment.
2. If the product has an agent, copy [products/templates/AGENTS.md](products/templates/AGENTS.md); fill Purpose, Guardian Integration, Policies, Audit Events, Dependencies.
3. Add `core/`, `tests/`, `README.md`, `requirements.txt`. Use [Guardian Runtime](products/guardian_runtime/) for any agent that performs tool/API actions.
4. Run tests: from product root `PYTHONPATH=.. pytest tests/ -v`, or from repo root `pytest` (uses root [pytest.ini](pytest.ini)).

---

## 🔄 The Breakthrough Loop

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│   DISCOVER (Scientist)                                          │
│   Find contradictions, cross-domain connections, failures       │
│                         ↓                                       │
│   HYPOTHESIZE (Ledger)                                          │
│   Auto-create, estimate falsification cost                      │
│                         ↓                                       │
│   VALIDATE (Reality Bridge)                                     │
│   SRFC/TSRFC/VRFC — Can it work? Will it translate?             │
│                         ↓                                       │
│   TRACK (Ledger)                                                │
│   Confidence decay, evidence, health monitoring                 │
│                         ↓                                       │
│   BREAKTHROUGH                                                  │
│   High confidence + GREEN SRFC + GREEN VRFC = actionable        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

```bash
# Clone the repo
git clone https://github.com/yourusername/OmegaStack.git
cd OmegaStack

# Install dependencies
pip install -r requirements.txt

# Start OMEGA Scientist
cd products/omega_scientist
streamlit run app.py --server.port 8506

# In another terminal, start Breakthrough Engine
cd products/breakthrough_engine
streamlit run app.py --server.port 8502

# In another terminal, start Reality Bridge
cd products/reality_bridge
uvicorn app:app --port 8000
streamlit run dashboard.py --server.port 8501
```

---

---

## 🏗️ Architecture

```
OmegaStack/
├── .cursor/                    # Cursor rules (OMEGA standards)
├── products/
│   ├── guardian_runtime/        # Governance layer (Guardian wrapper, audit bundles)
│   ├── omega_scientist/        # Discovery engine
│   ├── breakthrough_engine/    # Hypothesis ledger
│   ├── reality_bridge/         # Physics validation
│   ├── omega_foundry/          # Design from intent
│   ├── world_model_studio/     # Policy training
│   ├── omega_tutor/            # Adaptive teaching
│   ├── soft_robotics_lab/      # Domain tools
│   ├── templates/              # AGENTS.md, PRODUCT.md
│   └── enterprise/
│       └── decision_brief/     # Strategic assessment
├── products/shared/
│   ├── substrate/              # Memory layer
│   ├── audit/                  # Provenance
│   ├── contracts/              # Validation contracts
│   └── docs/
│       └── GLOSSARY.md         # SRFC/TSRFC/VRFC definitions
├── pytest.ini                  # Shared test config
└── requirements.txt
```

---

## 🔌 Integrations

### Scientist → Ledger
Discoveries auto-create hypotheses:
```python
# After running discovery modes in Scientist
from integration.ledger_integration import bulk_create_from_scientist_session
results = bulk_create_from_scientist_session(st.session_state, auto_add=True)
```

### Foundry → Reality Bridge
Designs auto-validate:
```python
mjcf = foundry.generate(intent)
result = reality_bridge.validate(mjcf)
if not result.passed:
    fixes = reality_bridge.get_fixes(result)
```

### Ledger → Decision Brief
Hypotheses inform decisions:
```python
near = ledger.get_near_breakthroughs()
brief = decision_brief.generate(
    question=f"How to get {near[0].id} to breakthrough?"
)
```

---

## 📊 Comparison to Alternatives

| Capability | Traditional Tools | OMEGA Stack |
|------------|-------------------|-------------|
| Paper analysis | Elicit, Consensus | OMEGA Scientist |
| Hypothesis tracking | Notion, spreadsheets | Hypothesis Ledger |
| Physics validation | Manual | Reality Bridge |
| Design generation | CAD | OMEGA Foundry |
| Translation check | None | SRFC/TSRFC/VRFC |
| Cross-domain discovery | Manual | Automatic |
| Breakthrough detection | Manual | Automatic |

**Key differentiator**: Translation Trinity. Nobody else asks "will it survive reality?" at the discovery stage.

---

## 🎯 Use Cases

1. **Literature review**: Find gaps and contradictions
2. **Grant writing**: Identify high-potential hypotheses
3. **Lab planning**: Prioritize experiments by cost
4. **Design validation**: Physics-check before fabrication
5. **Knowledge management**: Never lose insights
6. **Decision support**: Structured briefs with translation awareness
7. **Teaching**: Personalized learning for lab members

---

## 📋 Requirements

**Core:**
```
streamlit>=1.28.0
fastapi>=0.104.0
mujoco>=3.0.0
numpy>=1.24.0
```

**Optional:**
```
openai>=1.0.0           # Or local LLM
sentence-transformers   # For embeddings
stable-baselines3       # For policy training
```

---

## 🤝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

## 📬 Contact

Built by Warren with OMEGA Research Platform.

For questions or collaboration inquiries, open an issue or contact the development team.

---

*"The translation layer between research and reality."*
