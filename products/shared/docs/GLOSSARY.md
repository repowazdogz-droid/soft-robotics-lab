# OMEGA Terminology Glossary

## The Translation Trinity

OMEGA uses three feasibility compilers to assess whether an innovation can survive from concept to clinical reality. Each answers a different kill-zone question.

### SRFC — Soft Robotics Feasibility Compiler

**Core Question:** "Can it work physically?"

Deterministic assessment of physics, clinical, and materials constraints.

| Dimension | What It Checks |
|-----------|----------------|
| Geometry | Lumen clearance, bend radius, slenderness ratio |
| Actuation | Pressure limits, response time, actuation type vs domain |
| Materials | Young's modulus, Shore hardness, biocompatibility, sterilisation |
| Safety | Contact pressure, tip force, dwell time, tissue limits |
| Regulatory | Device class, intended use risk |
| Workflow | Deployment mode, control mode |

**Status Levels:**
- 🟢 GREEN — Feasible, constraints satisfied
- 🟡 AMBER — Borderline, needs revision
- 🔴 RED — Blocked, constraint violated

**Example Issue:**
```
[LUMEN_CLEARANCE_LOW] 
Outer diameter 14mm leaves <2mm clearance in sigmoid colon.
Suggest: ≤12mm OD or segmented distal region.
```

---

### TSRFC — Transition/Surgical Feasibility Compiler

**Core Question:** "What does it replace?"

Models the surgical workflow and unit operations that change.

| Dimension | What It Checks |
|-----------|----------------|
| Procedure Steps | Which operations are added/removed/modified |
| Time Impact | OR time change, setup time, recovery time |
| Training | Learning curve, credentialing requirements |
| Staffing | Personnel changes, skill requirements |
| Equipment | Capital equipment, consumables, integration |

**Example Output:**
```
Replaces: Manual tissue retraction (12 min avg)
Adds: Device setup (3 min), calibration (2 min)
Net OR time: -7 min
Training burden: 15 cases to proficiency
```

---

### VRFC — Validation & Risk Feasibility Compiler

**Core Question:** "Will it survive reality?"

The translation layer that kills most innovations. Assesses evidence, regulation, reimbursement, and adoption.

| Dimension | What It Checks |
|-----------|----------------|
| Evidence | Endpoint validity, comparator strength, RCT burden |
| Regulatory | Predicate availability, trial burden, device class |
| Reimbursement | CPT/DRG fit, cost-offset logic, payer incentives |
| Adoption | Workflow friction, training curve, SOC inertia |
| Incentives | Stakeholder alignment, liability, margin stack |
| Litigation | Malpractice risk by domain |
| Alternatives | SOC superiority threshold |
| Time-to-Proof | Feasibility vs runway |

**Failure Surface Examples:**
- "Good evidence but no predicates" → Regulatory bottleneck
- "Clinical win but payer negative" → Economic bottleneck  
- "Workflow positive, litigation lethal" → Risk bottleneck
- "Cost-saving but no CPT code" → Reimbursement bottleneck

---

## Status Indicators

| Status | Icon | Meaning | Action Required |
|--------|------|---------|-----------------|
| GREEN | 🟢 | Validated / Feasible | Proceed |
| AMBER | 🟡 | Uncertain / Borderline | Needs more work |
| RED | 🔴 | Blocked / Infeasible | Do not proceed until resolved |

---

## Scoring

Each dimension produces a score 0.0 – 1.0:
- Start at 1.0
- AMBER issues: -0.1 to -0.2
- RED issues: -0.3 to -0.5

**Aggregate Scores:**
- `physics` = f(geometry, actuation, materials)
- `clinical` = f(safety, regulatory)
- `manufacturability` = f(materials, workflow)
- `translation` = f(evidence, reimbursement, adoption)

**Overall Status:**
- Any dimension < 0.3 → RED
- Any dimension < 0.6 → AMBER
- All ≥ 0.6 → GREEN

---

## Other OMEGA Terms

| Term | Meaning |
|------|---------|
| **OMEGA-MAX** | Maximum depth mode for researchers — full technical detail, T1-T4 horizons, all substrates |
| **T1/T2/T3/T4** | Time horizons: Now-3mo / 3-12mo / 1-5yr / 5+yr |
| **Trust Score** | 0-1 metric measuring system reliability (first-run success, zero tracebacks, reproducibility) |
| **Substrate** | Shared data layer: vector store (semantic search), knowledge graph (concepts), lineage graph (provenance) |
| **Hypothesis Ledger** | Database of research hypotheses with confidence tracking, falsification cost, SRFC/VRFC status |
| **Falsification Cost** | How expensive to prove wrong? LOW (desk research) / MEDIUM (bench test) / HIGH (clinical trial) |
| **Demo Pack** | Known-good, known-bad, known-edge test cases for validation |
| **Audit Bundle** | Packaged evidence: artifact + validation + contract + logs + optional video |

---

## Time Horizons (T1-T4)

| Horizon | Timeframe | Focus |
|---------|-----------|-------|
| T1 | Now – 3 months | Immediate actions, quick wins, blockers |
| T2 | 3 – 12 months | Medium-term milestones, dependencies |
| T3 | 1 – 5 years | Strategic positioning, capability building |
| T4 | 5+ years | Long-term bets, existential risks |

---

## Substrates (What Can Be Affected)

| Substrate | Examples |
|-----------|----------|
| Materials | Silicones, hydrogels, composites, alloys |
| Compute | Simulation, ML training, edge inference |
| Biology | Gene expression, protein folding, tissue response |
| Manufacturing | 3D printing, molding, assembly, QC |
| Coordination | Multi-agent, human-robot, supply chain |
| Environment | Temperature, humidity, sterility, EM interference |
