# 📋 Decision Brief

**Strategic Assessment with Translation Trinity**

Generate structured decision briefs with temporal horizons, stakeholder analysis, and SRFC/TSRFC/VRFC validation.

---

## 🚀 Quick Start

```bash
cd products/enterprise/decision_brief
pip install -r requirements.txt
streamlit run app.py --server.port 8507
```

Open http://localhost:8507 in your browser.

*(If no `requirements.txt` in this folder, install from repo root or: `pip install streamlit markdown`.)*

---

## 💡 What Problem Does This Solve?

Strategic decisions lack structure. Translation risks get ignored until too late.

| Problem | Solution |
|---------|----------|
| Unstructured analysis | Standard brief format |
| No temporal thinking | T1–T4 horizon analysis |
| Stakeholder blindness | Explicit stakeholder mapping |
| Translation risk ignored | SRFC/TSRFC/VRFC built-in |
| No audit trail | Full provenance |

---

## 📦 Features

### 1. Structured Brief Format

Every brief includes:
- **Question**: What decision are you making?
- **Context**: Background and constraints
- **Options**: Alternatives with trade-offs
- **Analysis**: Evidence and reasoning
- **Recommendation**: Clear action with confidence
- **Risks**: What could go wrong
- **Timeline**: When to act

### 2. Temporal Horizons

| Horizon | Timeframe | Focus |
|---------|-----------|-------|
| **T1** | 0–3 months | Immediate actions |
| **T2** | 3–12 months | Near-term milestones |
| **T3** | 1–3 years | Strategic positioning |
| **T4** | 3–10 years | Long-term vision |

Each horizon gets separate analysis:
- What's possible in this timeframe?
- What resources required?
- What risks emerge?
- What must be decided now?

### 3. Stakeholder Analysis

Map who's affected:

| Stakeholder | Position | Power | Interest | Strategy |
|-------------|----------|-------|----------|----------|
| Engineering | Supportive | High | High | Engage closely |
| Finance | Neutral | High | Medium | Keep informed |
| Regulatory | Unknown | High | Low | Monitor |
| Customers | Supportive | Medium | High | Involve early |

**Influence mapping**: Who can block? Who can accelerate?

### 4. Translation Trinity

Built-in validation checks:

#### SRFC (Soft Robotics Feasibility Compiler)
*Can it work physically?*

| Status | Meaning |
|--------|---------|
| 🟢 GREEN | Physically feasible with known methods |
| 🟡 AMBER | Feasible but parameters need validation |
| 🔴 RED | Fundamental physics challenges |

#### TSRFC (Translational SRFC)
*What workflow does it replace?*

| Status | Meaning |
|--------|---------|
| 🟢 GREEN | Clear workflow improvement |
| 🟡 AMBER | Workflow benefit uncertain |
| 🔴 RED | No clear workflow fit |

#### VRFC (Validation & Risk Feasibility Compiler)
*Will it survive reality?*

| Status | Meaning |
|--------|---------|
| 🟢 GREEN | Clear translation path |
| 🟡 AMBER | Translation uncertainties |
| 🔴 RED | Major translation blockers |

**VRFC Dimensions:**
- Evidence grade (RCT, registry, bench)
- Regulatory path (510k, PMA, CE)
- Reimbursement (CPT, DRG)
- Adoption friction
- Litigation risk

### 5. Scenario Planning

Model different futures:

| Scenario | Probability | Impact | Response |
|----------|-------------|--------|----------|
| Success | 40% | High positive | Scale up |
| Partial | 35% | Medium | Pivot |
| Failure | 25% | Negative | Exit plan |

**Trigger events**: What signals each scenario?

### 6. Substrate Analysis

Identify underlying assumptions:

```
Substrates:
├── Technical: silicone material properties
├── Market: surgical robotics growth 15%/yr
├── Regulatory: FDA pathway unchanged
└── Competitive: no major entrant next 2 years
```

**Substrate risk**: What if an assumption breaks?

---

## 🏗️ Architecture

```
decision_brief/
├── app.py                      # Streamlit UI
├── decision_brief.py           # Core engine (brief generation, T1–T4, stakeholders, scenarios, SRFC/TSRFC/VRFC)
├── domains/
│   ├── __init__.py            # load_domain_model, list_domains
│   ├── business.json
│   ├── research.json
│   ├── robotics.json
│   └── synthetic_biology.json
├── decision_brief.json        # Example output
├── decision_brief.md           # Example output
├── substrate_integration.py    # Past decisions, record_decision
└── examples/                  # Sample briefs
```

---

## 🔄 Brief Generation Flow

```
┌─────────────────────────────────────────────────────────────────┐
│   1. Input question + context                                   │
│   2. Run Translation Trinity (SRFC/TSRFC/VRFC)                   │
│   3. Analyze temporal horizons (T1–T4)                          │
│   4. Map stakeholders                                            │
│   5. Generate scenarios                                          │
│   6. Identify substrates and risks                               │
│   7. Produce recommendation with confidence                      │
│   8. Export formatted brief                                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Brief Templates

### Standard Decision Brief
```markdown
# Decision Brief: [Title]

## Question
[What decision needs to be made?]

## Translation Trinity
- SRFC: [GREEN/AMBER/RED] — [reason]
- TSRFC: [GREEN/AMBER/RED] — [reason]
- VRFC: [GREEN/AMBER/RED] — [reason]

## Temporal Analysis
### T1 (0–3 months)
[Immediate actions and risks]

### T2 (3–12 months)
[Near-term milestones]

## Stakeholders
[Stakeholder map and strategies]

## Recommendation
[Clear action with confidence level]

## Risks
[What could go wrong]
```

### Go/No-Go Brief
Simplified format for binary decisions:
- What we're deciding
- Key criteria
- Score per criterion
- Recommendation: GO or NO-GO

---

## 🔌 Integration

### From OMEGA Scientist
```python
# Hypothesis identified
hypothesis = scientist.top_hypothesis()

# Generate brief
brief = generate_brief(
    f"Should we pursue: {hypothesis.claim}?",
    params={"context": hypothesis.evidence},
)
```

### From Hypothesis Ledger
```python
# Near-breakthrough hypothesis
near = ledger.get_near_breakthroughs()[0]

# Assess what's missing
brief = generate_brief(
    f"How to get {near.id} to breakthrough?",
    params={"context": f"Missing: {near.missing}"},
)
```

---

## 🧪 Example Workflow

```python
from decision_brief import generate_brief

# Generate brief
brief = generate_brief(
    "Should we develop a soft surgical gripper for laparoscopic procedures?",
    params={
        "horizon": 18,
        "budget": 500_000,
    },
)

# Recommendation
print(brief.recommended_next_action)
print(brief.overall_status)

# Translation Trinity
print(f"SRFC: {brief.srfc_status} — {brief.srfc_reason}")
print(f"TSRFC: {brief.tsrfc_status} — {brief.tsrfc_reason}")
print(f"VRFC: {brief.vrfc_status} — {brief.vrfc_reason}")

# Temporal (T1–T4)
from decision_brief import TEMPORAL_LABELS
for key, label in TEMPORAL_LABELS.items():
    d = getattr(brief, f"{key}_implications", {}) or {}
    print(f"{label}: {d.get('implications', [])}")

# Export
brief.to_markdown("surgical_gripper_decision.md")
brief.to_json("surgical_gripper_decision.json")
```

---

## 📁 Output Formats

| Format | Use |
|--------|-----|
| **Markdown** | Documentation, GitHub |
| **PDF** | Formal reports (export MD then convert) |
| **HTML** | Web presentation (export MD then convert) |
| **JSON** | Programmatic access |

---

## 🎯 Use Cases

1. **Investment decisions**: Should we fund this project?
2. **Go/No-Go gates**: Ready for next phase?
3. **Strategic planning**: Where to focus next year?
4. **Risk assessment**: What could derail this?
5. **Partnership evaluation**: Should we collaborate?

---

## 📋 Requirements

```
streamlit>=1.28.0
markdown>=3.4.0
```

Optional for LLM assistance:
- OpenAI API or local LLM

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"Structured decisions. Translation awareness. Clear recommendations."*
