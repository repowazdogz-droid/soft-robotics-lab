# 📊 Breakthrough Engine

**Hypothesis Ledger with Translation Trinity Integration**

Track hypotheses from discovery to breakthrough. Confidence decay, falsification cost estimation, and automatic breakthrough detection.

---

## 🚀 Quick Start

```bash
cd products/breakthrough_engine
pip install -r requirements.txt
streamlit run app.py --server.port 8502
```

Or use CLI:

```bash
python hypothesis_ledger.py list
python hypothesis_ledger.py breakthroughs
python hypothesis_ledger.py health
```

---

## 💡 What Problem Does This Solve?

Research ideas get lost. Hypotheses sit untested. Nobody knows what's close to breakthrough.

| Problem | Solution |
|---------|----------|
| Ideas scattered in notebooks | Central ledger with full lineage |
| No confidence tracking | Confidence decays without evidence |
| Unknown testing costs | Falsification cost estimation |
| No translation check | SRFC/VRFC status per hypothesis |
| Breakthroughs missed | Automatic detection when threshold met |

---

## 📦 Features

### 1. Hypothesis Tracking
Every hypothesis tracked with:
- **Claim**: What you're testing
- **Domain**: Research area
- **Confidence**: 0-100% with decay over time
- **Evidence**: Supporting and contradicting
- **Status**: Active, validated, killed, stale
- **SRFC/VRFC**: Translation feasibility

### 2. Confidence Decay
Hypotheses that sit untested become stale:
- Exponential decay (default half-life: 90 days)
- **Stale**: No activity for 60 days
- **Zombie**: Confidence below 20%
- Recommendations: KILL, REVIEW, ATTENTION, HEALTHY

### 3. Falsification Cost Estimation
Know what it costs to test:

| Test Type | Cost Range | Time Range | Resolution |
|-----------|------------|------------|------------|
| Literature Review | $0-500 | 1-7 days | 30% |
| Computational | $100-5K | 7-30 days | 50% |
| Bench Experiment | $1K-20K | 14-90 days | 70% |
| Animal Study | $10K-100K | 60-180 days | 80% |
| Clinical Pilot | $50K-500K | 180-365 days | 85% |
| Clinical Trial | $1M-50M | 1-5 years | 95% |

**Worth testing?** Combines confidence, VRFC status, cost, and resolution probability.

### 4. Breakthrough Detection
A hypothesis becomes a **breakthrough** when:
- Confidence ≥ 85%
- SRFC = GREEN (physically feasible)
- VRFC = GREEN (translation path clear)
- Evidence ≥ 3 supporting items

**Near-breakthroughs** (missing 1-2 criteria) flagged with next steps.

### 5. Health Dashboard
At a glance:
- Total / Active / Healthy / Stale / Zombies
- Per-hypothesis decay analysis
- Kill or review actions
- Batch decay application

---

## 🏗️ Architecture

```
breakthrough_engine/
├── app.py                      # Streamlit UI
├── hypothesis_ledger.py        # Core ledger + CLI
├── hypothesis_decay.py         # Decay calculations
├── falsification_cost.py       # Cost estimation
├── core/
│   ├── visualize.py            # Hypothesis graph
│   └── __init__.py
└── data/
    └── hypothesis_ledger.db    # SQLite storage (repo root data/)
```

---

## 📋 CLI Commands

```bash
# List hypotheses
python hypothesis_ledger.py list
python hypothesis_ledger.py list --status active
python hypothesis_ledger.py list --domain soft_robotics

# Create hypothesis (next_step required)
python hypothesis_ledger.py create \
  "Tendon drives outperform pneumatic for precision" \
  --domain soft_robotics \
  --next-step "Run bench comparison"

# Update confidence
python hypothesis_ledger.py update H-1FFFAB58 \
  --confidence 0.75 \
  --reason "New simulation evidence"

# Kill hypothesis
python hypothesis_ledger.py kill H-1FFFAB58 \
  "Contradicted by physical testing"

# Health check
python hypothesis_ledger.py health
python hypothesis_ledger.py health --apply-decay

# Falsification cost
python hypothesis_ledger.py falsify-cost H-1FFFAB58

# Breakthroughs
python hypothesis_ledger.py breakthroughs
```

*Evidence is added via the Streamlit app or Python API (`ledger.add_evidence(...)`).*

---

## 🔄 Integration with OMEGA Scientist

OMEGA Scientist auto-creates hypotheses from discoveries:

```python
# In OMEGA Scientist (products/omega_scientist)
from integration.ledger_integration import bulk_create_from_scientist_session

# After running discovery modes in Scientist
results = bulk_create_from_scientist_session(st.session_state, auto_add=True)
# Creates hypotheses from contradictions, cross-domain hits, failures, revivals
```

---

## 📊 Hypothesis Lifecycle

```
┌─────────────────────────────────────────────────────────────────┐
│   CREATED → ACTIVE → (gathering evidence) → VALIDATED            │
│                ↓                                                │
│            STALE (no activity)                                  │
│                ↓                                                │
│            ZOMBIE (confidence decayed)                           │
│                ↓                                                │
│            KILLED (falsified or abandoned)                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🎯 Breakthrough Criteria

| Criterion | Threshold | Meaning |
|-----------|-----------|---------|
| Confidence | ≥ 85% | Strong belief based on evidence |
| SRFC | GREEN | Physically feasible |
| VRFC | GREEN | Translation path clear |
| Evidence | ≥ 3 items | Multiple supporting data points |

When all four are met: **🎯 BREAKTHROUGH DETECTED**

---

## 🧪 Example Workflow

```python
from hypothesis_ledger import HypothesisLedger

ledger = HypothesisLedger()

# Create (next_step required for active hypotheses)
h = ledger.create(
    claim="Soft grippers improve egg handling success rate",
    domain="soft_robotics",
    author="warren",
    next_step="Run 50-trial bench test",
)

# Update confidence
ledger.update_confidence(h.id, 0.6, reason="Preliminary bench tests positive")

# Add evidence
ledger.add_evidence(h.id, "50 trials, 94% success rate", "supports", source="lab_notes")

# Check if breakthrough
is_bt, reasons, missing = ledger.is_breakthrough(h.id)

# Get health report
report = ledger.get_health_report()

# Get falsification cost
estimate = ledger.get_falsification_estimate(h.id)
```

---

## 📋 Requirements

```
streamlit>=1.28.0
sqlite3 (built-in)
```

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"Track every idea. Kill the weak ones. Find the breakthroughs."*
