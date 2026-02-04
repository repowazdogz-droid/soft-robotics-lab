# 🔬 OMEGA Scientist

**Discovery Engine for Research Translation**

Find contradictions, cross-domain connections, failures, and stalled discoveries — then validate which ones will survive reality.

---

## 🚀 Quick Start

```bash
cd products/omega_scientist
pip install -r requirements.txt
streamlit run app.py --server.port 8506
```

Open http://localhost:8506 in your browser.

---

## 💡 What Problem Does This Solve?

Most AI research tools ask: *"What do the papers say?"*

OMEGA Scientist asks: *"What will actually work?"*

| Traditional | OMEGA Scientist |
|-------------|-----------------|
| Summarize papers | Find contradictions between papers |
| Search by keyword | Connect ideas across domains |
| Read what succeeded | Mine what failed |
| Surface findings | Validate translation path |

---

## 📦 Features

### 1. Paper Parser
Extract structured information from research PDFs:
- **Claims** with confidence scores and evidence
- **Methods** and experimental details
- **Dataset links** (GEO, SRA, GitHub, Zenodo, etc.)
- **References** and citations

### 2. Discovery Modes

#### Contradiction Mining
Find where papers disagree — discovery opportunities hide in contradictions.
- Direct contradictions (A says X, B says not-X)
- Magnitude differences (>30% disagreement in quantified claims)
- Resolution hypotheses auto-generated

#### Cross-Domain Collision
Connect research across fields that don't normally talk.
- 10 domains: robotics, biology, materials, medicine, chemistry, physics, computing, engineering, neuroscience, genetics
- Structural, functional, mechanistic, and analogical connections
- Synthesis opportunities when ≥2 connections exist

#### Failure Analysis
Mine what didn't work — everyone reads successes, you read failures.
- Method failures, condition failures, partial successes
- "Failed to", "did not work", "worse than" pattern detection
- Fix hypotheses for fixable failures

#### Translation Gap Finder
Find stalled discoveries ready for revival.
- 5-10 year old discoveries in the "sweet spot"
- Blocker analysis (technical, regulatory, funding, manufacturing)
- Revival potential scoring

### 3. Hypothesis Ranking
Score discoveries by what matters:

| Score | Weight | What It Measures |
|-------|--------|------------------|
| Novelty | 15% | How new is this? |
| Feasibility | 35% | Can it work? (SRFC-like) |
| Translation | 30% | Will it survive reality? (VRFC-like) |
| Impact | 20% | How big if it works? |

### 4. Validation Integration
- **SRFC Check**: Can it work physically?
- **TSRFC Check**: What workflow does it replace?
- **VRFC Check**: Will it survive regulatory, reimbursement, adoption?
- **Full Trinity**: All three together

### 5. Hypothesis Ledger Integration
One-click export discoveries to the Hypothesis Ledger:
- Auto-creates hypotheses from contradictions, collisions, failures, revivals
- Preserves source papers and evidence
- Tracks through to breakthrough

---

## 🏗️ Architecture

```
omega_scientist/
├── app.py                      # Streamlit UI
├── core/
│   ├── paper_parser.py         # Orchestrator
│   ├── pdf_reader.py           # Extract text, sections, references
│   ├── claim_extractor.py      # Identify claims with confidence
│   ├── method_extractor.py     # Extract methodology
│   ├── data_linker.py          # Find dataset links
│   ├── contradiction_miner.py  # Find disagreements
│   ├── cross_domain.py         # Connect fields
│   ├── failure_analyzer.py     # Mine failures
│   ├── translation_gap.py      # Find revivals
│   └── hypothesis_ranker.py    # Score & rank
├── integration/
│   └── ledger_integration.py   # → Hypothesis Ledger
├── data/papers/                # Downloaded PDFs
└── outputs/parsed/             # Parsed paper JSONs
```

---

## 🔄 The Discovery → Breakthrough Loop

```
┌─────────────────────────────────────────────────────────────────┐
│   1. Parse papers                                               │
│   2. Run discovery modes (contradictions, collisions, etc.)     │
│   3. Rank hypotheses by feasibility × translation × impact      │
│   4. Send to Hypothesis Ledger                                  │
│   5. Track confidence, gather evidence                          │
│   6. Detect breakthrough when threshold met                     │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Comparison: OMEGA vs Alternatives

| Metric | Kosmos AI Scientist | OMEGA Scientist |
|--------|---------------------|-----------------|
| Papers per run | 1,500 | 50-100 |
| Approach | Brute force | Intelligence + targeting |
| Reproducibility | 79% | 100% auditable |
| Translation check | None | VRFC built-in |
| Cross-domain | Single domain | Multi-domain synthesis |
| Failure mining | No | Yes |
| Revival candidates | No | Yes |

**Positioning:** "Kosmos finds discoveries. OMEGA tells you which ones will survive reality."

---

## 🧪 Example Workflow

```python
# 1. Parse a paper
from core.paper_parser import parse_paper, save_parsed

result = parse_paper("paper.pdf", do_extract_claims=True, find_data=True)
save_parsed(result, "outputs/parsed/paper_parsed.json")

# 2. Find contradictions across multiple papers
from core.contradiction_miner import find_contradictions, ContradictionReport

parsed_list = [result1, result2, result3]  # ParsedPaperOutput or dicts
report = find_contradictions(parsed_list)

# 3. Rank the discoveries (build hypothesis list from reports, then rank)
from core.hypothesis_ranker import rank_all_hypotheses

hypotheses = []  # Build from report.contradictions, collision_report, etc.
ranked = rank_all_hypotheses(hypotheses)

# 4. Send to ledger (e.g. from Streamlit session_state)
from integration.ledger_integration import bulk_create_from_scientist_session

results = bulk_create_from_scientist_session(session_state, auto_add=True)
```

---

## 🎯 Use Cases

1. **Literature review**: Find gaps and contradictions in a research area
2. **Grant writing**: Identify high-potential hypotheses with translation paths
3. **Lab planning**: Prioritize experiments by falsification cost
4. **Collaboration**: Find cross-domain connection opportunities
5. **IP scouting**: Identify stalled discoveries ready for revival

---

## 📋 Requirements

```
streamlit>=1.28.0
pymupdf>=1.23.0
requests>=2.31.0
```

Optional for LLM-enhanced extraction:
- Local LLM via LM Studio (port 1234)
- Or OpenAI API key

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"The translation layer between research and reality."*
