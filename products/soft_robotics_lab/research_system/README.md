# OMEGA Research Memory

Private research OS for a lab's entire corpus.

## What It Does

- **Ingest** PDFs, notes, slides, protocols
- **Search** with keyword + semantic + metadata filters
- **Track** structured research objects (hypotheses, experiments, results)
- **Generate** insight briefs with full traceability
- **Audit** everything

## Quick Start

```python
from research_memory import ResearchMemory

# Initialize for your lab
memory = ResearchMemory("rossiter_lab")

# Ingest documents
memory.ingest("papers/", project="soft_grippers")
memory.ingest("notes/", project="soft_grippers")

# Search with sources
result = memory.query_with_sources("What do we know about variable stiffness?")
print(result['answer'])
print(result['sources'])  # Full provenance

# Create hypothesis
hyp = memory.create_hypothesis(
    claim="Tendon-driven grippers outperform pneumatic for delicate objects",
    scope="Objects < 100g, compliance > 0.5",
    assumptions=["Constant tendon tension", "Room temperature"],
    predictions=["Lower damage rate", "Higher success on eggs"]
)

# Generate weekly brief
brief = memory.generate_weekly_brief()
print(brief.to_markdown())
```

## Semantic Search (Optional)

For meaning-based search (not just keyword match), install:

```bash
pip install sentence-transformers
```

- **Ingest**: New chunks are embedded automatically; embeddings are stored in `chunk_embeddings`.
- **Search**: Uses cosine similarity over embeddings when available; falls back to keyword (LIKE) otherwise.
- **Reindex**: Use "Reindex embeddings" in the Research Memory UI (Ingest tab) or re-ingest to backfill embeddings for existing chunks.
- **CLI**: `python research_memory.py --lab rossiter search "variable stiffness"` uses semantic by default; add `--no-semantic` for keyword-only.

Model: `all-MiniLM-L6-v2` (small, fast, ~80MB).

## CLI

```bash
# Ingest documents
python research_memory.py --lab rossiter ingest papers/ --project soft_grippers

# Search (semantic by default)
python research_memory.py --lab rossiter search "variable stiffness gripper"
python research_memory.py --lab rossiter search "variable stiffness" --no-semantic

# List hypotheses
python research_memory.py --lab rossiter hypothesis list

# Generate weekly brief
python research_memory.py --lab rossiter brief

# Show stats
python research_memory.py --lab rossiter stats
```

## Research Objects

### Hypothesis Card

- Claim (mechanistic, falsifiable)
- Scope & assumptions
- Competing hypotheses
- Predictions
- Evidence links (supporting/weakening)
- Status: active / weakened / falsified / merged

### Experiment Card

- Aim (which hypotheses it tests)
- Design, variables, controls
- Expected outcomes per hypothesis
- Results links

### Result Card

- Summary with uncertainty flags
- Which hypotheses it updates
- Negative result support (first-class)
- Replication notes

## Insight Engines

1. **Contradiction Finder** - Claims that can't all be true
2. **Assumption Surfacer** - Implicit assumptions in hypotheses
3. **Discriminator Generator** - Smallest tests to separate hypotheses
4. **Weekly Brief** - Automated insight summary

## Privacy & Audit

- All data stored locally (no cloud)
- Full audit trail of every action
- Every insight linked to source chunks
- Speculation explicitly flagged

---

## Test

```bash
# Create test documents
mkdir -p test_docs
echo "# Variable Stiffness Grippers\n\nOur research shows that variable stiffness improves grasp success by 23%." > test_docs/notes.md

# Test the system
python products/soft_robotics_lab/research_system/research_memory.py --lab test_lab ingest test_docs/
python products/soft_robotics_lab/research_system/research_memory.py --lab test_lab search "variable stiffness"
python products/soft_robotics_lab/research_system/research_memory.py --lab test_lab stats
```
