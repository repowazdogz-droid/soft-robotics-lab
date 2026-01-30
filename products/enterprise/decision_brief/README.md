# OMEGA Decision Brief Generator

Converts natural language queries into executive decision briefs with uncertainty quantification.

## What It Does

- Takes a business/operational question
- Runs OMEGA simulation across relevant domains
- Analyzes cross-domain effects
- Assesses reversibility and approval requirements
- Outputs structured brief with:
  - Recommendation + confidence interval
  - Risk breakdown
  - Alternative scenarios compared
  - Key assumptions
  - What information would change the recommendation
  - Full audit trail

## Usage

```python
from decision_brief import generate_brief

brief = generate_brief("Should we dual-source suppliers given chip shortage risks?")

# Export
brief.to_markdown("decision.md")
brief.to_json("decision.json")
```

## CLI

```bash
python decision_brief.py "Should we expand into market X?" --output my_decision --format both
```

From repo root:

```bash
cd C:\Users\Warren\OmegaStack
python products/enterprise/decision_brief/decision_brief.py "Should we dual-source suppliers given chip shortage risks?" --output test_brief
```

## Output Example

See `examples/` folder for sample briefs.

## Key Features

- **Uncertainty Quantification**: Every recommendation includes confidence intervals
- **Reversibility Gates**: Flags decisions that need human approval
- **Cross-Domain Analysis**: Shows how decision affects multiple business areas
- **Audit Trail**: Full traceability of how recommendation was generated
