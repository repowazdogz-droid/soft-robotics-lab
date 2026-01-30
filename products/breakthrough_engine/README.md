# OMEGA Breakthrough Engine

Scientific discovery acceleration through structured hypothesis management.

## Components

### Hypothesis Ledger

A living, versioned registry of explicit causal hypotheses.

```bash
# Create a hypothesis
python hypothesis_ledger.py create "Drug X inhibits pathway Y" --domain drug_discovery

# List all active
python hypothesis_ledger.py list --status active

# Show hypothesis card
python hypothesis_ledger.py show H-ABC12345

# Update confidence
python hypothesis_ledger.py update H-ABC12345 0.7 --reason "Experiment 1 positive"

# Kill a hypothesis
python hypothesis_ledger.py kill H-ABC12345 "No longer worth pursuing"
```

### Weekly Review Generator

Auto-generates review agendas from active work.

```bash
# Generate and print
python weekly_review.py generate

# Save to file
python weekly_review.py generate --output review.md
```

## Key Rules

1. No experiment exists without a parent hypothesis
2. No hypothesis survives without exposure to falsification
3. Killing a hypothesis is a win, not a loss
