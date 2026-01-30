# OMEGA Dashboard (CLI)

View queries, experiments, alerts, and audit trail.

## Commands

```bash
# Show statistics
python omega_dashboard.py stats

# Show recent queries
python omega_dashboard.py recent
python omega_dashboard.py recent --limit 50 --domain spine_surgery

# Show experiments
python omega_dashboard.py experiments
python omega_dashboard.py experiments --status active

# Show alerts
python omega_dashboard.py alerts
python omega_dashboard.py alerts --all

# Show audit trail for specific query
python omega_dashboard.py audit DB-20260128-0001
```

## Features

- Query history with confidence levels
- Experiment tracking (integrates with omega_lab)
- Alert system for low-confidence or approval-required queries
- Full audit trail per query
- Statistics overview
