# Isaac Sim MJCF → USD Workflow

## Inside Isaac Sim Python

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp()

# Enable MJCF importer extension
import omni.kit.commands
omni.kit.commands.execute('OMNIImportMJCF', ...)

# Save to USD
stage.Export("artifacts/{run_id}/gripper.usd")
```

## Principles

- USD becomes canonical artifact
- Keep paths relative
- Never hardcode machine paths
