# OMEGA Lab Operating Principles

> The Run Bundle is the atomic unit of research.
> All knowledge flows from artifacts, not services.

## Core Beliefs

1. **Artifacts are truth** - Services can be rebuilt. Artifacts cannot be recreated.
2. **Append-only history** - Runs never overwrite. History is sacred.
3. **Lineage is everything** - Every run knows its parent, batch, and hypothesis.
4. **Compute is disposable** - Simulations can be rerun. Provenance cannot.
5. **Structure before intelligence** - Index first, then layer ML/anomaly detection.

## Canonical Artifact: Run Bundle

Every experiment produces a Run Bundle containing:

```json
{
  "run_id": "R-YYYYMMDD-HHMMSS-fff",
  "parent_run_id": "R-... | null",
  "batch_id": "B-YYYYMMDD-HHMMSS | null",
  "hypothesis_id": "H-...",
  "experiment_id": "E-...",
  "design_id": "GD-...",
  "schema_version": "1.0",
  "created_at": "ISO8601",
  "origin": "runner | manual | simulation",
  "engine": "isaac_sim | mock | mujoco",
  "environment": "dry | wet | surgical",
  "metrics": {},
  "outcome": {
    "direction": "supports | refutes | ambiguous",
    "strength": 0.0-1.0,
    "rationale": ""
  },
  "artifact_manifest": [],
  "notes": ""
}
```

## Confidence Update Logic

```
if direction == "supports":
    delta = +0.05 * strength
elif direction == "refutes":
    delta = -0.05 * strength
else:  # ambiguous
    delta = +0.01

new_confidence = clamp(current + delta, 0.05, 0.95)
Never allow absolute certainty. Reality is noisy.
```

## Artifact Locations

| Artifact | Path |
|----------|------|
| Run bundles | `artifacts/{run_id}/run.json` |
| Simulation exports | `artifacts/{run_id}/` |
| Database | `db/omega.db` |
| Experiment registry | `registry/experiment_index.json` |

## Failure Philosophy

- Runs never overwrite
- Artifacts are append-only
- History is sacred
- Graceful degradation (save locally if Lab OS unreachable)

## Architectural Red Lines

Never cross these:

- Never allow simulation logic inside Lab OS (Lab OS = cognition, Runners = action)
- Never mutate historical artifacts
- Never delete runs (archive instead)
- Never bypass the Run Bundle schema

## Schema Versioning

All artifacts include `schema_version`. Current: `"1.0"`

Before ANY schema change:

1. Increment version
2. Document migration path
3. Ensure backward compatibility

## The Test

**"If the Mac Mini died tonight, could I rebuild the lab from artifacts alone?"**

The answer must always be **YES**.
