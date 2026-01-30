# OMEGA Evaluation Framework

Regression testing and drift detection for OMEGA decision support.

## What It Does

- Define expected outcomes for key queries
- Run tests on-demand or scheduled
- Alert if confidence drops or recommendations change
- Track drift over time

## Usage

```python
from omega_eval import EvalSuite

suite = EvalSuite("my_suite")

suite.add_test(
    query="Should we expand into market X?",
    expected_confidence_min=0.5,
    description="Market expansion query",
    critical=True
)

result = suite.run()
print(result.summary())
```

## CLI

```bash
# Run default suite (from repo root)
python products/enterprise/eval_framework/omega_eval.py

# Run with specific tags
python products/enterprise/eval_framework/omega_eval.py --tags core spine

# Save results
python products/enterprise/eval_framework/omega_eval.py --output eval_results.json
```

## Drift Detection

The framework automatically compares results against historical runs and alerts if:

- Confidence changes by more than 15%
- Recommendations change significantly

History is stored in `eval_history/` and kept for 30 runs.
