"""
Semantic Experiment Query

Query the experiment space with natural language-like filters.
"Show me all soft grippers that improved confidence in wet environments with compliance > 0.7"
"""

import json
import re
from pathlib import Path
from typing import List, Dict, Optional, Any
from dataclasses import dataclass

ARTIFACTS_DIR = Path(__file__).parent.parent / "artifacts"
REGISTRY_PATH = Path(__file__).parent.parent / "registry" / "experiment_index.json"


@dataclass
class QueryFilter:
    """Filter for semantic queries"""
    hypothesis_id: Optional[str] = None
    design_id: Optional[str] = None
    environment: Optional[str] = None
    outcome: Optional[str] = None  # supports, refutes, ambiguous
    min_strength: Optional[float] = None
    max_strength: Optional[float] = None
    min_slip_rate: Optional[float] = None
    max_slip_rate: Optional[float] = None
    min_contact_area: Optional[float] = None
    max_contact_area: Optional[float] = None
    batch_id: Optional[str] = None
    has_parent: Optional[bool] = None
    novelty_only: bool = False


def load_all_runs() -> List[Dict]:
    """Load all run bundles from artifacts"""
    runs = []

    if not ARTIFACTS_DIR.exists():
        return runs

    for run_dir in ARTIFACTS_DIR.iterdir():
        if not run_dir.is_dir() or not run_dir.name.startswith("R-"):
            continue

        run_json = run_dir / "run.json"
        if not run_json.exists():
            continue

        try:
            with open(run_json) as f:
                runs.append(json.load(f))
        except Exception:
            continue

    return runs


def query_runs(filter: QueryFilter) -> List[Dict]:
    """
    Query runs with semantic filter.

    Examples:
        # All supporting runs in wet environment
        query_runs(QueryFilter(environment="wet", outcome="supports"))

        # High-strength refutations
        query_runs(QueryFilter(outcome="refutes", min_strength=0.8))

        # Low slip rate runs
        query_runs(QueryFilter(max_slip_rate=0.1))
    """
    runs = load_all_runs()
    results = []

    for run in runs:
        # Filter by hypothesis
        if filter.hypothesis_id and run.get("hypothesis_id") != filter.hypothesis_id:
            continue

        # Filter by design
        if filter.design_id and run.get("design_id") != filter.design_id:
            continue

        # Filter by environment
        if filter.environment and run.get("environment") != filter.environment:
            continue

        # Filter by outcome
        outcome = run.get("outcome", {})
        if filter.outcome and outcome.get("direction") != filter.outcome:
            continue

        # Filter by strength
        strength = outcome.get("strength", 0)
        if filter.min_strength and strength < filter.min_strength:
            continue
        if filter.max_strength and strength > filter.max_strength:
            continue

        # Filter by metrics
        metrics = run.get("metrics", {})

        slip_rate = metrics.get("slip_rate")
        if slip_rate is not None:
            if filter.min_slip_rate and slip_rate < filter.min_slip_rate:
                continue
            if filter.max_slip_rate and slip_rate > filter.max_slip_rate:
                continue

        contact_area = metrics.get("contact_area")
        if contact_area is not None:
            if filter.min_contact_area and contact_area < filter.min_contact_area:
                continue
            if filter.max_contact_area and contact_area > filter.max_contact_area:
                continue

        # Filter by batch
        if filter.batch_id and run.get("batch_id") != filter.batch_id:
            continue

        # Filter by lineage
        if filter.has_parent is not None:
            has_parent = run.get("parent_run_id") is not None
            if filter.has_parent != has_parent:
                continue

        results.append(run)

    return results


def natural_query(query_string: str) -> List[Dict]:
    """
    Parse natural language-like query into filters.

    Examples:
        "wet environment supports"
        "slip_rate < 0.1"
        "hypothesis H-TEST refutes"
        "batch B-20260202"
    """
    query = query_string.lower()
    filter = QueryFilter()

    # Environment
    for env in ["dry", "wet", "surgical"]:
        if env in query:
            filter.environment = env
            break

    # Outcome
    for outcome in ["supports", "refutes", "ambiguous"]:
        if outcome in query:
            filter.outcome = outcome
            break

    # Hypothesis
    if "h-" in query:
        match = re.search(r'h-[\w]+', query, re.IGNORECASE)
        if match:
            filter.hypothesis_id = match.group().upper()

    # Batch
    if "b-" in query:
        match = re.search(r'b-[\w-]+', query, re.IGNORECASE)
        if match:
            filter.batch_id = match.group().upper()

    # Slip rate
    if "slip" in query:
        match = re.search(r'slip[_\s]*(?:rate)?[_\s]*[<>]=?\s*([\d.]+)', query)
        if match:
            value = float(match.group(1))
            if "<" in query:
                filter.max_slip_rate = value
            else:
                filter.min_slip_rate = value

    # Strength
    if "strength" in query:
        match = re.search(r'strength[_\s]*[<>]=?\s*([\d.]+)', query)
        if match:
            value = float(match.group(1))
            if "<" in query:
                filter.max_strength = value
            else:
                filter.min_strength = value

    return query_runs(filter)


def query_summary(results: List[Dict]) -> Dict:
    """Summarize query results"""
    if not results:
        return {"count": 0}

    outcomes = {"supports": 0, "refutes": 0, "ambiguous": 0}
    environments = {}
    slip_rates = []

    for r in results:
        outcome = r.get("outcome", {}).get("direction")
        if outcome in outcomes:
            outcomes[outcome] += 1

        env = r.get("environment", "unknown")
        environments[env] = environments.get(env, 0) + 1

        slip = r.get("metrics", {}).get("slip_rate")
        if slip is not None:
            slip_rates.append(slip)

    return {
        "count": len(results),
        "outcomes": outcomes,
        "environments": environments,
        "avg_slip_rate": sum(slip_rates) / len(slip_rates) if slip_rates else None,
        "min_slip_rate": min(slip_rates) if slip_rates else None,
        "max_slip_rate": max(slip_rates) if slip_rates else None
    }


if __name__ == "__main__":
    print("\n" + "="*60)
    print("SEMANTIC QUERY EXAMPLES")
    print("="*60)

    # Example queries
    queries = [
        "wet environment",
        "supports",
        "slip_rate < 0.1",
        "H-TEST refutes",
        "dry supports strength > 0.8"
    ]

    for q in queries:
        print(f"\nQuery: '{q}'")
        results = natural_query(q)
        summary = query_summary(results)
        print(f"  Found: {summary['count']} runs")
        if summary['count'] > 0:
            print(f"  Outcomes: {summary['outcomes']}")
            if summary['avg_slip_rate']:
                print(f"  Avg slip rate: {summary['avg_slip_rate']:.4f}")
