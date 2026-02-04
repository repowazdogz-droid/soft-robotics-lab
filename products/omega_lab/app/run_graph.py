"""
Run Graph

Visualise experiment lineage - forks, branches, parameter families.
Breakthroughs emerge from constellations of runs, not single runs.
"""

import json
from pathlib import Path
from typing import Dict, List, Optional
from collections import defaultdict

ARTIFACTS_DIR = Path(__file__).parent.parent / "artifacts"


def build_run_graph() -> Dict:
    """
    Build graph of all runs showing lineage relationships.

    Returns:
        {
            "nodes": [{"id": "R-...", "hypothesis": "H-...", "outcome": "supports", ...}],
            "edges": [{"from": "R-parent", "to": "R-child"}],
            "roots": ["R-..."],  # Runs with no parent
            "branches": {"H-TEST": ["R-1", "R-2", ...]},  # Runs by hypothesis
            "batches": {"B-...": ["R-1", "R-2", ...]}  # Runs by batch
        }
    """
    nodes = []
    edges = []
    roots = []
    branches = defaultdict(list)
    batches = defaultdict(list)

    # Scan all artifact directories
    if not ARTIFACTS_DIR.exists():
        return {"nodes": [], "edges": [], "roots": [], "branches": {}, "batches": {}}

    for run_dir in ARTIFACTS_DIR.iterdir():
        if not run_dir.is_dir() or not run_dir.name.startswith("R-"):
            continue

        run_json = run_dir / "run.json"
        if not run_json.exists():
            continue

        try:
            with open(run_json) as f:
                bundle = json.load(f)

            run_id = bundle.get("run_id", run_dir.name)
            parent_id = bundle.get("parent_run_id")
            hypothesis_id = bundle.get("hypothesis_id")
            batch_id = bundle.get("batch_id")

            # Build node
            node = {
                "id": run_id,
                "hypothesis_id": hypothesis_id,
                "design_id": bundle.get("design_id"),
                "environment": bundle.get("environment"),
                "outcome": bundle.get("outcome", {}).get("direction"),
                "strength": bundle.get("outcome", {}).get("strength"),
                "batch_id": batch_id,
                "parent_run_id": parent_id,
                "timestamp": bundle.get("timestamp"),
                "metrics": bundle.get("metrics", {})
            }
            nodes.append(node)

            # Track lineage
            if parent_id:
                edges.append({"from": parent_id, "to": run_id})
            else:
                roots.append(run_id)

            # Track by hypothesis
            if hypothesis_id:
                branches[hypothesis_id].append(run_id)

            # Track by batch
            if batch_id:
                batches[batch_id].append(run_id)

        except Exception as e:
            print(f"Error reading {run_json}: {e}")
            continue

    return {
        "nodes": nodes,
        "edges": edges,
        "roots": roots,
        "branches": dict(branches),
        "batches": dict(batches)
    }


def get_run_ancestry(run_id: str) -> List[str]:
    """Get full ancestry chain for a run (parent → grandparent → ...)"""
    graph = build_run_graph()
    node_map = {n["id"]: n for n in graph["nodes"]}

    ancestry = []
    current = run_id

    while current:
        node = node_map.get(current)
        if not node:
            break
        ancestry.append(current)
        current = node.get("parent_run_id")

    return ancestry


def get_run_descendants(run_id: str) -> List[str]:
    """Get all descendants of a run"""
    graph = build_run_graph()

    # Build child map
    children = defaultdict(list)
    for edge in graph["edges"]:
        children[edge["from"]].append(edge["to"])

    # BFS to find all descendants
    descendants = []
    queue = [run_id]

    while queue:
        current = queue.pop(0)
        for child in children.get(current, []):
            descendants.append(child)
            queue.append(child)

    return descendants


def get_hypothesis_tree(hypothesis_id: str) -> Dict:
    """Get the full experiment tree for a hypothesis"""
    graph = build_run_graph()

    # Filter to hypothesis
    hyp_runs = [n for n in graph["nodes"] if n["hypothesis_id"] == hypothesis_id]
    hyp_run_ids = {n["id"] for n in hyp_runs}
    hyp_edges = [e for e in graph["edges"] if e["from"] in hyp_run_ids or e["to"] in hyp_run_ids]

    # Find roots for this hypothesis
    children = {e["to"] for e in hyp_edges}
    roots = [n["id"] for n in hyp_runs if n["id"] not in children]

    return {
        "hypothesis_id": hypothesis_id,
        "runs": hyp_runs,
        "edges": hyp_edges,
        "roots": roots,
        "total": len(hyp_runs),
        "outcomes": {
            "supports": sum(1 for n in hyp_runs if n["outcome"] == "supports"),
            "refutes": sum(1 for n in hyp_runs if n["outcome"] == "refutes"),
            "ambiguous": sum(1 for n in hyp_runs if n["outcome"] == "ambiguous")
        }
    }


def export_graph_for_viz(output_path: str = None) -> Dict:
    """
    Export graph in format suitable for visualization (D3, vis.js, etc.)
    """
    graph = build_run_graph()

    # Convert to vis.js format
    viz_data = {
        "nodes": [
            {
                "id": n["id"],
                "label": f"{n['id'][:12]}...\n{n['outcome'] or 'pending'}",
                "group": n["hypothesis_id"],
                "title": f"Design: {n['design_id']}\nEnv: {n['environment']}\nOutcome: {n['outcome']} ({n['strength']})"
            }
            for n in graph["nodes"]
        ],
        "edges": [
            {"from": e["from"], "to": e["to"]}
            for e in graph["edges"]
        ]
    }

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(viz_data, f, indent=2)

    return viz_data


def print_graph_summary():
    """Print a text summary of the run graph"""
    graph = build_run_graph()

    print("\n" + "="*60)
    print("RUN GRAPH SUMMARY")
    print("="*60)

    print(f"\nTotal runs: {len(graph['nodes'])}")
    print(f"Root runs (no parent): {len(graph['roots'])}")
    print(f"Lineage edges: {len(graph['edges'])}")

    print(f"\nHypotheses: {len(graph['branches'])}")
    for hyp, runs in graph['branches'].items():
        print(f"  {hyp}: {len(runs)} runs")

    print(f"\nBatches: {len(graph['batches'])}")
    for batch, runs in list(graph['batches'].items())[:5]:
        print(f"  {batch}: {len(runs)} runs")
    if len(graph['batches']) > 5:
        print(f"  ... and {len(graph['batches']) - 5} more")

    # Outcome distribution
    outcomes = defaultdict(int)
    for node in graph['nodes']:
        outcomes[node.get('outcome', 'unknown')] += 1

    print(f"\nOutcomes:")
    for outcome, count in outcomes.items():
        print(f"  {outcome}: {count}")

    print("="*60 + "\n")


if __name__ == "__main__":
    print_graph_summary()

    # Export for visualization
    out_path = Path(__file__).parent.parent / "registry" / "run_graph.json"
    export_graph_for_viz(str(out_path))
    print(f"Exported to {out_path}")
