"""
World Model Studio — Substrate integration (vector store, knowledge graph, lineage).
Store scenes, training runs; link TrainingRun → uses Design, on Scene; lineage trained_policy → derived_from training_run.
"""

from pathlib import Path
from typing import Any, Dict, List, Optional

_PRODUCTS = Path(__file__).resolve().parent.parent
if str(_PRODUCTS) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(_PRODUCTS))

_vector_store = None
_knowledge_graph = None
_lineage_graph = None
_NodeType = None
_EdgeType = None

try:
    from shared.substrate.vector_store import vector_store as _vector_store
except Exception:
    pass
try:
    from shared.substrate.knowledge_graph import knowledge_graph as _knowledge_graph
    from shared.substrate.knowledge_graph import NodeType as _NodeType
    from shared.substrate.knowledge_graph import EdgeType as _EdgeType
except Exception:
    pass
try:
    from shared.substrate.lineage import lineage_graph as _lineage_graph
except Exception:
    pass


def record_training_to_substrate(
    run_id: str,
    gripper_id: str,
    scene_id: str,
    results: Dict[str, Any],
    policy_id: Optional[str] = None,
) -> None:
    """
    Store training run in vector_store('training_runs'), add Experiment node,
    link to Design (gripper) and Scene (related_to). Record lineage: policy_id derived_from run_id.
    """
    if _vector_store is not None:
        try:
            text = f"run={run_id} gripper={gripper_id} scene={scene_id} success_rate={results.get('success_rate', 0)}"
            _vector_store.add(
                "training_runs",
                text,
                metadata={"run_id": run_id, "gripper_id": gripper_id, "scene_id": scene_id, **{k: v for k, v in results.items() if isinstance(v, (str, int, float, bool))}},
            )
        except Exception:
            pass
    if _knowledge_graph is not None and _NodeType is not None and _EdgeType is not None:
        try:
            _knowledge_graph.add_node(
                _NodeType.Experiment,
                run_id,
                properties={"run_id": run_id, "gripper_id": gripper_id, "scene_id": scene_id, "success_rate": results.get("success_rate"), "avg_reward": results.get("avg_reward")},
            )
            if gripper_id:
                if not _knowledge_graph.get_node(gripper_id):
                    _knowledge_graph.add_node(_NodeType.Design, gripper_id, properties={"name": gripper_id})
                _knowledge_graph.add_edge(run_id, gripper_id, _EdgeType.related_to, {"relation": "uses"})
            if scene_id:
                scene_node = f"scene_{scene_id}"
                if not _knowledge_graph.get_node(scene_node):
                    _knowledge_graph.add_node(_NodeType.Concept, scene_node, properties={"name": scene_id})
                _knowledge_graph.add_edge(run_id, scene_node, _EdgeType.related_to, {"relation": "on"})
        except Exception:
            pass
    if _lineage_graph is not None and policy_id:
        try:
            _lineage_graph.record(policy_id, run_id, "derived_from", parameters={"type": "trained_policy"})
        except Exception:
            pass


def find_similar_training(task_type: str, gripper_type: Optional[str] = None, n: int = 5) -> List[Dict[str, Any]]:
    """Search vector store 'training_runs' for similar runs. Returns list of {id, text, metadata, score}."""
    if _vector_store is None:
        return []
    try:
        query = f"task={task_type}"
        if gripper_type:
            query += f" gripper={gripper_type}"
        return _vector_store.search("training_runs", query, n=n)
    except Exception:
        return []


def get_training_lineage(policy_id: str) -> List[Dict[str, Any]]:
    """Get lineage (parents) for policy from lineage graph (training runs it was derived from)."""
    if _lineage_graph is None:
        return []
    try:
        return _lineage_graph.get_lineage(policy_id)
    except Exception:
        return []


def record_scene_to_substrate(scene_id: str, mjcf_summary: str, metadata: Optional[Dict[str, Any]] = None) -> None:
    """Store scene in vector_store('scenes') for search."""
    if _vector_store is None:
        return
    try:
        _vector_store.add("scenes", mjcf_summary[:5000], metadata={"scene_id": scene_id, **(metadata or {})})
    except Exception:
        pass
