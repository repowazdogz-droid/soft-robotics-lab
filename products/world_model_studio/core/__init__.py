"""
World Model Studio - Environment builder for scenes and manipulation agents.
"""

from .asset_library import list_assets, load_asset, import_from_foundry
from .scene_composer import SceneComposer
from .task_builder import TaskDefinition, TaskBuilder, PICK, PLACE, HANDOVER, MOVE_TO
from .simulator import Simulator
from .trainer import random_policy, run_episode, train_random_search
try:
    from .reality_gap import (
        RealityGapTracker,
        SimMetrics,
        RealMetrics,
        GapMetrics,
        GapRecord,
        compute_gap,
        generate_calibration_recommendations,
    )
except ImportError:
    RealityGapTracker = None
    SimMetrics = None
    RealMetrics = None
    GapMetrics = None
    GapRecord = None
    compute_gap = None
    generate_calibration_recommendations = None

__all__ = [
    "list_assets",
    "load_asset",
    "import_from_foundry",
    "SceneComposer",
    "TaskDefinition",
    "TaskBuilder",
    "PICK",
    "PLACE",
    "HANDOVER",
    "MOVE_TO",
    "Simulator",
    "random_policy",
    "run_episode",
    "train_random_search",
    "RealityGapTracker",
    "SimMetrics",
    "RealMetrics",
    "GapMetrics",
    "GapRecord",
    "compute_gap",
    "generate_calibration_recommendations",
]
