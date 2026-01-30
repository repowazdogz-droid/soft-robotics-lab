"""Soft robotics material models (DragonSkin, Ecoflex, etc.)."""
from .dragonskin import (
    DragonSkinProperties,
    DRAGONSKIN_LIBRARY,
    calculate_stress,
    calculate_damping,
    get_mjcf_parameters,
    list_materials,
)

__all__ = [
    "DragonSkinProperties",
    "DRAGONSKIN_LIBRARY",
    "calculate_stress",
    "calculate_damping",
    "get_mjcf_parameters",
    "list_materials",
]
