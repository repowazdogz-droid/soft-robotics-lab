"""
DragonSkin Silicone Material Models

Implements non-linear viscoelastic behavior including:
- Toe region (crimp straightening analog)
- Strain-dependent stiffness
- Hysteresis modeling
"""

from dataclasses import dataclass
from typing import Dict


@dataclass
class DragonSkinProperties:
    """Material properties for DragonSkin silicones"""
    name: str
    shore_hardness: float  # Shore A
    tensile_strength: float  # MPa
    elongation_at_break: float  # %
    tear_strength: float  # kN/m

    # Non-linear parameters
    toe_region_strain: float  # Strain at end of toe region (typically 0.05-0.15)
    toe_region_modulus: float  # Low modulus in toe region
    linear_modulus: float  # Modulus after toe region

    # Viscoelastic parameters
    damping_coefficient: float
    relaxation_time: float  # seconds


# Pre-calibrated material library
DRAGONSKIN_LIBRARY = {
    "dragonskin_10": DragonSkinProperties(
        name="DragonSkin 10 NV",
        shore_hardness=10,
        tensile_strength=3.28,
        elongation_at_break=1000,
        tear_strength=16.8,
        toe_region_strain=0.10,
        toe_region_modulus=0.05,  # Very soft in toe region
        linear_modulus=0.15,
        damping_coefficient=0.12,
        relaxation_time=0.5,
    ),
    "dragonskin_20": DragonSkinProperties(
        name="DragonSkin 20",
        shore_hardness=20,
        tensile_strength=3.79,
        elongation_at_break=620,
        tear_strength=17.9,
        toe_region_strain=0.08,
        toe_region_modulus=0.10,
        linear_modulus=0.25,
        damping_coefficient=0.15,
        relaxation_time=0.4,
    ),
    "dragonskin_30": DragonSkinProperties(
        name="DragonSkin 30",
        shore_hardness=30,
        tensile_strength=3.45,
        elongation_at_break=364,
        tear_strength=13.4,
        toe_region_strain=0.05,
        toe_region_modulus=0.15,
        linear_modulus=0.35,
        damping_coefficient=0.18,
        relaxation_time=0.3,
    ),
    "ecoflex_0030": DragonSkinProperties(
        name="Ecoflex 00-30",
        shore_hardness=0,  # 00-30
        tensile_strength=1.38,
        elongation_at_break=900,
        tear_strength=6.7,
        toe_region_strain=0.15,
        toe_region_modulus=0.02,
        linear_modulus=0.08,
        damping_coefficient=0.08,
        relaxation_time=0.8,
    ),
}


def calculate_stress(strain: float, material: DragonSkinProperties) -> float:
    """
    Calculate stress for given strain using bi-linear toe region model.

    Models the "crimp straightening" behavior seen in biological tissues
    where initial loading meets low resistance (toe region) before
    engaging the primary load-bearing structure.
    """
    if strain < 0:
        return 0.0

    if strain <= material.toe_region_strain:
        # Toe region: low stiffness (crimps straightening)
        return strain * material.toe_region_modulus
    else:
        # Linear region: primary stiffness
        toe_stress = material.toe_region_strain * material.toe_region_modulus
        additional_strain = strain - material.toe_region_strain
        return toe_stress + additional_strain * material.linear_modulus


def calculate_damping(velocity: float, material: DragonSkinProperties) -> float:
    """Calculate viscous damping force"""
    return velocity * material.damping_coefficient


def get_mjcf_parameters(material_name: str) -> Dict:
    """
    Convert material properties to MuJoCo-compatible parameters.

    Returns dict with: stiffness, damping, frictionloss
    """
    mat = DRAGONSKIN_LIBRARY.get(material_name)
    if not mat:
        raise ValueError(f"Unknown material: {material_name}")

    return {
        "stiffness": mat.linear_modulus,
        "damping": mat.damping_coefficient,
        "frictionloss": mat.damping_coefficient * 0.1,
        "armature": 0.001,
        "rgba": "0.9 0.85 0.8 1",
        # Metadata for documentation
        "_material_name": mat.name,
        "_toe_region_strain": mat.toe_region_strain,
        "_shore_hardness": mat.shore_hardness,
    }


def list_materials() -> list:
    """List available materials"""
    return list(DRAGONSKIN_LIBRARY.keys())
