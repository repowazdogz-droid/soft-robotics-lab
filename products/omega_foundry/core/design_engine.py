"""
Design engine - routes intent to domain generators, returns GeneratedDesign.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from .intent_parser import DesignSpec
from .primitives.grippers import GripperGenerator
from .primitives.mechanisms import MechanismGenerator
from .primitives.enclosures import EnclosureGenerator


@dataclass
class GeneratedDesign:
    """Design output: geometry + physics params + export artifacts."""
    id: str
    name: str
    domain: str
    spec: DesignSpec
    design_dict: Dict
    mjcf_xml: Optional[str] = None
    urdf_xml: Optional[str] = None
    mesh_path: Optional[str] = None
    errors: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "id": self.id,
            "name": self.name,
            "domain": self.domain,
            "spec": self.spec.to_dict(),
            "design_dict": self.design_dict,
            "errors": self.errors,
        }


class DesignEngine:
    """Route DesignSpec to domain generator and return GeneratedDesign."""

    def __init__(self):
        self.gripper_gen = GripperGenerator()
        self.mechanism_gen = MechanismGenerator()
        self.enclosure_gen = EnclosureGenerator()

    def generate(self, spec: DesignSpec) -> GeneratedDesign:
        """
        Generate design from spec. Routes by domain.

        Returns:
            GeneratedDesign with design_dict, mjcf_xml (or urdf_xml for enclosure), mesh_path where applicable.
        """
        errors: List[str] = []
        design_dict: Dict = {}
        mjcf_xml: Optional[str] = None
        urdf_xml: Optional[str] = None
        mesh_path: Optional[str] = None

        try:
            if spec.domain == "gripper":
                design_dict = self.gripper_gen.generate(spec)
                mjcf_xml = self.gripper_gen.generate_mjcf(design_dict)
            elif spec.domain == "mechanism":
                design_dict = self.mechanism_gen.generate(spec)
                mjcf_xml = self.mechanism_gen.generate_mjcf(design_dict)
            elif spec.domain == "enclosure":
                design_dict = self.enclosure_gen.generate(spec)
                mesh = self.enclosure_gen.generate_mesh(design_dict)
                if mesh is not None:
                    out_dir = Path("outputs")
                    out_dir.mkdir(parents=True, exist_ok=True)
                    stl_path = out_dir / f"{design_dict['id'].replace('-', '_')}.stl"
                    mesh.export(str(stl_path))
                    mesh_path = str(stl_path)
                urdf_xml = self.enclosure_gen.generate_urdf(
                    design_dict, mesh_path=mesh_path
                )
            else:
                design_dict = self.gripper_gen.generate(spec)
                mjcf_xml = self.gripper_gen.generate_mjcf(design_dict)
        except Exception as e:
            errors.append(str(e))

        return GeneratedDesign(
            id=design_dict.get("id", "unknown"),
            name=design_dict.get("name", "Unnamed"),
            domain=spec.domain,
            spec=spec,
            design_dict=design_dict,
            mjcf_xml=mjcf_xml,
            urdf_xml=urdf_xml,
            mesh_path=mesh_path,
            errors=errors,
        )
