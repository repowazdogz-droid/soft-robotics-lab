#!/usr/bin/env python3
"""
Soft Robotics Workbench: Motion to Morphology
==============================================

Converts human motion capture data into soft robot designs with:
- Gripper morphology generation
- Uncertainty analysis
- Failure mode prediction
- Export to USD/MJCF/URDF

Usage:
    from motion_to_morphology import MotionToMorphology

    m2m = MotionToMorphology()
    design = m2m.from_gesture("pinch", aperture=0.03)
    export_design(design, "outputs/my_gripper")
"""

import sys
import json
import math
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Tuple
from enum import Enum

_repo_root = Path(__file__).resolve().parent.parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))


class GestureType(Enum):
    """Human gesture types that map to gripper morphologies."""
    PINCH = "pinch"           # Precision grip → 2-finger tendon gripper
    POWER = "power"           # Power grasp → multi-finger pneumatic
    WRAP = "wrap"             # Wrapping → continuum/tentacle
    HOOK = "hook"             # Hook grip → curved fingers
    LATERAL = "lateral"       # Lateral pinch → parallel jaw
    SQUEEZE = "squeeze"       # Squeeze → variable stiffness
    SPREAD = "spread"         # Spread fingers → adaptive gripper


class ActuatorType(Enum):
    """Soft actuator types."""
    TENDON = "tendon"
    PNEUMATIC = "pneumatic"
    HYDRAULIC = "hydraulic"
    DEA = "dielectric_elastomer"
    SMA = "shape_memory_alloy"
    JAMMING = "jamming"


class MaterialType(Enum):
    """Soft materials."""
    SILICONE_SOFT = "ecoflex_0030"      # Shore 00-30
    SILICONE_MEDIUM = "dragonskin_10"   # Shore 10A
    SILICONE_FIRM = "dragonskin_30"     # Shore 30A
    TPU = "tpu_95a"
    HYDROGEL = "pva_hydrogel"


@dataclass
class TendonRouting:
    """Tendon routing configuration."""
    num_tendons: int
    routing_pattern: str  # "parallel", "antagonist", "helical"
    anchor_points: List[Tuple[float, float, float]]
    max_tension_n: float = 20.0
    stiffness_n_per_m: float = 1000.0


@dataclass
class FingerDesign:
    """Design for a single finger."""
    length_mm: float
    width_mm: float
    thickness_mm: float
    num_segments: int
    taper_ratio: float  # tip/base ratio
    material: MaterialType
    actuator: ActuatorType
    stiffness_gradient: List[float]  # Per-segment stiffness multipliers
    tendon_routing: Optional[TendonRouting] = None


@dataclass
class GripperDesign:
    """Complete gripper design."""
    id: str
    name: str
    created_at: str

    # Source
    source_gesture: GestureType
    source_parameters: Dict

    # Geometry
    num_fingers: int
    finger_designs: List[FingerDesign]
    palm_radius_mm: float
    palm_thickness_mm: float

    # Actuation
    primary_actuator: ActuatorType
    actuation_pressure_kpa: Optional[float] = None
    actuation_voltage_v: Optional[float] = None

    # Performance estimates
    max_aperture_mm: float = 100.0
    max_force_n: float = 10.0
    grasp_speed_mm_per_s: float = 50.0

    # Uncertainty
    confidence: float = 0.5
    uncertainty_factors: List[str] = field(default_factory=list)

    # Failure modes
    failure_modes: List[Dict] = field(default_factory=list)

    # Metadata
    tags: List[str] = field(default_factory=list)
    notes: str = ""

    def to_dict(self) -> Dict:
        d = asdict(self)
        d['source_gesture'] = self.source_gesture.value
        d['primary_actuator'] = self.primary_actuator.value
        for i, f in enumerate(d['finger_designs']):
            d['finger_designs'][i]['material'] = self.finger_designs[i].material.value
            d['finger_designs'][i]['actuator'] = self.finger_designs[i].actuator.value
        return d

    def summary(self) -> str:
        fm_lines = chr(10).join(
            f'║    • {fm["mode"]}: {fm["probability"]:.0%}' for fm in self.failure_modes[:3]
        ) if self.failure_modes else "║    (none)"
        uf_lines = chr(10).join(f'║    • {uf}' for uf in self.uncertainty_factors[:3]) or "║    (none)"
        return f"""
╔══════════════════════════════════════════════════════════════════╗
║  GRIPPER DESIGN: {self.name}
╠══════════════════════════════════════════════════════════════════╣
║  ID: {self.id}
║  Source Gesture: {self.source_gesture.value}
║  Confidence: {self.confidence:.0%}
║
║  GEOMETRY:
║    Fingers: {self.num_fingers}
║    Finger Length: {self.finger_designs[0].length_mm:.1f}mm
║    Palm Radius: {self.palm_radius_mm:.1f}mm
║    Max Aperture: {self.max_aperture_mm:.1f}mm
║
║  ACTUATION:
║    Type: {self.primary_actuator.value}
║    Max Force: {self.max_force_n:.1f}N
║    Speed: {self.grasp_speed_mm_per_s:.1f}mm/s
║
║  MATERIALS:
║    Finger: {self.finger_designs[0].material.value}
║    Segments per finger: {self.finger_designs[0].num_segments}
║
║  FAILURE MODES:
{fm_lines}
║
║  UNCERTAINTY:
{uf_lines}
╚══════════════════════════════════════════════════════════════════╝
"""

    def export_all(self, output_dir: str) -> Dict[str, str]:
        """Export design to all formats (JSON, MJCF, URDF, USD)."""
        return export_design(self, output_dir)


class MotionToMorphology:
    """Converts motion/gesture data to gripper morphologies."""

    def __init__(self):
        self.design_counter = 0

        self.gesture_actuator_map = {
            GestureType.PINCH: ActuatorType.TENDON,
            GestureType.POWER: ActuatorType.PNEUMATIC,
            GestureType.WRAP: ActuatorType.PNEUMATIC,
            GestureType.HOOK: ActuatorType.TENDON,
            GestureType.LATERAL: ActuatorType.TENDON,
            GestureType.SQUEEZE: ActuatorType.JAMMING,
            GestureType.SPREAD: ActuatorType.PNEUMATIC,
        }

        self.gesture_fingers_map = {
            GestureType.PINCH: 2,
            GestureType.POWER: 4,
            GestureType.WRAP: 3,
            GestureType.HOOK: 3,
            GestureType.LATERAL: 2,
            GestureType.SQUEEZE: 4,
            GestureType.SPREAD: 5,
        }

    def from_gesture(
        self,
        gesture: str,
        aperture: float = 0.05,
        force_requirement: float = 5.0,
        object_compliance: float = 0.5,
        environment: str = "dry",
    ) -> GripperDesign:
        """
        Generate gripper design from gesture type.

        Args:
            gesture: Gesture type (pinch, power, wrap, etc.)
            aperture: Target grasp aperture in meters
            force_requirement: Required grasp force in Newtons
            object_compliance: Object softness 0=rigid, 1=very soft
            environment: Operating environment (dry, wet, surgical)
        """
        self.design_counter += 1

        gesture_type = GestureType(gesture.lower())

        actuator = self.gesture_actuator_map[gesture_type]
        num_fingers = self.gesture_fingers_map[gesture_type]

        if environment == "wet":
            pass
        elif environment == "surgical":
            aperture = min(aperture, 0.03)
            num_fingers = min(num_fingers, 3)

        finger_length = aperture * 1.5
        finger_width = finger_length * 0.15

        fingers = []
        for i in range(num_fingers):
            finger = self._generate_finger(
                length_mm=finger_length * 1000,
                width_mm=finger_width * 1000,
                actuator=actuator,
                object_compliance=object_compliance,
            )
            fingers.append(finger)

        failure_modes = self._predict_failure_modes(
            gesture_type, actuator, object_compliance, environment
        )

        confidence, uncertainty_factors = self._estimate_uncertainty(
            gesture_type, force_requirement, object_compliance, environment
        )

        design = GripperDesign(
            id=f"GD-{datetime.now().strftime('%Y%m%d')}-{self.design_counter:04d}",
            name=f"{gesture_type.value.title()} Gripper",
            created_at=datetime.now().isoformat(),
            source_gesture=gesture_type,
            source_parameters={
                "aperture": aperture,
                "force_requirement": force_requirement,
                "object_compliance": object_compliance,
                "environment": environment,
            },
            num_fingers=num_fingers,
            finger_designs=fingers,
            palm_radius_mm=aperture * 500,
            palm_thickness_mm=10.0,
            primary_actuator=actuator,
            max_aperture_mm=aperture * 1000,
            max_force_n=force_requirement * 1.5,
            grasp_speed_mm_per_s=50.0 if actuator == ActuatorType.TENDON else 30.0,
            confidence=confidence,
            uncertainty_factors=uncertainty_factors,
            failure_modes=failure_modes,
            tags=[gesture_type.value, actuator.value, environment],
        )

        return design

    def _generate_finger(
        self,
        length_mm: float,
        width_mm: float,
        actuator: ActuatorType,
        object_compliance: float,
    ) -> FingerDesign:
        """Generate a single finger design."""
        num_segments = (
            3 if object_compliance < 0.3 else 4 if object_compliance < 0.7 else 5
        )

        stiffness_gradient = [1.0 - (i * 0.15) for i in range(num_segments)]

        if object_compliance > 0.7:
            material = MaterialType.SILICONE_SOFT
        elif object_compliance > 0.3:
            material = MaterialType.SILICONE_MEDIUM
        else:
            material = MaterialType.SILICONE_FIRM

        tendon_routing = None
        if actuator == ActuatorType.TENDON:
            tendon_routing = TendonRouting(
                num_tendons=2,
                routing_pattern="antagonist",
                anchor_points=[(0, 0, 0), (0, 0, length_mm)],
                max_tension_n=20.0,
            )

        return FingerDesign(
            length_mm=length_mm,
            width_mm=width_mm,
            thickness_mm=width_mm * 0.6,
            num_segments=num_segments,
            taper_ratio=0.7,
            material=material,
            actuator=actuator,
            stiffness_gradient=stiffness_gradient,
            tendon_routing=tendon_routing,
        )

    def _predict_failure_modes(
        self,
        gesture: GestureType,
        actuator: ActuatorType,
        object_compliance: float,
        environment: str,
    ) -> List[Dict]:
        """Predict potential failure modes for this design."""
        modes = []

        modes.append({
            "mode": "Slip on smooth surfaces",
            "probability": 0.3 if environment == "dry" else 0.5,
            "mitigation": "Add textured surface or increase normal force",
        })

        if actuator == ActuatorType.TENDON:
            modes.append({
                "mode": "Tendon fatigue/breakage",
                "probability": 0.1,
                "mitigation": "Use braided cable, limit max tension",
            })
            modes.append({
                "mode": "Friction lock in routing",
                "probability": 0.15,
                "mitigation": "Use PTFE tubing, reduce routing angles",
            })

        elif actuator == ActuatorType.PNEUMATIC:
            modes.append({
                "mode": "Air leak at seal",
                "probability": 0.2,
                "mitigation": "Redundant seals, pressure monitoring",
            })
            modes.append({
                "mode": "Slow response time",
                "probability": 0.25,
                "mitigation": "Smaller chambers, higher flow valves",
            })

        if object_compliance > 0.7:
            modes.append({
                "mode": "Object deformation/damage",
                "probability": 0.35,
                "mitigation": "Force limiting, compliance sensing",
            })

        if environment == "wet":
            modes.append({
                "mode": "Reduced friction in wet conditions",
                "probability": 0.4,
                "mitigation": "Hydrophilic surface treatment",
            })

        if environment == "surgical":
            modes.append({
                "mode": "Sterilization degradation",
                "probability": 0.15,
                "mitigation": "Autoclave-compatible materials",
            })

        return modes

    def _estimate_uncertainty(
        self,
        gesture: GestureType,
        force: float,
        compliance: float,
        environment: str,
    ) -> Tuple[float, List[str]]:
        """Estimate design confidence and uncertainty sources."""
        confidence = 0.7
        factors = []

        if gesture in [GestureType.PINCH, GestureType.POWER]:
            confidence += 0.1
            factors.append("Well-studied grasp type (+10%)")
        else:
            factors.append("Less common grasp type - limited validation data")

        if force > 20:
            confidence -= 0.15
            factors.append("High force requirement increases actuator uncertainty (-15%)")

        if 0.3 <= compliance <= 0.7:
            confidence += 0.05
            factors.append("Mid-range compliance - good material match (+5%)")
        else:
            confidence -= 0.1
            factors.append("Extreme compliance - material selection uncertain (-10%)")

        if environment == "surgical":
            confidence -= 0.1
            factors.append("Surgical environment - regulatory uncertainty (-10%)")
        elif environment == "wet":
            confidence -= 0.05
            factors.append("Wet environment - friction modeling uncertain (-5%)")

        return max(0.2, min(0.95, confidence)), factors


class GripperExporter:
    """Exports gripper designs to various formats."""

    @staticmethod
    def to_json(design: GripperDesign, path: str) -> str:
        """Export to JSON."""
        data = design.to_dict()
        Path(path).write_text(json.dumps(data, indent=2))
        return path

    @staticmethod
    def to_mjcf(design: GripperDesign, path: str) -> str:
        """Export to MuJoCo MJCF format."""
        fingers_xml = ""
        angle_step = 360 / design.num_fingers

        for i, finger in enumerate(design.finger_designs):
            angle = math.radians(i * angle_step)
            x = design.palm_radius_mm / 1000 * math.cos(angle)
            y = design.palm_radius_mm / 1000 * math.sin(angle)

            finger_xml = f"""
        <body name="finger_{i}" pos="{x:.4f} {y:.4f} {design.palm_thickness_mm/1000:.4f}">
            <joint name="finger_{i}_base" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
            <geom type="capsule" size="{finger.width_mm/2000:.4f}" fromto="0 0 0 0 0 {finger.length_mm/1000:.4f}"/>
"""
            seg_length = finger.length_mm / (finger.num_segments * 1000)
            for j in range(1, finger.num_segments):
                taper = 1 - (j * (1 - finger.taper_ratio) / finger.num_segments)
                finger_xml += f"""
            <body name="finger_{i}_seg_{j}" pos="0 0 {seg_length:.4f}">
                <joint name="finger_{i}_joint_{j}" type="hinge" axis="0 1 0" range="-1.0 1.0"/>
                <geom type="capsule" size="{finger.width_mm/2000 * taper:.4f}" fromto="0 0 0 0 0 {seg_length:.4f}"/>
"""
            finger_xml += "            </body>\n" * (finger.num_segments - 1)
            finger_xml += "        </body>\n"

            fingers_xml += finger_xml

        mjcf = f"""<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{design.name.replace(' ', '_')}">
    <option timestep="0.002" gravity="0 0 -9.81"/>

    <default>
        <joint damping="0.5" armature="0.01"/>
        <geom condim="4" friction="0.8 0.005 0.0001" rgba="0.8 0.4 0.3 1"/>
    </default>

    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>

        <!-- Palm -->
        <body name="palm" pos="0 0 0.1">
            <joint name="palm_free" type="free"/>
            <geom type="cylinder" size="{design.palm_radius_mm/1000:.4f} {design.palm_thickness_mm/2000:.4f}" rgba="0.5 0.5 0.6 1"/>

            <!-- Fingers -->
            {fingers_xml}
        </body>
    </worldbody>

    <actuator>
        {"".join(f'<motor name="finger_{i}_motor" joint="finger_{i}_base" gear="1"/>' + chr(10) + '        ' for i in range(design.num_fingers))}
    </actuator>
</mujoco>
"""

        Path(path).write_text(mjcf)
        return path

    @staticmethod
    def to_urdf(design: GripperDesign, path: str) -> str:
        """Export to URDF format (ROS compatible)."""
        fingers_urdf = ""
        angle_step = 360 / design.num_fingers

        for i, finger in enumerate(design.finger_designs):
            angle = math.radians(i * angle_step)
            x = design.palm_radius_mm / 1000 * math.cos(angle)
            y = design.palm_radius_mm / 1000 * math.sin(angle)

            fingers_urdf += f"""
    <link name="finger_{i}_link">
        <visual>
            <geometry>
                <cylinder radius="{finger.width_mm/2000:.4f}" length="{finger.length_mm/1000:.4f}"/>
            </geometry>
            <origin xyz="0 0 {finger.length_mm/2000:.4f}"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="{finger.width_mm/2000:.4f}" length="{finger.length_mm/1000:.4f}"/>
            </geometry>
            <origin xyz="0 0 {finger.length_mm/2000:.4f}"/>
        </collision>
    </link>

    <joint name="finger_{i}_joint" type="revolute">
        <parent link="palm_link"/>
        <child link="finger_{i}_link"/>
        <origin xyz="{x:.4f} {y:.4f} {design.palm_thickness_mm/1000:.4f}" rpy="0 0 {angle:.4f}"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>
"""

        urdf = f"""<?xml version="1.0"?>
<robot name="{design.name.replace(' ', '_')}">

    <link name="palm_link">
        <visual>
            <geometry>
                <cylinder radius="{design.palm_radius_mm/1000:.4f}" length="{design.palm_thickness_mm/1000:.4f}"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="{design.palm_radius_mm/1000:.4f}" length="{design.palm_thickness_mm/1000:.4f}"/>
            </geometry>
        </collision>
    </link>

    {fingers_urdf}
</robot>
"""

        Path(path).write_text(urdf)
        return path

    @staticmethod
    def to_usd_json(design: GripperDesign, path: str) -> str:
        """Export to USD-compatible JSON (for Omniverse)."""
        usd_data = {
            "format": "omega_soft_robotics",
            "version": "1.0",
            "design": design.to_dict(),
            "stage": {
                "upAxis": "Z",
                "metersPerUnit": 0.001,
            },
            "prims": [],
        }

        usd_data["prims"].append({
            "path": "/gripper/palm",
            "type": "Cylinder",
            "radius": design.palm_radius_mm,
            "height": design.palm_thickness_mm,
            "material": "silicone",
        })

        angle_step = 360 / design.num_fingers
        for i, finger in enumerate(design.finger_designs):
            angle = i * angle_step
            usd_data["prims"].append({
                "path": f"/gripper/finger_{i}",
                "type": "Capsule",
                "radius": finger.width_mm / 2,
                "height": finger.length_mm,
                "rotation": [0, 0, angle],
                "segments": finger.num_segments,
                "material": finger.material.value,
            })

        Path(path).write_text(json.dumps(usd_data, indent=2))
        return path


def export_design(design: GripperDesign, output_dir: str) -> Dict[str, str]:
    """Export design to all formats."""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    base_name = design.id.lower().replace("-", "_")

    exports = {}
    exports["json"] = GripperExporter.to_json(design, str(output_path / f"{base_name}.json"))
    exports["mjcf"] = GripperExporter.to_mjcf(design, str(output_path / f"{base_name}.mjcf"))
    exports["urdf"] = GripperExporter.to_urdf(design, str(output_path / f"{base_name}.urdf"))
    exports["usd"] = GripperExporter.to_usd_json(design, str(output_path / f"{base_name}_usd.json"))

    return exports


# CLI
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Motion to Morphology")
    parser.add_argument(
        "gesture",
        help="Gesture type (pinch, power, wrap, hook, lateral, squeeze, spread)",
    )
    parser.add_argument("--aperture", "-a", type=float, default=0.05, help="Grasp aperture in meters")
    parser.add_argument("--force", "-f", type=float, default=5.0, help="Required force in Newtons")
    parser.add_argument("--compliance", "-c", type=float, default=0.5, help="Object compliance 0-1")
    parser.add_argument("--environment", "-e", default="dry", help="Environment (dry, wet, surgical)")
    parser.add_argument("--output", "-o", default="outputs", help="Output directory")

    args = parser.parse_args()

    m2m = MotionToMorphology()
    design = m2m.from_gesture(
        args.gesture,
        aperture=args.aperture,
        force_requirement=args.force,
        object_compliance=args.compliance,
        environment=args.environment,
    )

    print(design.summary())

    exports = export_design(design, args.output)
    print("\nExported to:")
    for fmt, path in exports.items():
        print(f"  {fmt}: {path}")
