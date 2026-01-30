"""
Gripper generator - design dict + physics-correct MJCF.
Cutkosky grasp taxonomy: PINCH, POWER, HOOK, LATERAL, SPHERICAL, TRIPOD.
"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from ..intent_parser import DesignSpec


# Cutkosky grasp taxonomy: default fingers, spread_deg, curl_bias, tip_friction, stiffness/damping profile
GESTURE_PROFILES = {
    "pinch": {
        "num_fingers": 2,
        "spread_angle_deg": 30,
        "curl_bias": 0.0,
        "tip_friction": 0.9,
        "stiffness_profile": [1.0, 0.85, 0.7, 0.55],
        "damping_profile": [1.0, 0.8, 0.6, 0.5],
        "aperture_scale": 0.8,
    },
    "power": {
        "num_fingers": 4,
        "spread_angle_deg": 25,
        "curl_bias": 0.3,
        "tip_friction": 1.0,
        "stiffness_profile": [0.9, 0.75, 0.6, 0.5],
        "damping_profile": [0.9, 0.7, 0.55, 0.45],
        "aperture_scale": 1.2,
    },
    "hook": {
        "num_fingers": 3,
        "spread_angle_deg": 20,
        "curl_bias": 0.6,
        "tip_friction": 0.85,
        "stiffness_profile": [0.95, 0.8, 0.65, 0.5],
        "damping_profile": [0.95, 0.75, 0.6, 0.5],
        "aperture_scale": 1.0,
    },
    "lateral": {
        "num_fingers": 2,
        "spread_angle_deg": 15,
        "curl_bias": 0.0,
        "tip_friction": 0.95,
        "stiffness_profile": [1.0, 0.88, 0.75, 0.6],
        "damping_profile": [1.0, 0.82, 0.65, 0.55],
        "aperture_scale": 0.7,
    },
    "spherical": {
        "num_fingers": 5,
        "spread_angle_deg": 35,
        "curl_bias": 0.4,
        "tip_friction": 0.9,
        "stiffness_profile": [0.85, 0.7, 0.58, 0.48],
        "damping_profile": [0.88, 0.7, 0.55, 0.45],
        "aperture_scale": 1.3,
    },
    "tripod": {
        "num_fingers": 3,
        "spread_angle_deg": 120.0 / 2,  # 60 deg half-angle for 120° between fingers
        "curl_bias": 0.1,
        "tip_friction": 0.92,
        "stiffness_profile": [1.0, 0.86, 0.72, 0.58],
        "damping_profile": [1.0, 0.8, 0.64, 0.52],
        "aperture_scale": 0.85,
    },
}

# Material selection by force / object: shore, max_force_n, stiffness_base
MATERIAL_PROFILES = {
    "dragonskin_10": {"shore": 10, "max_force_n": 5, "stiffness_base": 0.02},
    "dragonskin_30": {"shore": 30, "max_force_n": 15, "stiffness_base": 0.05},
    "dragonskin_50": {"shore": 50, "max_force_n": 30, "stiffness_base": 0.10},
    "ecoflex_30": {"shore": 30, "max_force_n": 10, "stiffness_base": 0.04},
}


@dataclass
class GripperGenerator:
    """Generate gripper design dict and MJCF from DesignSpec."""

    design_counter: int = 0

    SCALE_MM = {"small": 0.7, "medium": 1.0, "large": 1.4}
    MATERIAL_MAP = {
        "silicone": "dragonskin_10",
        "tpu": "tpu_95a",
        "rigid": "tpu_95a",
        None: "dragonskin_10",
    }
    FORCE_TO_MAX_N = {"gentle": 2.0, "medium": 5.0, "strong": 12.0}

    def generate(self, spec: DesignSpec) -> Dict:
        gesture = (spec.params.get("gesture") or "pinch").lower()
        profile = GESTURE_PROFILES.get(gesture, GESTURE_PROFILES["pinch"])

        self.design_counter += 1
        scale_factor = self.SCALE_MM.get(spec.scale, 1.0)
        num_fingers = spec.params.get("num_fingers") or profile["num_fingers"]
        num_fingers = max(2, min(5, int(num_fingers)))

        aperture_scale = profile["aperture_scale"]
        aperture_m = 0.04 * scale_factor * aperture_scale
        if spec.target_object and "egg" in (spec.target_object or ""):
            aperture_m = 0.03
        if spec.target_object and "apple" in (spec.target_object or ""):
            aperture_m = 0.06

        finger_length_mm = aperture_m * 1.5 * 1000 * scale_factor
        finger_width_mm = finger_length_mm * 0.2
        num_segments = 4
        stiffness_gradient = profile["stiffness_profile"][:num_segments]
        damping_gradient = profile["damping_profile"][:num_segments]
        while len(stiffness_gradient) < num_segments:
            stiffness_gradient.append(stiffness_gradient[-1] * 0.85)
        while len(damping_gradient) < num_segments:
            damping_gradient.append(damping_gradient[-1] * 0.85)

        # Material selection: delicate/egg/electronics -> dragonskin_10; force > 10N -> dragonskin_50; heavy or power -> dragonskin_30 min; default dragonskin_10
        target_lower = (spec.target_object or "").lower()
        force_level = spec.params.get("force_requirement") or "medium"
        force_value = spec.params.get("force_requirement_n")
        if force_value is None:
            force_value = self.FORCE_TO_MAX_N.get(force_level, 5.0)
        force_value = float(force_value)

        if "delicate" in target_lower or "egg" in target_lower or "electronics" in target_lower:
            material = "dragonskin_10"
        elif force_value > 10:
            material = "dragonskin_50"
        elif "heavy" in target_lower or gesture == "power":
            material = "dragonskin_30"
        else:
            material = "dragonskin_10"
        material = self.MATERIAL_MAP.get(spec.material, material) if spec.material else material

        # Scale max_force_n: power+heavy=20, power+normal=10, pinch+delicate=2; else material default
        if gesture == "power" and "heavy" in target_lower:
            max_force_n = 20.0
        elif gesture == "power":
            max_force_n = 10.0
        elif gesture == "pinch" and ("delicate" in target_lower or "egg" in target_lower or "electronics" in target_lower):
            max_force_n = 2.0
        else:
            max_force_n = MATERIAL_PROFILES.get(material, {}).get("max_force_n", 5.0)

        # Dimensions override from params
        dims = spec.params.get("dimensions_mm") or {}
        if dims.get("length_mm"):
            finger_length_mm = dims["length_mm"]
        env = spec.params.get("environment") or "dry"

        finger_design = {
            "length_mm": finger_length_mm,
            "width_mm": finger_width_mm,
            "thickness_mm": finger_width_mm * 0.6,
            "num_segments": num_segments,
            "taper_ratio": 0.6 - 0.1 * profile["curl_bias"],
            "material": material,
            "actuator": "tendon",
            "stiffness_gradient": stiffness_gradient,
            "damping_gradient": damping_gradient,
            "tip_friction": profile["tip_friction"],
            "curl_bias": profile["curl_bias"],
        }

        design_id = f"OF-{datetime.now().strftime('%Y%m%d')}-{self.design_counter:04d}"
        name = f"{gesture.title()} Gripper"
        if spec.target_object:
            name += f" for {spec.target_object}"

        design = {
            "id": design_id,
            "name": name,
            "created_at": datetime.now().isoformat(),
            "source_gesture": gesture,
            "source_parameters": {
                "aperture": aperture_m,
                "force_requirement": max_force_n,
                "object_compliance": 0.3,
                "environment": env,
            },
            "num_fingers": num_fingers,
            "finger_designs": [finger_design],
            "palm_radius_mm": aperture_m * 500 * scale_factor,
            "palm_thickness_mm": 10.0,
            "primary_actuator": "tendon",
            "max_force_n": max_force_n,
            "max_aperture_mm": aperture_m * 1000,
            "spread_angle_deg": profile["spread_angle_deg"],
            "curl_bias": profile["curl_bias"],
            "tip_friction": profile["tip_friction"],
            "source_intent": spec.raw_intent,
        }
        return design

    def generate_mjcf(self, design: Dict) -> str:
        return _build_mjcf(design)


def _build_mjcf(design: Dict) -> str:
    finger_list = design.get("finger_designs") or [{}]
    finger_data = finger_list[0] if finger_list else {}
    num_fingers = design.get("num_fingers", 3)
    finger_length_m = finger_data.get("length_mm", 50) / 1000
    finger_width_m = finger_data.get("width_mm", 12) / 1000
    num_segments = finger_data.get("num_segments", 4)
    taper_ratio = finger_data.get("taper_ratio", 0.6)
    palm_radius_m = design.get("palm_radius_mm", 25) / 1000
    palm_thickness_m = design.get("palm_thickness_mm", 10) / 1000
    spread_angle_deg = design.get("spread_angle_deg", 30)
    tip_friction = design.get("tip_friction", 0.9)

    stiffness_gradient = finger_data.get("stiffness_gradient") or [
        1.0 - (i * 0.15) for i in range(num_segments)
    ]
    damping_gradient = finger_data.get("damping_gradient") or [
        1.0 - (i * 0.2) for i in range(num_segments)
    ]
    material_name = (finger_data.get("material") or "dragonskin_10").lower()
    base_props = {
        "dragonskin_10": {"stiffness": 0.02, "damping": 0.15, "rgba": "0.9 0.85 0.8 1"},
        "dragonskin_30": {"stiffness": 0.05, "damping": 0.12, "rgba": "0.85 0.8 0.75 1"},
        "dragonskin_50": {"stiffness": 0.10, "damping": 0.10, "rgba": "0.8 0.75 0.7 1"},
        "ecoflex_30": {"stiffness": 0.04, "damping": 0.18, "rgba": "0.95 0.9 0.85 1"},
        "tpu_95a": {"stiffness": 0.15, "damping": 0.08, "rgba": "0.7 0.7 0.75 1"},
    }
    mat_props = base_props.get(material_name, {"stiffness": 0.02, "damping": 0.15, "rgba": "0.9 0.85 0.8 1"})
    if material_name in MATERIAL_PROFILES:
        mat_props = {**mat_props, "stiffness": MATERIAL_PROFILES[material_name]["stiffness_base"]}

    friction = (tip_friction, 0.005, 0.0001)
    segment_length = finger_length_m / num_segments
    base_radius = finger_width_m / 2
    seg_end_z = segment_length * 0.95
    gear_ratio = min(50, max(10, design.get("max_force_n", 5.0) * 6))

    xml = [
        f'<?xml version="1.0" encoding="utf-8"?>',
        f'<mujoco model="{design.get("id", "gripper")}">',
        "  <compiler angle=\"radian\" autolimits=\"true\" meshdir=\"meshes\"/>",
        "  <option timestep=\"0.002\" iterations=\"50\" solver=\"Newton\" tolerance=\"1e-10\">",
        "    <flag warmstart=\"enable\"/>",
        "  </option>",
        "  <default>",
        f'    <default class="gripper">',
        f'      <geom type="capsule" condim="4" friction="{friction[0]} {friction[1]} {friction[2]}" rgba="{mat_props["rgba"]}" margin="0.001"/>',
        f'      <joint type="hinge" limited="true" range="-1.57 1.57" stiffness="{mat_props["stiffness"]}" damping="{mat_props["damping"]}" armature="0.001"/>',
        "    </default>",
        f'    <default class="palm"><geom type="cylinder" rgba="0.4 0.4 0.5 1" friction="{friction[0]} {friction[1]} {friction[2]}"/>',
        "    </default>",
        "  </default>",
        "  <worldbody>",
        "    <geom name=\"ground\" type=\"plane\" size=\"0.5 0.5 0.1\" condim=\"3\"/>",
        "    <light pos=\"0 0 1.5\" dir=\"0 0 -1\" directional=\"true\"/>",
        f"    <body name=\"palm\" pos=\"0 0 0.15\">",
        f'      <geom class="palm" name="palm_geom" size="{palm_radius_m:.4f} {palm_thickness_m/2:.4f}" pos="0 0 {palm_thickness_m/2:.4f}" />',
    ]

    tendon_sites: Dict[int, List[str]] = {i: [] for i in range(num_fingers)}
    palm_z = 0.15
    spread_rad = np.deg2rad(spread_angle_deg)

    for f_idx in range(num_fingers):
        angle = 2 * np.pi * f_idx / num_fingers
        bx = palm_radius_m * 0.65 * np.cos(angle)
        by = palm_radius_m * 0.65 * np.sin(angle)
        bz = palm_thickness_m
        sg0 = stiffness_gradient[min(0, len(stiffness_gradient) - 1)]
        dg0 = damping_gradient[min(0, len(damping_gradient) - 1)]
        xml.append(
            f'      <body name="finger_{f_idx}_base" pos="{bx:.4f} {by:.4f} {bz:.4f}">'
        )
        xml.append(
            f'        <joint class="gripper" name="finger_{f_idx}_base_joint" axis="0 1 0" stiffness="{mat_props["stiffness"] * sg0:.4f}" damping="{mat_props["damping"] * dg0:.4f}"/>'
        )
        xml.append(f'        <site name="finger_{f_idx}_site_0" pos="0 0 0.002" size="0.002"/>')
        tendon_sites[f_idx].append(f"finger_{f_idx}_site_0")

        seg_start = 0.002
        for s_idx in range(num_segments):
            taper_f = 1.0 - (s_idx * (1.0 - taper_ratio) / max(num_segments, 1))
            seg_radius = base_radius * taper_f * 0.9
            sg = stiffness_gradient[min(s_idx, len(stiffness_gradient) - 1)]
            dg = damping_gradient[min(s_idx, len(damping_gradient) - 1)]
            sk = mat_props["stiffness"] * sg
            dk = mat_props["damping"] * dg
            if s_idx == 0:
                xml.append(
                    f'        <geom class="gripper" name="finger_{f_idx}_seg_0" fromto="0 0 {seg_start:.4f} 0 0 {seg_end_z:.4f}" size="{seg_radius:.4f}" />'
                )
            else:
                xml.append(
                    f'        <body name="finger_{f_idx}_seg_{s_idx}" pos="0 0 {seg_end_z:.4f}">'
                )
                xml.append(
                    f'          <joint class="gripper" name="finger_{f_idx}_joint_{s_idx}" axis="0 1 0" stiffness="{sk:.4f}" damping="{dk:.4f}"/>'
                )
                xml.append(
                    f'          <geom class="gripper" name="finger_{f_idx}_seg_{s_idx}_geom" fromto="0 0 0.001 0 0 {seg_end_z:.4f}" size="{seg_radius:.4f}" />'
                )
                xml.append(
                    f'          <site name="finger_{f_idx}_site_{s_idx}" pos="0 0 {seg_end_z/2:.4f}" size="0.002"/>'
                )
                tendon_sites[f_idx].append(f"finger_{f_idx}_site_{s_idx}")
                xml.append("        </body>")
        xml.append("      </body>")

    xml.append("    </body>")
    xml.append("  </worldbody>")

    xml.append("  <tendon>")
    for f_idx in range(num_fingers):
        sites = tendon_sites[f_idx]
        xml.append(f'    <spatial name="finger_{f_idx}_flexor" limited="true" range="0 0.15" width="0.002" frictionloss="0.1">')
        for site in sites:
            xml.append(f'      <site site="{site}"/>')
        xml.append("    </spatial>")
        xml.append(f'    <spatial name="finger_{f_idx}_extensor" limited="true" range="0 0.15" width="0.002" frictionloss="0.1">')
        for site in reversed(sites):
            xml.append(f'      <site site="{site}"/>')
        xml.append("    </spatial>")
    xml.append("  </tendon>")

    xml.append("  <actuator>")
    for f_idx in range(num_fingers):
        xml.append(f'    <motor name="finger_{f_idx}_flexor_motor" tendon="finger_{f_idx}_flexor" gear="{gear_ratio:.0f}" ctrllimited="true" ctrlrange="0 1"/>')
        xml.append(f'    <motor name="finger_{f_idx}_extensor_motor" tendon="finger_{f_idx}_extensor" gear="{gear_ratio*0.5:.0f}" ctrllimited="true" ctrlrange="0 0.5"/>')
    xml.append("  </actuator>")

    xml.append("  <sensor>")
    for f_idx in range(num_fingers):
        xml.append(f'    <jointpos name="finger_{f_idx}_pos" joint="finger_{f_idx}_base_joint"/>')
        xml.append(f'    <jointvel name="finger_{f_idx}_vel" joint="finger_{f_idx}_base_joint"/>')
        xml.append(f'    <tendonpos name="finger_{f_idx}_flexor_len" tendon="finger_{f_idx}_flexor"/>')
    xml.append("  </sensor>")
    xml.append("</mujoco>")
    return "\n".join(xml)
