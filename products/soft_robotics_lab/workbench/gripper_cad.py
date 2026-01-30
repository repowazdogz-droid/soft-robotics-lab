#!/usr/bin/env python3
"""
Gripper CAD Generator
=====================

Generate real 3D geometry for soft grippers using trimesh.
Exports to STL for 3D printing or simulation.
"""

import trimesh
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class FingerGeometry:
    """Parameters for a single finger."""
    length_mm: float
    base_radius_mm: float
    tip_radius_mm: float  # For taper
    num_segments: int
    segment_gap_mm: float = 1.0  # Gap between segments for flexibility


@dataclass
class GripperGeometry:
    """Complete gripper geometry parameters."""
    num_fingers: int
    finger: FingerGeometry
    palm_radius_mm: float
    palm_thickness_mm: float
    finger_spread_angle: float = 30.0  # Degrees from vertical


def create_tapered_cylinder(
    base_radius: float,
    tip_radius: float,
    height: float,
    sections: int = 32
) -> trimesh.Trimesh:
    """Create a tapered cylinder (cone frustum)."""

    # Create vertices for top and bottom circles
    angles = np.linspace(0, 2 * np.pi, sections, endpoint=False)

    # Bottom circle (base)
    bottom_verts = np.column_stack([
        base_radius * np.cos(angles),
        base_radius * np.sin(angles),
        np.zeros(sections)
    ])

    # Top circle (tip)
    top_verts = np.column_stack([
        tip_radius * np.cos(angles),
        tip_radius * np.sin(angles),
        np.ones(sections) * height
    ])

    # Center vertices for caps
    bottom_center = np.array([[0, 0, 0]])
    top_center = np.array([[0, 0, height]])

    # Combine vertices
    vertices = np.vstack([bottom_verts, top_verts, bottom_center, top_center])

    # Bottom center index: sections * 2
    # Top center index: sections * 2 + 1
    bc_idx = sections * 2
    tc_idx = sections * 2 + 1

    faces = []

    # Side faces (quads split into triangles)
    for i in range(sections):
        next_i = (i + 1) % sections

        # Two triangles per quad
        faces.append([i, next_i, sections + next_i])
        faces.append([i, sections + next_i, sections + i])

    # Bottom cap
    for i in range(sections):
        next_i = (i + 1) % sections
        faces.append([bc_idx, next_i, i])

    # Top cap
    for i in range(sections):
        next_i = (i + 1) % sections
        faces.append([tc_idx, sections + i, sections + next_i])

    faces = np.array(faces)

    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    mesh.fix_normals()

    return mesh


def create_sphere(radius: float, center: Tuple[float, float, float] = (0, 0, 0)) -> trimesh.Trimesh:
    """Create a sphere at given center."""
    sphere = trimesh.creation.icosphere(subdivisions=2, radius=radius)
    sphere.apply_translation(center)
    return sphere


def create_finger(
    geometry: FingerGeometry,
    base_position: Tuple[float, float, float] = (0, 0, 0),
    direction: Tuple[float, float, float] = (0, 0, 1),
    bend_angle: float = 0.0
) -> trimesh.Trimesh:
    """
    Create a segmented finger with taper.

    Args:
        geometry: Finger parameters
        base_position: Where finger attaches to palm
        direction: Initial direction of finger
        bend_angle: Total bend angle in degrees (distributed across segments)

    Returns:
        Combined mesh for the finger
    """
    meshes = []

    segment_length = geometry.length_mm / geometry.num_segments
    bend_per_segment = np.radians(bend_angle) / geometry.num_segments

    # Calculate radius taper per segment
    radius_step = (geometry.base_radius_mm - geometry.tip_radius_mm) / geometry.num_segments

    current_pos = np.array(base_position, dtype=float)
    current_dir = np.array(direction, dtype=float)
    current_dir = current_dir / np.linalg.norm(current_dir)

    for seg in range(geometry.num_segments):
        # Current segment radii
        seg_base_radius = geometry.base_radius_mm - seg * radius_step
        seg_tip_radius = geometry.base_radius_mm - (seg + 1) * radius_step

        # Create segment
        segment = create_tapered_cylinder(
            base_radius=seg_base_radius,
            tip_radius=seg_tip_radius,
            height=segment_length - geometry.segment_gap_mm
        )

        # Calculate rotation to align with current direction
        # Default cylinder is along Z axis
        z_axis = np.array([0, 0, 1])

        if not np.allclose(current_dir, z_axis):
            rotation_axis = np.cross(z_axis, current_dir)
            if np.linalg.norm(rotation_axis) > 1e-6:
                rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                angle = np.arccos(np.clip(np.dot(z_axis, current_dir), -1, 1))
                rot_4x4 = trimesh.transformations.rotation_matrix(angle, rotation_axis)
                rot_3x3 = rot_4x4[:3, :3]
                euler = trimesh.transformations.euler_from_matrix(rot_4x4)
                transform = trimesh.transformations.compose_matrix(
                    translate=current_pos, angles=euler
                )
                segment.apply_transform(transform)
            else:
                segment.apply_translation(current_pos)
        else:
            segment.apply_translation(current_pos)

        meshes.append(segment)

        # Add joint sphere between segments
        if seg < geometry.num_segments - 1:
            joint_pos = current_pos + current_dir * (segment_length - geometry.segment_gap_mm / 2)
            joint = create_sphere(seg_tip_radius * 0.9, tuple(joint_pos))
            meshes.append(joint)

        # Update position for next segment
        current_pos = current_pos + current_dir * segment_length

        # Apply bend for next segment (rotate direction outward)
        if bend_per_segment > 0:
            # Bend away from center (outward)
            bend_axis = np.cross(current_dir, np.array([0, 0, 1]))
            if np.linalg.norm(bend_axis) < 1e-6:
                bend_axis = np.array([1, 0, 0])
            bend_axis = bend_axis / np.linalg.norm(bend_axis)

            rot = trimesh.transformations.rotation_matrix(bend_per_segment, bend_axis)[:3, :3]
            current_dir = rot @ current_dir

    # Combine all segment meshes
    combined = trimesh.util.concatenate(meshes)
    return combined


def create_palm(
    radius: float,
    thickness: float,
    num_fingers: int,
    finger_base_radius: float
) -> trimesh.Trimesh:
    """
    Create palm with finger mounting points.
    """
    # Main palm cylinder
    palm = trimesh.creation.cylinder(radius=radius, height=thickness, sections=64)

    # Move so bottom is at z=0
    palm.apply_translation([0, 0, thickness / 2])

    return palm


def create_gripper(
    geometry: GripperGeometry,
    bend_angle: float = 15.0
) -> trimesh.Trimesh:
    """
    Create complete gripper geometry.

    Args:
        geometry: Gripper parameters
        bend_angle: How much fingers bend outward (degrees)

    Returns:
        Combined mesh for entire gripper
    """
    meshes = []

    # Create palm
    palm = create_palm(
        radius=geometry.palm_radius_mm,
        thickness=geometry.palm_thickness_mm,
        num_fingers=geometry.num_fingers,
        finger_base_radius=geometry.finger.base_radius_mm
    )
    meshes.append(palm)

    # Create fingers around palm
    finger_angle_step = 2 * np.pi / geometry.num_fingers
    finger_offset_radius = geometry.palm_radius_mm * 0.7

    for i in range(geometry.num_fingers):
        angle = i * finger_angle_step

        # Finger base position on top of palm
        base_x = finger_offset_radius * np.cos(angle)
        base_y = finger_offset_radius * np.sin(angle)
        base_z = geometry.palm_thickness_mm

        # Finger direction (angled outward)
        spread_rad = np.radians(geometry.finger_spread_angle)
        dir_x = np.sin(spread_rad) * np.cos(angle)
        dir_y = np.sin(spread_rad) * np.sin(angle)
        dir_z = np.cos(spread_rad)

        finger = create_finger(
            geometry=geometry.finger,
            base_position=(base_x, base_y, base_z),
            direction=(dir_x, dir_y, dir_z),
            bend_angle=bend_angle
        )
        meshes.append(finger)

    # Combine all meshes
    gripper = trimesh.util.concatenate(meshes)

    return gripper


def gripper_from_design(design: Dict) -> trimesh.Trimesh:
    """
    Create gripper geometry from a design dictionary (from motion_to_morphology).
    """
    # Extract parameters
    num_fingers = design.get('num_fingers', 3)
    finger_data = design.get('finger_designs', [{}])[0]

    finger_length = finger_data.get('length_mm', 50)
    finger_width = finger_data.get('width_mm', 12)
    num_segments = finger_data.get('num_segments', 4)
    taper_ratio = finger_data.get('taper_ratio', 0.6)

    palm_radius = design.get('palm_radius_mm', 25)
    palm_thickness = design.get('palm_thickness_mm', 10)

    finger_geom = FingerGeometry(
        length_mm=finger_length,
        base_radius_mm=finger_width / 2,
        tip_radius_mm=(finger_width / 2) * taper_ratio,
        num_segments=num_segments,
        segment_gap_mm=1.5
    )

    gripper_geom = GripperGeometry(
        num_fingers=num_fingers,
        finger=finger_geom,
        palm_radius_mm=palm_radius,
        palm_thickness_mm=palm_thickness,
        finger_spread_angle=25.0
    )

    return create_gripper(gripper_geom)


def export_gripper_stl(design: Dict, output_path: str) -> str:
    """
    Generate and export gripper STL from design.

    Returns:
        Path to exported STL file
    """
    mesh = gripper_from_design(design)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    mesh.export(str(output_path))

    return str(output_path)


# ════════════════════════════════════════════════════════════════════════════════
# CLI
# ════════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Generate gripper STL")
    parser.add_argument("design_json", nargs="?", help="Path to design JSON file")
    parser.add_argument("--output", "-o", default="gripper.stl", help="Output STL path")
    parser.add_argument("--demo", action="store_true", help="Generate demo gripper")

    args = parser.parse_args()

    if args.demo:
        # Demo gripper
        finger = FingerGeometry(
            length_mm=60,
            base_radius_mm=6,
            tip_radius_mm=3,
            num_segments=4
        )

        geom = GripperGeometry(
            num_fingers=4,
            finger=finger,
            palm_radius_mm=25,
            palm_thickness_mm=12
        )

        mesh = create_gripper(geom)
        mesh.export(args.output)
        print(f"✓ Exported demo gripper to {args.output}")
        print(f"  Vertices: {len(mesh.vertices)}")
        print(f"  Faces: {len(mesh.faces)}")

    elif args.design_json:
        with open(args.design_json) as f:
            design = json.load(f)

        path = export_gripper_stl(design, args.output)
        print(f"✓ Exported gripper to {path}")

    else:
        parser.print_help()
