"""
TacTip-style Tactile Sensor Site Generator

Generates MJCF site definitions for mounting tactile sensors
on gripper fingertips and contact surfaces.
"""

from typing import List, Dict, Tuple
import math


def generate_fingertip_sites(
    finger_name: str,
    tip_position: Tuple[float, float, float],
    tip_radius: float,
    num_taxels: int = 9,
    pattern: str = "radial"
) -> List[Dict]:
    """
    Generate tactile sensor sites for a fingertip.

    Args:
        finger_name: Base name for sites (e.g., "finger_0")
        tip_position: (x, y, z) of fingertip center
        tip_radius: Radius of fingertip
        num_taxels: Number of sensing points
        pattern: "radial", "grid", or "hexagonal"

    Returns:
        List of site definitions for MJCF
    """
    sites = []
    x, y, z = tip_position

    if pattern == "radial":
        # Center taxel
        sites.append({
            "name": f"{finger_name}_tactile_center",
            "pos": f"{x} {y} {z}",
            "size": "0.001",
            "type": "sphere",
            "rgba": "1 0 0 0.5"
        })

        # Ring of taxels
        ring_taxels = num_taxels - 1
        ring_radius = tip_radius * 0.7
        for i in range(ring_taxels):
            angle = 2 * math.pi * i / ring_taxels
            tx = x + ring_radius * math.cos(angle)
            ty = y + ring_radius * math.sin(angle)
            sites.append({
                "name": f"{finger_name}_tactile_{i}",
                "pos": f"{tx:.4f} {ty:.4f} {z}",
                "size": "0.001",
                "type": "sphere",
                "rgba": "1 0 0 0.5"
            })

    elif pattern == "grid":
        # 3x3 grid
        spacing = tip_radius * 0.5
        idx = 0
        for row in range(-1, 2):
            for col in range(-1, 2):
                tx = x + col * spacing
                ty = y + row * spacing
                sites.append({
                    "name": f"{finger_name}_tactile_{idx}",
                    "pos": f"{tx:.4f} {ty:.4f} {z}",
                    "size": "0.001",
                    "type": "sphere",
                    "rgba": "1 0 0 0.5"
                })
                idx += 1

    return sites


def sites_to_mjcf(sites: List[Dict]) -> str:
    """Convert site definitions to MJCF XML string"""
    lines = []
    for site in sites:
        attrs = ' '.join(f'{k}="{v}"' for k, v in site.items())
        lines.append(f'      <site {attrs}/>')
    return '\n'.join(lines)


def generate_tactile_sensors_mjcf(sites: List[Dict]) -> str:
    """Generate MJCF sensor definitions for tactile sites"""
    lines = []
    for site in sites:
        name = site["name"]
        lines.append(f'    <touch name="{name}_touch" site="{name}"/>')
    return '\n'.join(lines)
