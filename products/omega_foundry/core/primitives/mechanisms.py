"""
Mechanism generator - hinge, four-bar, cam, slider, gear pair, universal joint.
Full MJCF with joint limits, damping, actuators.
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Dict

from ..intent_parser import DesignSpec


@dataclass
class MechanismGenerator:
    """Generate mechanism design dict and MJCF from DesignSpec."""

    design_counter: int = 0

    def generate(self, spec: DesignSpec) -> Dict:
        self.design_counter += 1
        mech_type = spec.params.get("type", "hinge")
        scale = spec.scale
        scale_m = {"small": 0.02, "medium": 0.05, "large": 0.1}.get(scale, 0.05)

        design_id = f"OF-M-{datetime.now().strftime('%Y%m%d')}-{self.design_counter:04d}"
        return {
            "id": design_id,
            "name": f"{mech_type.replace('_', ' ').title()} Mechanism",
            "domain": "mechanism",
            "type": mech_type,
            "scale_m": scale_m,
            "created_at": datetime.now().isoformat(),
            "source_intent": spec.raw_intent,
        }

    def generate_mjcf(self, design: Dict) -> str:
        mech_type = design.get("type", "hinge")
        scale = design.get("scale_m", 0.05)
        model_id = design.get("id", "mechanism").replace("-", "_").replace(" ", "_")

        builders = {
            "hinge": _mjcf_hinge,
            "slider": _mjcf_slider,
            "four_bar_linkage": _mjcf_four_bar,
            "cam_follower": _mjcf_cam_follower,
            "gear_pair": _mjcf_gear_pair,
            "universal_joint": _mjcf_universal_joint,
        }
        fn = builders.get(mech_type, _mjcf_hinge)
        return fn(model_id, scale)


def _mjcf_hinge(model_id: str, scale: float) -> str:
    return f'''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{model_id}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002"/>
  <default>
    <joint damping="0.1" armature="0.001"/>
    <geom friction="0.5 0.005 0.0001" rgba="0.6 0.6 0.7 1"/>
  </default>
  <worldbody>
    <geom name="ground" type="plane" size="0.5 0.5 0.1" condim="3"/>
    <light pos="0 0 1" dir="0 0 -1" directional="true"/>
    <body name="link0" pos="0 0 {scale}">
      <joint name="hinge0" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
      <geom name="link0_geom" type="capsule" fromto="0 0 0 0 0 {scale}" size="{scale*0.2}"/>
      <body name="link1" pos="0 0 {scale}">
        <joint name="hinge1" type="hinge" axis="0 1 0" range="-1 1"/>
        <geom name="link1_geom" type="capsule" fromto="0 0 0 0 0 {scale}" size="{scale*0.15}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="act0" joint="hinge0" gear="10" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos name="q0" joint="hinge0"/>
    <jointvel name="dq0" joint="hinge0"/>
  </sensor>
</mujoco>
'''


def _mjcf_slider(model_id: str, scale: float) -> str:
    return f'''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{model_id}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002"/>
  <default>
    <joint damping="0.1"/>
    <geom friction="0.5 0.005 0.0001" rgba="0.6 0.6 0.7 1"/>
  </default>
  <worldbody>
    <geom name="ground" type="plane" size="0.5 0.5 0.1" condim="3"/>
    <light pos="0 0 1" dir="0 0 -1" directional="true"/>
    <body name="slider" pos="0 0 {scale}">
      <joint name="slide0" type="slide" axis="0 0 1" range="-{scale} {scale}"/>
      <geom name="slider_geom" type="box" size="{scale*0.3} {scale*0.2} {scale*0.1}"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="act0" joint="slide0" gear="5" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos name="q0" joint="slide0"/>
  </sensor>
</mujoco>
'''


def _mjcf_four_bar(model_id: str, scale: float) -> str:
    L = scale
    return f'''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{model_id}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002"/>
  <default>
    <joint damping="0.08" armature="0.001"/>
    <geom friction="0.5 0.005 0.0001" rgba="0.5 0.6 0.7 1"/>
  </default>
  <worldbody>
    <geom name="ground" type="plane" size="0.5 0.5 0.1" condim="3"/>
    <light pos="0 0 1" dir="0 0 -1" directional="true"/>
    <body name="crank" pos="0 0 {L}">
      <joint name="crank_joint" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
      <geom name="crank_geom" type="capsule" fromto="0 0 0 {L} 0 0" size="{L*0.08}"/>
      <body name="coupler" pos="{L} 0 0">
        <joint name="coupler_joint" type="hinge" axis="0 0 1" range="-2 2"/>
        <geom name="coupler_geom" type="capsule" fromto="0 0 0 {-L*0.6} {L*0.5} 0" size="{L*0.06}"/>
        <body name="rocker" pos="{-L*0.6} {L*0.5} 0">
          <joint name="rocker_joint" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
          <geom name="rocker_geom" type="capsule" fromto="0 0 0 {-L*0.4} {-L*0.5} 0" size="{L*0.05}"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="crank_act" joint="crank_joint" gear="15" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos name="crank_pos" joint="crank_joint"/>
    <jointpos name="coupler_pos" joint="coupler_joint"/>
    <jointpos name="rocker_pos" joint="rocker_joint"/>
  </sensor>
</mujoco>
'''


def _mjcf_cam_follower(model_id: str, scale: float) -> str:
    return f'''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{model_id}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002"/>
  <default>
    <joint damping="0.1"/>
    <geom friction="0.5 0.005 0.0001" rgba="0.6 0.55 0.65 1"/>
  </default>
  <worldbody>
    <geom name="ground" type="plane" size="0.5 0.5 0.1" condim="3"/>
    <light pos="0 0 1" dir="0 0 -1" directional="true"/>
    <body name="cam" pos="0 0 {scale}">
      <joint name="cam_joint" type="hinge" axis="0 0 1" range="-6.28 6.28"/>
      <geom name="cam_geom" type="cylinder" size="{scale*0.4} {scale*0.15}" euler="0 90 0"/>
      <body name="follower" pos="{scale*0.55} 0 0">
        <joint name="follower_joint" type="slide" axis="1 0 0" range="-{scale*0.3} {scale*0.3}"/>
        <geom name="follower_geom" type="box" size="{scale*0.1} {scale*0.2} {scale*0.15}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="cam_act" joint="cam_joint" gear="20" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos name="cam_pos" joint="cam_joint"/>
    <jointpos name="follower_pos" joint="follower_joint"/>
  </sensor>
</mujoco>
'''


def _mjcf_gear_pair(model_id: str, scale: float) -> str:
    r = scale * 0.5
    return f'''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{model_id}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002"/>
  <default>
    <joint damping="0.05" armature="0.001"/>
    <geom friction="0.3 0.005 0.0001" rgba="0.65 0.6 0.55 1"/>
  </default>
  <worldbody>
    <geom name="ground" type="plane" size="0.5 0.5 0.1" condim="3"/>
    <light pos="0 0 1" dir="0 0 -1" directional="true"/>
    <body name="gear1" pos="0 0 {scale}">
      <joint name="gear1_joint" type="hinge" axis="0 0 1" range="-6.28 6.28"/>
      <geom name="gear1_geom" type="cylinder" size="{r} {scale*0.1}"/>
      <body name="gear2" pos="{r*2.1} 0 0">
        <joint name="gear2_joint" type="hinge" axis="0 0 1" range="-6.28 6.28"/>
        <geom name="gear2_geom" type="cylinder" size="{r*0.7} {scale*0.1}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="gear1_act" joint="gear1_joint" gear="25" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos name="gear1_pos" joint="gear1_joint"/>
    <jointpos name="gear2_pos" joint="gear2_joint"/>
  </sensor>
</mujoco>
'''


def _mjcf_universal_joint(model_id: str, scale: float) -> str:
    return f'''<?xml version="1.0" encoding="utf-8"?>
<mujoco model="{model_id}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.002"/>
  <default>
    <joint damping="0.08" armature="0.001"/>
    <geom friction="0.5 0.005 0.0001" rgba="0.6 0.6 0.7 1"/>
  </default>
  <worldbody>
    <geom name="ground" type="plane" size="0.5 0.5 0.1" condim="3"/>
    <light pos="0 0 1" dir="0 0 -1" directional="true"/>
    <body name="link0" pos="0 0 {scale}">
      <joint name="hinge0" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
      <geom name="link0_geom" type="capsule" fromto="0 0 0 0 0 {scale}" size="{scale*0.15}"/>
      <body name="link1" pos="0 0 {scale}">
        <joint name="hinge1" type="hinge" axis="1 0 0" range="-1.57 1.57"/>
        <geom name="link1_geom" type="capsule" fromto="0 0 0 0 0 {scale}" size="{scale*0.12}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="act0" joint="hinge0" gear="10" ctrllimited="true" ctrlrange="-1 1"/>
    <motor name="act1" joint="hinge1" gear="10" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
  <sensor>
    <jointpos name="q0" joint="hinge0"/>
    <jointpos name="q1" joint="hinge1"/>
    <jointvel name="dq0" joint="hinge0"/>
    <jointvel name="dq1" joint="hinge1"/>
  </sensor>
</mujoco>
'''


def _mjcf_linkage(model_id: str, scale: float) -> str:
    return _mjcf_hinge(model_id, scale)
