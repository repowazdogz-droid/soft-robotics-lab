"""
Compare Designs Page
"""

import streamlit as st
import sys
from pathlib import Path

_soft_lab = Path(__file__).resolve().parent.parent
if str(_soft_lab) not in sys.path:
    sys.path.insert(0, str(_soft_lab))

from workbench.motion_to_morphology import MotionToMorphology

st.set_page_config(
    page_title="Compare Designs", page_icon="⚖️", layout="wide"
)

st.markdown("# ⚖️ Compare Designs")
st.markdown("Side-by-side comparison of two gripper designs")

st.markdown("---")

m2m = MotionToMorphology()

col1, col2 = st.columns(2)

with col1:
    st.markdown("### Design A")

    gesture_a = st.selectbox(
        "Gesture", ["pinch", "power", "wrap", "hook"], key="gesture_a"
    )
    aperture_a = st.slider(
        "Aperture (mm)", 20, 150, 50, key="aperture_a"
    )
    compliance_a = st.slider(
        "Compliance", 0.0, 1.0, 0.3, key="compliance_a"
    )
    env_a = st.selectbox(
        "Environment", ["dry", "wet", "surgical"], key="env_a"
    )

with col2:
    st.markdown("### Design B")

    gesture_b = st.selectbox(
        "Gesture",
        ["pinch", "power", "wrap", "hook"],
        index=1,
        key="gesture_b",
    )
    aperture_b = st.slider(
        "Aperture (mm)", 20, 150, 80, key="aperture_b"
    )
    compliance_b = st.slider(
        "Compliance", 0.0, 1.0, 0.7, key="compliance_b"
    )
    env_b = st.selectbox(
        "Environment",
        ["dry", "wet", "surgical"],
        index=1,
        key="env_b",
    )

if st.button("🔄 Compare", type="primary", use_container_width=True):
    with st.spinner("Generating designs..."):
        design_a = m2m.from_gesture(
            gesture_a, aperture_a / 1000, 5.0, compliance_a, env_a
        )
        design_b = m2m.from_gesture(
            gesture_b, aperture_b / 1000, 5.0, compliance_b, env_b
        )

    st.markdown("---")
    st.markdown("## Comparison Results")

    col1, col2 = st.columns(2)

    with col1:
        conf_color_a = (
            "#00ff88"
            if design_a.confidence > 0.75
            else "#ffaa00"
            if design_a.confidence > 0.5
            else "#ff4444"
        )
        st.markdown(
            f"""
        <div style="background: #1a1a2e; padding: 20px; border-radius: 10px;">
            <h3 style="color: #00d4ff;">{design_a.name}</h3>
            <p style="color: {conf_color_a}; font-size: 2rem; font-weight: bold;">{design_a.confidence*100:.0f}%</p>
            <p>Confidence</p>
        </div>
        """,
            unsafe_allow_html=True,
        )

    with col2:
        conf_color_b = (
            "#00ff88"
            if design_b.confidence > 0.75
            else "#ffaa00"
            if design_b.confidence > 0.5
            else "#ff4444"
        )
        st.markdown(
            f"""
        <div style="background: #1a1a2e; padding: 20px; border-radius: 10px;">
            <h3 style="color: #ffaa00;">{design_b.name}</h3>
            <p style="color: {conf_color_b}; font-size: 2rem; font-weight: bold;">{design_b.confidence*100:.0f}%</p>
            <p>Confidence</p>
        </div>
        """,
            unsafe_allow_html=True,
        )

    st.markdown("### Specifications")

    import pandas as pd

    comparison_data = {
        "Property": [
            "Fingers",
            "Actuator",
            "Max Force",
            "Speed",
            "Material",
            "Segments",
        ],
        design_a.name[:20]: [
            design_a.num_fingers,
            design_a.primary_actuator.value,
            f"{design_a.max_force_n:.1f}N",
            f"{design_a.grasp_speed_mm_per_s:.0f}mm/s",
            design_a.finger_designs[0].material.value,
            design_a.finger_designs[0].num_segments,
        ],
        design_b.name[:20]: [
            design_b.num_fingers,
            design_b.primary_actuator.value,
            f"{design_b.max_force_n:.1f}N",
            f"{design_b.grasp_speed_mm_per_s:.0f}mm/s",
            design_b.finger_designs[0].material.value,
            design_b.finger_designs[0].num_segments,
        ],
    }

    df = pd.DataFrame(comparison_data)
    st.table(df)

    st.markdown("### Recommendation")

    if design_a.confidence > design_b.confidence + 0.1:
        st.success(
            f"**{design_a.name}** has higher confidence ({design_a.confidence*100:.0f}% vs {design_b.confidence*100:.0f}%)"
        )
    elif design_b.confidence > design_a.confidence + 0.1:
        st.success(
            f"**{design_b.name}** has higher confidence ({design_b.confidence*100:.0f}% vs {design_a.confidence*100:.0f}%)"
        )
    else:
        st.info(
            "Both designs have similar confidence levels. Choose based on specific requirements."
        )
