#!/usr/bin/env python3
"""
OMEGA Soft Robotics Lab
=======================

A research toolkit for soft robot design, analysis, and knowledge management.

Run: streamlit run app.py
"""

import streamlit as st
import sys
from pathlib import Path

_soft_lab = Path(__file__).resolve().parent
if str(_soft_lab) not in sys.path:
    sys.path.insert(0, str(_soft_lab))

from utils.status import get_system_status, render_status_banner

st.set_page_config(
    page_title="OMEGA Soft Robotics Lab",
    page_icon="🦾",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Custom CSS for polish
st.markdown("""
<style>
    .main-header {
        font-size: 2.5rem;
        font-weight: 700;
        color: #00d4ff;
        margin-bottom: 0;
    }
    .sub-header {
        font-size: 1.1rem;
        color: #888;
        margin-top: 0;
    }
    .metric-card {
        background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
        border-radius: 10px;
        padding: 20px;
        border: 1px solid #333;
    }
    .stTabs [data-baseweb="tab-list"] {
        gap: 8px;
    }
    .stTabs [data-baseweb="tab"] {
        background-color: #1a1a2e;
        border-radius: 8px;
        padding: 10px 20px;
    }
</style>
""", unsafe_allow_html=True)

# Sidebar
with st.sidebar:
    st.markdown("## 🦾 OMEGA")
    st.markdown("### Soft Robotics Lab")
    st.markdown("---")
    st.markdown("""
    **Tools:**
    - 🎯 Design grippers from gestures
    - ⚠️ Predict failure modes
    - ⚖️ Compare designs
    - 🦾 Browse 50 pre-built grippers
    - 📚 Research memory
    - 📋 Weekly briefs
    """)
    st.markdown("---")
    st.markdown("*Built with OMEGA*")
    st.markdown("*Physical AI Decision Support*")

# Status banner (Zoo | Embeddings | MuJoCo)
st.markdown("---")
render_status_banner(st)
st.markdown("---")

# Main content
st.markdown('<p class="main-header">🦾 OMEGA Soft Robotics Lab</p>', unsafe_allow_html=True)
st.markdown(
    '<p class="sub-header">Design soft grippers with uncertainty quantification</p>',
    unsafe_allow_html=True,
)

st.markdown("---")

# Quick stats
col1, col2, col3, col4 = st.columns(4)

with col1:
    st.metric("Gripper Zoo", "50+ designs", "Ready to use")
with col2:
    st.metric("Export Formats", "4", "MJCF, URDF, USD, JSON")
with col3:
    st.metric("Gesture Types", "7", "Pinch, Power, Wrap...")
with col4:
    st.metric("Environments", "3", "Dry, Wet, Surgical")

st.markdown("---")

# Navigation cards
st.markdown("## Get Started")

col1, col2, col3 = st.columns(3)

with col1:
    st.markdown("""
    ### 🎯 Design a Gripper

    Convert human grasp gestures into soft gripper morphologies with:
    - Automatic actuator selection
    - Material optimization
    - Uncertainty quantification

    **→ Go to Gripper Design**
    """)
    if st.button("Open Gripper Design", key="btn_design"):
        st.switch_page("pages/1_Gripper_Design.py")

with col2:
    st.markdown("""
    ### 🦾 Browse Gripper Zoo

    Explore 50 pre-generated designs:
    - Filter by gesture, environment
    - Download simulation files
    - Compare specifications

    **→ Go to Gripper Zoo**
    """)
    if st.button("Open Gripper Zoo", key="btn_zoo"):
        st.switch_page("pages/4_Gripper_Zoo.py")

with col3:
    st.markdown("""
    ### 📚 Research Memory

    Your lab's private knowledge base:
    - Ingest papers and notes
    - Search with source citations
    - Track hypotheses

    **→ Go to Research Memory**
    """)
    if st.button("Open Research Memory", key="btn_memory"):
        st.switch_page("pages/5_Research_Memory.py")

st.markdown("---")

# Recent activity / quick actions
st.markdown("## Quick Actions")

col1, col2 = st.columns(2)

with col1:
    st.markdown("### Generate a Quick Design")

    gesture = st.selectbox(
        "Gesture",
        ["pinch", "power", "wrap", "hook", "lateral", "squeeze", "spread"],
    )
    environment = st.selectbox("Environment", ["dry", "wet", "surgical"])

    if st.button("Generate", type="primary", key="quick_gen"):
        st.info(
            f"→ Go to Gripper Design page to generate a {gesture} gripper for {environment} environment"
        )

with col2:
    st.markdown("### Search Research Memory")

    query = st.text_input(
        "Search your papers", placeholder="e.g., variable stiffness grippers"
    )

    if st.button("Search", type="primary", key="quick_search"):
        if query:
            st.info(f"→ Go to Research Memory page to search for: {query}")
        else:
            st.warning("Enter a search query")

st.markdown("---")

# Footer
st.markdown("""
<div style="text-align: center; color: #666; padding: 20px;">
    <p>OMEGA Soft Robotics Lab v1.0</p>
    <p>Questions? Contact the maintainer or check the README.</p>
</div>
""", unsafe_allow_html=True)
