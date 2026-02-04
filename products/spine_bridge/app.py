"""
Spine Bridge — Streamlit UI.
Upload MJCF, enter problem context, run Analyze → Reality Bridge → Spine → combined report.
"""

import os
import sys
from pathlib import Path

_spine_bridge = Path(__file__).resolve().parent
if str(_spine_bridge) not in sys.path:
    sys.path.insert(0, str(_spine_bridge))

import streamlit as st

from pipeline import analyze_design
from reality_bridge_client import get_base_url, health_check, RealityBridgeUnavailable

st.set_page_config(
    page_title="Spine Bridge",
    page_icon="🔗",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
    .main-header { font-size: 2rem; font-weight: 700; color: #0ea5e9; margin-bottom: 0; }
    .sub-header { color: #64748b; margin-top: 0; }
    .report-section { background: #1e293b; border-radius: 8px; padding: 1rem; margin: 0.5rem 0; border: 1px solid #334155; }
    .pass-badge { color: #22c55e; font-weight: 600; }
    .fail-badge { color: #ef4444; font-weight: 600; }
    .stTabs [data-baseweb="tab"] { background-color: #1e293b; border-radius: 6px; }
</style>
""", unsafe_allow_html=True)

# Sidebar
with st.sidebar:
    st.markdown("## 🔗 Spine Bridge")
    st.markdown("Design → Physics → Decision")
    st.markdown("---")
    rb_url = get_base_url()
    st.markdown(f"**Reality Bridge:** `{rb_url}`")
    try:
        health = health_check()
        st.success(f"Reality Bridge OK — MuJoCo {health.get('mujoco_version', '?')}")
    except RealityBridgeUnavailable:
        st.warning("Reality Bridge offline — physics step will show error.")
    st.markdown("---")
    st.markdown("Set `REALITY_BRIDGE_URL` to change API URL.")

st.markdown('<p class="main-header">Spine Bridge</p>', unsafe_allow_html=True)
st.markdown('<p class="sub-header">Upload a design (MJCF), set problem context, run physics validation and decision analysis.</p>', unsafe_allow_html=True)

# Session state defaults for "Load Sample Design"
if "ctx_name" not in st.session_state:
    st.session_state["ctx_name"] = "My design"
if "ctx_domain" not in st.session_state:
    st.session_state["ctx_domain"] = "robotics"
if "ctx_obj" not in st.session_state:
    st.session_state["ctx_obj"] = "Stable simulation, feasible grasp"
if "design_xml" not in st.session_state:
    st.session_state["design_xml"] = None

# Inputs
col1, col2 = st.columns([1, 1])

with col1:
    st.subheader("Design")
    load_sample = st.button("Load Sample Design", key="load_sample")
    if load_sample:
        sample_path = _spine_bridge / "sample_design.mjcf"
        if sample_path.exists():
            st.session_state["design_xml"] = sample_path.read_text(encoding="utf-8")
            st.session_state["ctx_name"] = "Soft gripper for fragile tissue manipulation"
            st.session_state["ctx_domain"] = "surgical_robotics"
            st.session_state["ctx_obj"] = "safe_manipulation, adequate_grip, minimal_tissue_damage"
            st.success("Sample design and defaults loaded.")
        else:
            st.error("sample_design.mjcf not found.")
    st.caption("Or upload your own MJCF below.")
    design_file = st.file_uploader("Upload MJCF", type=["mjcf", "xml"], key="mjcf_upload")
    design_path = None
    design_xml = None
    if design_file:
        design_xml = design_file.read().decode("utf-8", errors="replace")
        st.session_state["design_xml"] = design_xml
    elif st.session_state.get("design_xml"):
        design_xml = st.session_state["design_xml"]
    if design_xml:
        with st.expander("View loaded MJCF", expanded=False):
            st.code(design_xml, language="xml")

with col2:
    st.subheader("Problem context")
    context_name = st.text_input("Name", value=st.session_state["ctx_name"], key="ctx_name")
    context_domain = st.text_input("Domain", value=st.session_state["ctx_domain"], key="ctx_domain")
    context_objectives = st.text_area(
        "Objectives (one per line or comma-separated)",
        value=st.session_state["ctx_obj"],
        height=80,
        key="ctx_obj",
    )
    objectives_list = [
        x.strip() for x in context_objectives.replace(",", "\n").splitlines() if x.strip()
    ]

analyze_clicked = st.button("Analyze", type="primary")

if analyze_clicked:
    if not design_xml and not design_path:
        st.error("Upload an MJCF file first.")
    else:
        problem_context = {
            "name": context_name,
            "domain": context_domain,
            "objectives": objectives_list or [context_objectives],
        }
        with st.spinner("Running pipeline: Reality Bridge → Case builder → Spine..."):
            out = analyze_design(
                design_path=design_path,
                design_xml=design_xml,
                problem_context=problem_context,
            )

        err = out.get("error")
        if err:
            st.warning(f"Pipeline note: {err}")

        # Physics report
        st.subheader("Physics report")
        pr = out.get("physics_report") or {}
        if pr.get("passed") is True:
            st.markdown('<span class="pass-badge">✓ Passed</span>', unsafe_allow_html=True)
        elif pr.get("passed") is False:
            st.markdown('<span class="fail-badge">✗ Failed</span>', unsafe_allow_html=True)
        else:
            st.info("Physics validation was not available (Reality Bridge offline or error).")

        if pr:
            st.metric("Score", f"{pr.get('score', 0):.2f}" if pr.get("score") is not None else "—")
            if pr.get("validation_time_ms") is not None:
                st.caption(f"Validation time: {pr['validation_time_ms']} ms")
            if pr.get("failures"):
                with st.expander("Failures"):
                    for f in pr["failures"]:
                        st.markdown(f"- **{f.get('code', '')}**: {f.get('message', '')}")
                        if f.get("suggestions"):
                            for s in f["suggestions"]:
                                st.caption(f"  → {s}")
            if pr.get("warnings"):
                with st.expander("Warnings"):
                    for w in pr["warnings"]:
                        st.markdown(f"- {w}")

        # Spine case
        st.subheader("Spine case (constraints & uncertainties)")
        case = out.get("spine_case") or {}
        st.json(case)

        # Decision analysis
        st.subheader("Decision analysis")
        da = out.get("decision_analysis") or {}
        st.json(da)
        if not out.get("spine_available"):
            st.caption("Spine runtime not installed; showing stub analysis. Add spine/runtime for full decision analysis.")

# Sample file hint
st.markdown("---")
with st.expander("Sample MJCF"):
    sample_path = _spine_bridge / "sample_design.mjcf"
    if sample_path.exists():
        st.code(sample_path.read_text(encoding="utf-8"), language="xml")
    else:
        st.info("Save a minimal MJCF as sample_design.mjcf in this folder for testing.")
