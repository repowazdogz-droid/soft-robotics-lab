"""
First Run Setup — Lab name, first hypothesis, Gripper Zoo, Weekly Brief explanation.
"""

import streamlit as st
import sys
from pathlib import Path

_soft_lab = Path(__file__).resolve().parent.parent
if str(_soft_lab) not in sys.path:
    sys.path.insert(0, str(_soft_lab))

from utils.status import get_system_status, render_status_banner

st.set_page_config(
    page_title="First Run Setup",
    page_icon="⚙️",
    layout="wide",
)

render_status_banner(st)
st.markdown("---")

st.markdown("# ⚙️ First Run Setup")
st.markdown("Configure your lab and get ready for research.")

st.markdown("---")

# Lab name (persisted in session for other pages to use)
if "lab_name" not in st.session_state:
    st.session_state["lab_name"] = "demo_lab"

lab_name = st.text_input(
    "Lab name",
    value=st.session_state["lab_name"],
    key="setup_lab_name",
    help="Used for Research Memory and Weekly Brief. Change to your lab identifier.",
)
if lab_name:
    st.session_state["lab_name"] = lab_name

st.markdown("---")
st.markdown("### 1. Create your first hypothesis")

st.markdown(
    "Hypotheses are the spine of the app: they drive the Weekly Brief, evidence links, and design–hypothesis links."
)

sys.path.insert(0, str(_soft_lab / "research_system"))
from research_memory import ResearchMemory

@st.cache_resource
def _get_memory(lab_name):
    return ResearchMemory(lab_name)

memory = _get_memory(lab_name)
stats = memory.get_stats()

if stats["hypotheses"] == 0:
    with st.expander("Create first hypothesis", expanded=True):
        claim = st.text_area(
            "Claim (falsifiable)",
            placeholder="e.g., Tendon-driven grippers achieve higher precision than pneumatic for objects under 50g in dry conditions",
            key="setup_claim",
        )
        scope = st.text_input("Scope / conditions", placeholder="e.g., Room temperature, objects 10–50g", key="setup_scope")
        if st.button("Create hypothesis", type="primary", key="setup_create_hyp"):
            if claim:
                hyp = memory.create_hypothesis(
                    claim=claim,
                    scope=scope or "Not specified",
                    assumptions=[],
                    predictions=[],
                    created_by=lab_name,
                )
                _get_memory.clear()
                st.success(f"Created: {hyp.id}. Go to Research Memory to manage hypotheses.")
                st.rerun()
            else:
                st.warning("Enter a claim.")
else:
    st.success(f"You have {stats['hypotheses']} hypothesis/hypotheses. Manage them in **Research Memory**.")

st.markdown("---")
st.markdown("### 2. Gripper Zoo")

status = get_system_status()
if not status["zoo_ready"]:
    st.markdown("The Gripper Zoo provides 50+ pre-generated designs. Generate it once to browse and download from the Zoo page.")
    if st.button("Generate Gripper Zoo", type="primary", key="setup_gen_zoo"):
        with st.spinner("Generating 50 designs…"):
            try:
                import importlib.util
                zoo_script = _soft_lab / "gripper_zoo" / "generate_zoo.py"
                spec = importlib.util.spec_from_file_location("generate_zoo", zoo_script)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                designs = mod.generate_zoo()
                st.success(f"Generated {len(designs)} designs. Go to **Gripper Zoo** to browse.")
                st.rerun()
            except Exception as e:
                st.error(str(e))
else:
    st.success("Gripper Zoo is ready. Browse designs on the **Gripper Zoo** page.")

st.markdown("---")
st.markdown("### 3. Weekly Brief")

st.info(
    "**Weekly Brief** is hypothesis-driven: it summarizes what changed in your hypotheses, contradictions, discriminators, and discussion questions. "
    "Add and update hypotheses in Research Memory to see richer briefs."
)

st.markdown("---")
st.markdown("[Open Research Memory](Research_Memory) · [Open Gripper Zoo](Gripper_Zoo) · [Generate Weekly Brief](Weekly_Brief)")
