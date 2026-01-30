"""
OMEGA Console - Single Entry Point
"""
import streamlit as st
import requests
import sys
from pathlib import Path

# Add products to path
sys.path.insert(0, str(Path(__file__).parent.parent))
from shared.id_generator import generate_id

st.set_page_config(
    page_title="OMEGA Console",
    page_icon="🎯",
    layout="wide"
)

# Product display name -> (path under products/, port, run hint)
# Use "python -m streamlit" to bypass blocked streamlit.exe (e.g. Device Guard on Windows)
PRODUCT_RUN = {
    "Foundry": ("omega_foundry", 8501, "python -m streamlit run app.py --server.port 8501"),
    "Reality Bridge": ("reality_bridge", 8000, "uvicorn app:app --reload --port 8000"),
    "World Model": ("world_model_studio", 8502, "python -m streamlit run app.py --server.port 8502"),
    "Breakthrough": ("breakthrough_engine", None, "python hypothesis_ledger.py --help"),
    "Decision Brief": ("enterprise/decision_brief", None, "python decision_brief.py --help"),
    "Tutor": ("omega_tutor", 8503, "python -m streamlit run app.py --server.port 8503"),
}


# --- Health Checks ---
def check_reality_bridge():
    """Check if Reality Bridge API is running"""
    try:
        r = requests.get("http://localhost:8000/health", timeout=2)
        return r.status_code == 200
    except Exception:
        return False


def check_service(name: str, port: int = None, path: str = None):
    """Generic service checker"""
    products_dir = Path(__file__).parent.parent
    if path:
        return (products_dir / path).exists()
    return True


def get_product_status():
    """Get status of all products"""
    return {
        "Foundry": {"status": check_service("foundry", path="omega_foundry/app.py"), "port": 8501, "path": "omega_foundry", "hint": PRODUCT_RUN["Foundry"][2]},
        "Reality Bridge": {"status": check_reality_bridge(), "port": 8000, "path": "reality_bridge", "hint": PRODUCT_RUN["Reality Bridge"][2]},
        "World Model": {"status": check_service("world_model", path="world_model_studio/app.py"), "port": 8502, "path": "world_model_studio", "hint": PRODUCT_RUN["World Model"][2]},
        "Breakthrough": {"status": check_service("breakthrough", path="breakthrough_engine/hypothesis_ledger.py"), "port": None, "path": "breakthrough_engine", "hint": PRODUCT_RUN["Breakthrough"][2]},
        "Decision Brief": {"status": check_service("decision", path="enterprise/decision_brief/decision_brief.py"), "port": None, "path": "enterprise/decision_brief", "hint": PRODUCT_RUN["Decision Brief"][2]},
        "Tutor": {"status": check_service("tutor", path="omega_tutor/app.py"), "port": 8503, "path": "omega_tutor", "hint": PRODUCT_RUN["Tutor"][2]},
    }


# --- UI ---
st.title("🎯 OMEGA Console")

# Trust Score (placeholder for now)
col1, col2, col3 = st.columns([2, 1, 1])
with col1:
    st.metric("Trust Score", "—/100", help="Calculated after Trust Release")
with col2:
    st.metric("Validations", "—", help="Total validations in Reality Bridge")
with col3:
    st.metric("Hypotheses", "—", help="Active hypotheses in Breakthrough Engine")

st.divider()

# --- Golden Path ---
st.subheader("🚀 Start Here")

col1, col2 = st.columns(2)

with col1:
    if st.button("▶ Start Golden Path", type="primary", use_container_width=True):
        st.info("""
**Golden Path: Intent → Design → Validate → Learn → Decide**

1. Open **Foundry** - describe what you want to build
2. Design is generated and sent to **Reality Bridge**
3. Validation results show pass/fail
4. If confused, **Tutor** explains the failure
5. Use **Decision Brief** for complex choices

Click a product below to begin.
""")

with col2:
    st.markdown("**📦 Demo Pack**")
    demo_col1, demo_col2, demo_col3 = st.columns(3)
    with demo_col1:
        if st.button("✅ Known-Good", use_container_width=True, key="demo_good"):
            st.success("Egg gripper - validated, stable, manufacturable")
    with demo_col2:
        if st.button("❌ Known-Bad", use_container_width=True, key="demo_bad"):
            st.error("Unstable mass - fails physics validation")
    with demo_col3:
        if st.button("⚠️ Known-Edge", use_container_width=True, key="demo_edge"):
            st.warning("Self-collision - passes with warnings")

st.divider()

# --- Product Grid ---
st.subheader("Products")

products = get_product_status()
product_list = list(products.items())

row1 = st.columns(3)
row2 = st.columns(3)

for i, (name, info) in enumerate(product_list[:3]):
    with row1[i]:
        status_icon = "🟢" if info["status"] else "🔴"
        st.markdown(f"### {status_icon} {name}")
        if info["port"]:
            if st.button(f"Open {name}", key=f"open_{name.replace(' ', '_')}", use_container_width=True):
                st.info(f"`cd products/{info['path']} && {info['hint']}`")
        else:
            if st.button(f"Open {name}", key=f"open_{name.replace(' ', '_')}", use_container_width=True):
                st.info(f"CLI: `cd products/{info['path']} && {info['hint']}`")

for i, (name, info) in enumerate(product_list[3:]):
    with row2[i]:
        status_icon = "🟢" if info["status"] else "🔴"
        st.markdown(f"### {status_icon} {name}")
        if info["port"]:
            if st.button(f"Open {name}", key=f"open2_{name.replace(' ', '_')}", use_container_width=True):
                st.info(f"`cd products/{info['path']} && {info['hint']}`")
        else:
            if st.button(f"Open {name}", key=f"open2_{name.replace(' ', '_')}", use_container_width=True):
                st.info(f"CLI: `cd products/{info['path']} && {info['hint']}`")

st.divider()

# --- Quick Commands ---
st.subheader("Quick Commands")

st.code("""
# Start Reality Bridge API
cd products/reality_bridge && uvicorn app:app --reload --port 8000

# Start Foundry
cd products/omega_foundry && python -m streamlit run app.py --server.port 8501

# Start World Model Studio
cd products/world_model_studio && python -m streamlit run app.py --server.port 8502

# Start Tutor
cd products/omega_tutor && python -m streamlit run app.py --server.port 8503

# Start Console (this)
cd products/omega_console && python -m streamlit run app.py --server.port 8500
""", language="bash")

# --- Footer ---
st.divider()
all_ok = all(p["status"] for p in products.values())
st.caption("OMEGA Stack v2.0 | All systems operational" if all_ok else "OMEGA Stack v2.0 | Some systems offline")
