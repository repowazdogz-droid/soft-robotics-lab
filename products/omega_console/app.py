"""
OMEGA Console - Single Entry Point
"""
import base64
import os
import streamlit as st
import requests
import sys
from pathlib import Path

# Reality Bridge URL (env for cross-machine; default local 18000)
REALITY_BRIDGE_URL = os.environ.get("REALITY_BRIDGE_URL", "http://localhost:18000")

# Add products to path
sys.path.insert(0, str(Path(__file__).parent.parent))
from shared.id_generator import generate_id
from shared.tutor_links import get_failure_tutor_link
from shared.demo_pack import get_demo, DEMO_DESCRIPTIONS
from shared.trust import get_trust_score, get_trust_metrics

st.set_page_config(
    page_title="OMEGA Console",
    page_icon="🎯",
    layout="wide"
)

# Product display name -> (path under products/, port, run hint)
# Use "python -m streamlit" to bypass blocked streamlit.exe (e.g. Device Guard on Windows)
PRODUCT_RUN = {
    "Foundry": ("omega_foundry", 8501, "python -m streamlit run app.py --server.port 8501"),
    "Reality Bridge": ("reality_bridge", 18000, "uvicorn app:app --reload --port 18000"),
    "World Model": ("world_model_studio", 8502, "python -m streamlit run app.py --server.port 8502"),
    "Breakthrough": ("breakthrough_engine", None, "python hypothesis_ledger.py --help"),
    "Decision Brief": ("enterprise/decision_brief", None, "python decision_brief.py --help"),
    "Tutor": ("omega_tutor", 8503, "python -m streamlit run app.py --server.port 8503"),
}


# --- Health Checks ---
def check_reality_bridge():
    """Check if Reality Bridge API is running (uses REALITY_BRIDGE_URL env)."""
    try:
        r = requests.get(f"{REALITY_BRIDGE_URL.rstrip('/')}/health", timeout=2)
        return r.status_code == 200
    except Exception:
        return False


def check_service(name: str, port: int = None, path: str = None):
    """Generic service checker"""
    products_dir = Path(__file__).parent.parent
    if path:
        return (products_dir / path).exists()
    return True


def _validate_demo(mjcf_content: str):
    """POST MJCF to Reality Bridge /validate; return response dict or None (uses REALITY_BRIDGE_URL)."""
    try:
        r = requests.post(
            f"{REALITY_BRIDGE_URL.rstrip('/')}/validate",
            data={"xml_string": mjcf_content},
            timeout=10,
        )
        if r.status_code != 200:
            return None
        return r.json()
    except Exception:
        return None


def _fetch_audit_bundle(mjcf_content: str):
    """POST MJCF to Reality Bridge /validate; return (zip_bytes, filename) or (None, None) (uses REALITY_BRIDGE_URL)."""
    try:
        r = requests.post(
            f"{REALITY_BRIDGE_URL.rstrip('/')}/validate",
            data={"xml_string": mjcf_content},
            timeout=10,
        )
        if r.status_code != 200:
            return None, None
        data = r.json()
        b64 = data.get("bundle_base64")
        name = data.get("bundle_filename")
        if not b64 or not name:
            return None, None
        return base64.b64decode(b64), name
    except Exception:
        return None, None


def get_product_status():
    """Get status of all products"""
    return {
        "Foundry": {"status": check_service("foundry", path="omega_foundry/app.py"), "port": 8501, "path": "omega_foundry", "hint": PRODUCT_RUN["Foundry"][2]},
        "Reality Bridge": {"status": check_reality_bridge(), "port": 18000, "path": "reality_bridge", "hint": PRODUCT_RUN["Reality Bridge"][2]},
        "World Model": {"status": check_service("world_model", path="world_model_studio/app.py"), "port": 8502, "path": "world_model_studio", "hint": PRODUCT_RUN["World Model"][2]},
        "Breakthrough": {"status": check_service("breakthrough", path="breakthrough_engine/hypothesis_ledger.py"), "port": None, "path": "breakthrough_engine", "hint": PRODUCT_RUN["Breakthrough"][2]},
        "Decision Brief": {"status": check_service("decision", path="enterprise/decision_brief/decision_brief.py"), "port": None, "path": "enterprise/decision_brief", "hint": PRODUCT_RUN["Decision Brief"][2]},
        "Tutor": {"status": check_service("tutor", path="omega_tutor/app.py"), "port": 8503, "path": "omega_tutor", "hint": PRODUCT_RUN["Tutor"][2]},
    }


# --- UI ---
st.title("🎯 OMEGA Console")

# Trust Score and metrics
col1, col2, col3 = st.columns([2, 1, 1])
with col1:
    trust_score = get_trust_score()
    st.metric("Trust Score", f"{trust_score}/100", help="System trustworthiness (0-100)")
    with st.expander("Trust score breakdown"):
        metrics = get_trust_metrics()
        weights = {"first_run_success": "30%", "zero_traceback_rate": "20%", "reproducibility_rate": "20%", "failure_clarity_rate": "20%", "uptime_rate": "10%"}
        for k in ["first_run_success", "zero_traceback_rate", "reproducibility_rate", "failure_clarity_rate", "uptime_rate"]:
            v = metrics.get(k, 0)
            label = k.replace("_", " ").title()
            st.metric(label, f"{v:.1f}%", help=f"Weight: {weights.get(k, '')}")
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
    for key in ("good", "bad", "edge"):
        info = DEMO_DESCRIPTIONS[key]
        with st.expander(f"{'✅' if key == 'good' else '❌' if key == 'bad' else '⚠️'} {info['name']}", expanded=False):
            st.markdown(info["description"])
            st.caption(f"**Expected:** {info['expected']}")
            if st.button("Validate This", key=f"validate_{key}", use_container_width=True):
                result = _validate_demo(get_demo(key))
                st.session_state[f"validation_{key}"] = result if result is not None else {"success": False, "message": "Reality Bridge not running"}
            res = st.session_state.get(f"validation_{key}")
            if res is not None:
                if res.get("success"):
                    passed = res.get("passed", False)
                    score = res.get("score", 0)
                    st.metric("Result", "Pass" if passed else "Fail", f"Score: {score:.2f}")
                    if res.get("errors"):
                        st.error("Errors: " + "; ".join(res["errors"][:3]))
                    if res.get("warnings"):
                        st.warning("Warnings: " + "; ".join(res["warnings"][:3]))
                    if not passed and key == "bad":
                        tutor_url = get_failure_tutor_link("PHYSICS_INSTABILITY")
                        if tutor_url:
                            st.link_button("📚 Learn why", tutor_url, type="secondary")
                    if res.get("warnings") and key == "edge":
                        tutor_url = get_failure_tutor_link("GEOMETRY_SELF_INTERSECTION")
                        if tutor_url:
                            st.link_button("📚 Learn why", tutor_url, type="secondary")
                else:
                    st.error(res.get("message", "Validation failed"))
    # Export Audit Bundle: use known-good gripper
    if st.button("📦 Export Audit Bundle", use_container_width=True, key="export_bundle"):
        st.session_state["export_bundle_clicked"] = True
    if st.session_state.get("export_bundle_clicked"):
        bundle_bytes, filename = _fetch_audit_bundle(get_demo("good"))
        if bundle_bytes and filename:
            st.download_button(
                f"Download {filename}",
                data=bundle_bytes,
                file_name=filename,
                mime="application/zip",
                key="dl_bundle",
            )
        else:
            st.warning("Start Reality Bridge (port 8000) to export a validated audit bundle.")

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
# Start Reality Bridge API (port 18000; or set REALITY_BRIDGE_PORT)
cd products/reality_bridge && uvicorn app:app --reload --port 18000

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
