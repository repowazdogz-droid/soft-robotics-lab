"""
Spine Bridge — Streamlit UI (demo-ready).
Upload or select sample design, set domain preset, run physics + decision analysis.
"""

import sys
from pathlib import Path

_spine_bridge = Path(__file__).resolve().parent
if str(_spine_bridge) not in sys.path:
    sys.path.insert(0, str(_spine_bridge))

import streamlit as st

from pipeline import analyze_design
from reality_bridge_client import get_base_url, health_check, RealityBridgeUnavailable

# Sample designs: (filename, display_name, domain, objectives_str)
SAMPLES = {
    "Passing (clean)": ("sample_passing.mjcf", "Soft gripper for fragile tissue", "surgical_robotics", "safe_manipulation, adequate_grip, minimal_tissue_damage"),
    "Passes with physics warnings": ("sample_warnings.mjcf", "Heavy-link test rig", "surgical_robotics", "mass_within_limit, stability, grasp_repeatability"),
    "Failing (invalid design)": ("sample_failing.mjcf", "Invalid mass design", "surgical_robotics", "valid_physics, feasible_dynamics"),
}

DOMAIN_PRESETS = {
    "Surgical Robotics": ("surgical_robotics", "Soft gripper for fragile tissue manipulation", "safe_manipulation, adequate_grip, minimal_tissue_damage, sterilization_compatible"),
    "Industrial Manipulation": ("industrial_manipulation", "Pick-and-place end effector", "payload_5kg, repeatability_0.1mm, cycle_time_30s"),
    "Delicate Assembly": ("delicate_assembly", "Micro-assembly gripper", "max_force_0.5N, no_scratch, sub_mm_alignment"),
}

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
    .summary-bar { background: linear-gradient(90deg, #1e3a5f 0%, #1e293b 100%); border-radius: 10px; padding: 1rem 1.25rem; margin: 0.5rem 0; border: 1px solid #334155; display: flex; gap: 1.5rem; flex-wrap: wrap; align-items: center; color: #e2e8f0; }
    .summary-item { font-size: 0.95rem; color: #e2e8f0; }
    .summary-num { font-weight: 700; color: #38bdf8; }
    .card-high { background: #451a1a; border-left: 4px solid #ef4444; border-radius: 8px; padding: 0.75rem 1rem; margin: 0.4rem 0; color: #f1f5f9; }
    .card-medium { background: #422006; border-left: 4px solid #f97316; border-radius: 8px; padding: 0.75rem 1rem; margin: 0.4rem 0; color: #f1f5f9; }
    .card-low { background: #422006; border-left: 4px solid #eab308; border-radius: 8px; padding: 0.75rem 1rem; margin: 0.4rem 0; color: #f1f5f9; }
    .unknown-badge { display: inline-block; background: #1e3a5f; color: #93c5fd; padding: 0.2rem 0.5rem; border-radius: 4px; font-size: 0.85rem; margin: 0.2rem 0.2rem 0.2rem 0; }
    .contradiction-icon { color: #f59e0b; }
    .check-item { margin: 0.3rem 0; }
    .demo-step { background: #1e293b; border-radius: 8px; padding: 1rem; margin: 0.5rem 0; border: 1px solid #475569; }
    .pass-badge { color: #22c55e; font-weight: 600; }
    .fail-badge { color: #ef4444; font-weight: 600; }
    .confidence-high { color: #22c55e; font-weight: 600; }
    .confidence-medium { color: #eab308; font-weight: 600; }
    .confidence-low { color: #ef4444; font-weight: 600; }
    .evidence-physics { background: #14532d; color: #86efac; padding: 0.15rem 0.4rem; border-radius: 4px; font-size: 0.75rem; margin-left: 0.25rem; }
    .evidence-rule { background: #422006; color: #fcd34d; padding: 0.15rem 0.4rem; border-radius: 4px; font-size: 0.75rem; margin-left: 0.25rem; }
    .evidence-heuristic { background: #431407; color: #fdba74; padding: 0.15rem 0.4rem; border-radius: 4px; font-size: 0.75rem; margin-left: 0.25rem; }
    .requires-validation { color: #f59e0b; font-size: 0.85rem; margin-left: 0.5rem; }
    .provenance-block { font-family: monospace; font-size: 0.8rem; color: #94a3b8; }
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
    demo_mode = st.checkbox("Demo mode (step-by-step)", value=False, key="demo_mode")
    st.caption("Walk through analysis with short explanations.")
    st.markdown("---")
    st.markdown("Set `REALITY_BRIDGE_URL` to change API URL.")

st.markdown('<p class="main-header">Spine Bridge</p>', unsafe_allow_html=True)
st.markdown('<p class="sub-header">Select a sample design or upload MJCF, set domain, then run physics validation and decision analysis.</p>', unsafe_allow_html=True)

# Session state
for key, default in [("ctx_name", "My design"), ("ctx_domain", "robotics"), ("ctx_obj", "Stable simulation, feasible grasp"), ("design_xml", None)]:
    if key not in st.session_state:
        st.session_state[key] = default

# Inputs
col1, col2 = st.columns([1, 1])

with col1:
    st.subheader("Design")
    sample_choice = st.selectbox(
        "Sample design",
        options=["— Custom upload only —"] + list(SAMPLES.keys()),
        key="sample_choice",
    )
    load_btn = st.button("Load selected sample", key="load_sample")
    if load_btn and sample_choice and sample_choice != "— Custom upload only —":
        fname, dname, ddomain, dobj = SAMPLES[sample_choice]
        path = _spine_bridge / fname
        if path.exists():
            st.session_state["design_xml"] = path.read_text(encoding="utf-8")
            st.session_state["ctx_name"] = dname
            st.session_state["ctx_domain"] = ddomain
            st.session_state["ctx_obj"] = dobj
            st.success(f"Loaded **{sample_choice}**.")
        else:
            st.error(f"File {fname} not found.")
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
    domain_preset = st.selectbox(
        "Domain preset",
        options=list(DOMAIN_PRESETS.keys()),
        key="domain_preset",
    )
    if domain_preset:
        _d, _n, _o = DOMAIN_PRESETS[domain_preset]
        st.session_state["ctx_domain"] = _d
        if st.button("Apply preset to name & objectives", key="apply_preset"):
            st.session_state["ctx_name"] = _n
            st.session_state["ctx_obj"] = _o
            st.rerun()
    context_name = st.text_input("Name", value=st.session_state["ctx_name"], key="ctx_name")
    context_domain = st.text_input("Domain", value=st.session_state["ctx_domain"], key="ctx_domain")
    context_objectives = st.text_area(
        "Objectives (one per line or comma-separated)",
        value=st.session_state["ctx_obj"],
        height=80,
        key="ctx_obj",
    )
    objectives_list = [x.strip() for x in context_objectives.replace(",", "\n").splitlines() if x.strip()]

analyze_clicked = st.button("Analyze", type="primary")

if analyze_clicked:
    if not design_xml and not design_path:
        st.error("Load a sample design or upload an MJCF first.")
    else:
        problem_context = {
            "name": context_name,
            "domain": context_domain,
            "objectives": objectives_list or [context_objectives],
        }
        if st.session_state.get("demo_mode"):
            st.markdown("### Demo: Step 1 — Physics validation")
            st.markdown("Sending design to Reality Bridge for load, stability, mass, and dynamics checks.")
        with st.spinner("Reality Bridge → Case builder → Spine..."):
            out = analyze_design(design_path=design_path, design_xml=design_xml, problem_context=problem_context)
        err = out.get("error")
        if err:
            st.warning(f"Pipeline note: {err}")

        # Summary bar
        case = out.get("spine_case") or {}
        da = out.get("decision_analysis") or {}
        n_constraints = len(case.get("constraints") or [])
        failure_modes = da.get("failure_modes") or da.get("constraints") or []
        if isinstance(failure_modes, list) and failure_modes and isinstance(failure_modes[0], dict):
            n_failure_modes = len(failure_modes)
        else:
            n_failure_modes = len(failure_modes) if isinstance(failure_modes, list) else 0
        unknowns = da.get("uncertainties") or case.get("uncertainties") or []
        n_unknowns = len(unknowns)
        st.markdown(
            f'<div class="summary-bar">'
            f'<span class="summary-item"><span class="summary-num">{n_constraints}</span> constraints checked</span>'
            f'<span class="summary-item"><span class="summary-num">{n_failure_modes}</span> failure modes</span>'
            f'<span class="summary-item"><span class="summary-num">{n_unknowns}</span> unknowns</span>'
            f'</div>',
            unsafe_allow_html=True,
        )

        if st.session_state.get("demo_mode"):
            st.markdown("### Demo: Step 2 — Physics report")
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

        if st.session_state.get("demo_mode"):
            st.markdown("### Demo: Step 3 — Decision analysis")
        st.subheader("Decision analysis")
        # Formatted output: failure modes as cards, contradictions, unknowns, recommended experiments
        failure_modes_list = da.get("failure_modes")
        if isinstance(failure_modes_list, list) and failure_modes_list:
            st.markdown("**Failure modes**")
            for i, fm in enumerate(failure_modes_list):
                if isinstance(fm, dict):
                    def _esc(s):
                        return str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                    sev = (fm.get("severity") or fm.get("level") or "medium").lower()
                    mode = fm.get("mode") or fm.get("message") or fm.get("description") or ""
                    if mode:
                        mode_label = _esc(mode).replace("_", " ").strip().title()
                    else:
                        mode_label = "Failure mode"
                    mitigation = fm.get("mitigation") or ""
                    if mitigation:
                        mitigation_label = _esc(mitigation).replace("_", " ").strip()
                        msg = f"<strong>{mode_label}</strong> ({sev}) — Mitigation: {mitigation_label}"
                    else:
                        msg = f"<strong>{mode_label}</strong> ({sev})"
                    # Epistemic: confidence level with color
                    confidence = (fm.get("confidence") or fm.get("confidence_level") or "").lower()
                    if confidence:
                        conf_cls = "confidence-high" if confidence in ("high", "strong") else "confidence-medium" if confidence in ("medium", "moderate") else "confidence-low"
                        msg += f' <span class="{conf_cls}">[{confidence}]</span>'
                    # Evidence type badge
                    evidence_type = (fm.get("evidence_type") or fm.get("evidence") or "").lower()
                    if evidence_type:
                        if evidence_type == "physics_derived":
                            msg += ' <span class="evidence-physics">physics_derived</span>'
                        elif evidence_type == "rule_derived":
                            msg += ' <span class="evidence-rule">rule_derived</span>'
                        elif evidence_type == "heuristic":
                            msg += ' <span class="evidence-heuristic">heuristic</span>'
                        else:
                            msg += f' <span class="evidence-rule">{_esc(evidence_type)}</span>'
                    # Requires validation warning
                    if fm.get("requires_validation") in (True, "true", 1):
                        msg += ' <span class="requires-validation">⚠ requires validation</span>'
                    cls = "card-high" if sev in ("high", "critical") else "card-medium" if sev == "medium" else "card-low"
                    st.markdown(f'<div class="{cls}">{msg}</div>', unsafe_allow_html=True)
                    # Provenance in expandable section
                    provenance = fm.get("provenance")
                    if provenance is not None and provenance != "":
                        with st.expander(f"Provenance — {mode_label}", expanded=False):
                            if isinstance(provenance, dict):
                                st.json(provenance)
                            else:
                                st.markdown(f'<pre class="provenance-block">{_esc(str(provenance))}</pre>', unsafe_allow_html=True)
                else:
                    st.markdown(f'<div class="card-medium" style="color:#f1f5f9;">{fm}</div>', unsafe_allow_html=True)
        constraints_list = da.get("constraints") or case.get("constraints") or []
        if constraints_list and not (isinstance(failure_modes_list, list) and len(failure_modes_list) > 0):
            st.markdown("**Constraints**")
            for c in constraints_list:
                cstr = str(c).replace("<", "&lt;").replace(">", "&gt;")
                st.markdown(f'<div class="card-low">{cstr}</div>', unsafe_allow_html=True)

        contradictions = da.get("contradictions") or []
        if contradictions:
            st.markdown("**Contradictions** <span class=\"contradiction-icon\">⚠</span>", unsafe_allow_html=True)
            for c in contradictions:
                text = c.get("description", c.get("text", str(c))) if isinstance(c, dict) else c
                st.markdown(f"- ⚠ {text}")

        unknowns_list = da.get("uncertainties") or case.get("uncertainties") or []
        if unknowns_list:
            st.markdown("**Unknowns**")
            for u in unknowns_list:
                ustr = u if isinstance(u, str) else u.get("description", str(u))
                st.markdown(f'<span class="unknown-badge">{ustr}</span>', unsafe_allow_html=True)
            st.markdown("")

        recommended = da.get("recommended_experiments") or da.get("experiments") or []
        if recommended:
            st.markdown("**Recommended experiments**")
            for r in recommended:
                rstr = r if isinstance(r, str) else r.get("name", str(r))
                st.markdown(f'<div class="check-item">☐ {rstr}</div>', unsafe_allow_html=True)

        # Raw JSON in expander
        with st.expander("Raw decision analysis JSON"):
            st.json(da)
        with st.expander("Spine case (constraints & uncertainties)"):
            st.json(case)
        if not out.get("spine_available"):
            st.caption("Spine runtime not installed; showing stub analysis.")

st.markdown("---")
with st.expander("Sample MJCF reference"):
    for label, (fname, _, _, _) in SAMPLES.items():
        path = _spine_bridge / fname
        if path.exists():
            st.markdown(f"**{label}** (`{fname}`)")
            st.code(path.read_text(encoding="utf-8"), language="xml")
