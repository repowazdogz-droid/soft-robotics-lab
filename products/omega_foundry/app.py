"""
Omega Foundry - Design from natural language intent.
Streamlit app: 3D preview, voice-to-design, templates, similar designs, history, Reality Bridge validation, export.
"""

import os
import sys
import requests
from pathlib import Path

# Reality Bridge URL (env for cross-machine; default local 18000)
REALITY_BRIDGE_URL = os.environ.get("REALITY_BRIDGE_URL", "http://localhost:18000")

_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))
_products = _root.parent
if str(_products) not in sys.path:
    sys.path.insert(0, str(_products))

import streamlit as st
from core.intent_parser import IntentParser, DesignSpec
from core.design_engine import DesignEngine, GeneratedDesign
from core.validator import PhysicsValidator, ValidationResult
from core.exporter import DesignExporter
from core.preview import mjcf_to_geometry, generate_preview_html
from core.template_loader import list_templates, load_template
from core.voice_design import voice_to_intent, intent_to_design
from core.history import save_version, list_versions, restore_version
from core.constraint_solver import (
    ConstraintSolver, ConstraintSet, ConstraintType, Constraint
)
from core.design_evolver import DesignEvolver, EvolutionResult

try:
    from substrate_integration import record_design_to_substrate, find_similar_designs, get_design_lineage
except Exception:
    record_design_to_substrate = lambda *a, **k: None
    find_similar_designs = lambda q, n=3: []
    get_design_lineage = lambda i: []

st.set_page_config(
    page_title="Omega Foundry",
    page_icon="⚙",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
    .main-header { font-size: 2rem; font-weight: 700; color: #00d4ff; margin-bottom: 0; }
    .sub-header { font-size: 1rem; color: #888; margin-top: 0.25rem; }
    .spec-box { background: #1a1a2e; padding: 1rem; border-radius: 8px; margin: 0.5rem 0; }
</style>
""", unsafe_allow_html=True)

# State
if "parsed_spec" not in st.session_state:
    st.session_state.parsed_spec = None
if "generated_design" not in st.session_state:
    st.session_state.generated_design = None
if "validation_result" not in st.session_state:
    st.session_state.validation_result = None
if "reality_bridge_result" not in st.session_state:
    st.session_state.reality_bridge_result = None
if "export_paths" not in st.session_state:
    st.session_state.export_paths = {}
if "export_dir" not in st.session_state:
    st.session_state.export_dir = ""
if "active_tab" not in st.session_state:
    st.session_state.active_tab = "scratch"
if "selected_template" not in st.session_state:
    st.session_state.selected_template = None
if "template_params" not in st.session_state:
    st.session_state.template_params = {}

# Header: OMEGA FOUNDRY + Voice
col_title, col_voice = st.columns([4, 1])
with col_title:
    st.title("OMEGA Foundry")
    st.caption("Design from natural language — 3D preview, physics-validated, fabrication-ready.")
with col_voice:
    st.markdown("<br>", unsafe_allow_html=True)
    voice_btn = st.button("🎤 Voice", help="Upload audio to describe your design")

# Voice flow
if voice_btn:
    st.session_state.show_voice_upload = True
if st.session_state.get("show_voice_upload"):
    audio_file = st.file_uploader("Upload voice recording (WAV/MP3)", type=["wav", "mp3"], key="voice_upload")
    if audio_file:
        voice_dir = Path("outputs") / "_voice_temp"
        voice_dir.mkdir(parents=True, exist_ok=True)
        path = voice_dir / (audio_file.name or "voice.wav")
        path.write_bytes(audio_file.getvalue())
        with st.spinner("Transcribing and parsing intent…"):
            intent_dict = voice_to_intent(str(path))
        if intent_dict.get("error"):
            st.warning(intent_dict.get("error", "") + " Using rule-based parse.")
            spec = parser.parse(intent_dict.get("raw_intent", ""))
            st.session_state.parsed_spec = spec
        else:
            with st.spinner("Generating design…"):
                spec = DesignSpec(
                    domain=intent_dict.get("domain", "gripper"),
                    scale=intent_dict.get("scale", "medium"),
                    material=intent_dict.get("material"),
                    params=intent_dict.get("params", {}),
                    target_object=intent_dict.get("target_object"),
                    raw_intent=intent_dict.get("raw_intent", ""),
                )
                design = engine.generate(spec)
                st.session_state.generated_design = design
                st.session_state.parsed_spec = spec
                st.success("Design generated from voice.")
        st.session_state.show_voice_upload = False
        st.rerun()
    if st.button("Cancel voice"):
        st.session_state.show_voice_upload = False
        st.rerun()

# Mode: Design from scratch | Use template | Similar designs
st.markdown("---")
mode = st.radio(
    "Mode",
    ["Design from scratch", "Use template", "Similar designs"],
    horizontal=True,
    key="mode_radio",
)

parser = IntentParser()
engine = DesignEngine()
validator = PhysicsValidator()
exporter = DesignExporter()

# Similar designs (search before generating)
if mode == "Similar designs":
    similar_query = st.text_input("Describe what you need", placeholder="e.g. two-finger gripper for eggs", key="similar_q")
    similar = find_similar_designs(similar_query, n=5) if similar_query else []
    if similar:
        st.markdown("**Similar designs** (click to use as reference)")
        for hit in similar:
            meta = hit.get("metadata", {})
            design_id = meta.get("design_id", hit.get("id", ""))
            st.caption(f"ID: {design_id} — score: {hit.get('score', 0):.2f}")
        st.info("Enter intent below to generate a new design inspired by these.")
    else:
        st.caption("No similar designs found. Generate a new design to populate the library.")

# Template browser
if mode == "Use template":
    templates = list_templates()
    categories = ["grippers", "mechanisms", "enclosures"]
    cat = st.selectbox("Category", categories, key="template_cat")
    subset = [t for t in templates if t.get("category") == cat]
    if subset:
        names = [t.get("name", "") for t in subset]
        sel_name = st.selectbox("Template", names, key="template_sel")
        t = next((x for x in subset if x.get("name") == sel_name), None)
        if t:
            st.caption(t.get("description", ""))
            params_schema = t.get("parameters", {})
            template_params = {}
            for pname, pdef in params_schema.items():
                default = pdef.get("default")
                opts = pdef.get("options")
                if isinstance(default, bool):
                    template_params[pname] = st.checkbox(pname, value=default, key=f"tp_{pname}")
                elif isinstance(default, int) and not opts:
                    template_params[pname] = st.number_input(pname, value=default, key=f"tp_{pname}")
                elif isinstance(opts, list):
                    template_params[pname] = st.selectbox(pname, opts, key=f"tp_{pname}")
                else:
                    template_params[pname] = st.text_input(pname, value=str(default) if default else "", key=f"tp_{pname}")
            st.session_state.template_params = template_params
            st.session_state.selected_template = load_template(cat, sel_name) or t
            if st.button("Generate from template"):
                template = st.session_state.selected_template
                if template:
                    spec_defaults = template.get("spec_defaults") or {}
                    params = {**(spec_defaults.get("params") or {}), **template_params}
                    spec = DesignSpec(
                        domain=spec_defaults.get("domain", "gripper"),
                        scale=spec_defaults.get("scale", "medium"),
                        material=spec_defaults.get("material"),
                        params=params,
                        target_object=spec_defaults.get("target_object"),
                        raw_intent=template.get("description", ""),
                    )
                    with st.spinner("Instantiating template…"):
                        design = engine.generate(spec)
                    st.session_state.generated_design = design
                    st.session_state.parsed_spec = spec
                    record_design_to_substrate(design.id, design.mjcf_xml or design.urdf_xml or "", metadata={"name": design.name, "domain": design.domain}, derived_from_id=template.get("name"), derived_from_type="template")
                    if design.mjcf_xml:
                        save_version(design.id, design.name, design.mjcf_xml, params)
                    st.success(f"Generated: {design.name}")
                    st.rerun()

# Intent input (scratch or after similar)
intent = st.text_input(
    "Describe your design",
    placeholder="e.g. two-finger pinch gripper for eggs, waterproof enclosure 10 by 15 cm",
    key="intent_input",
)

col_parse, col_gen = st.columns(2)
with col_parse:
    parse_btn = st.button("Parse intent", type="primary", use_container_width=True)
with col_gen:
    gen_btn = st.button("Generate design", use_container_width=True)

if parse_btn or (gen_btn and not st.session_state.parsed_spec):
    if intent.strip():
        spec = parser.parse_enhanced(intent, use_llm=True)
        st.session_state.parsed_spec = spec
        st.session_state.generated_design = None
        st.session_state.validation_result = None
        st.session_state.reality_bridge_result = None
    else:
        st.warning("Enter an intent first.")

if st.session_state.parsed_spec:
    spec = st.session_state.parsed_spec
    with st.expander("Parsed design spec", expanded=True):
        st.markdown(f"**Domain:** `{spec.domain}` · **Scale:** `{spec.scale}` · **Material:** `{spec.material or 'default'}` · **Target:** `{spec.target_object or '—'}`")
        st.json(spec.to_dict())

if gen_btn and st.session_state.parsed_spec:
    spec = st.session_state.parsed_spec
    with st.spinner("Generating design…"):
        design = engine.generate(spec)
        st.session_state.generated_design = design
        record_design_to_substrate(design.id, design.mjcf_xml or design.urdf_xml or "", metadata={"name": design.name, "domain": design.domain})
        if design.mjcf_xml:
            save_version(design.id, design.name, design.mjcf_xml, design.spec.params if design.spec else None)
        if design.errors:
            st.error("Generation had issues: " + "; ".join(design.errors))
        else:
            st.success(f"Generated: **{design.name}** ({design.id})")

# Layout: Intent/Params left, 3D Preview right
if st.session_state.generated_design:
    design = st.session_state.generated_design
    st.markdown("---")
    st.subheader(f"Generated Design: {design.id}")

    col_left, col_right = st.columns([1, 1])
    with col_left:
        st.markdown("**Parameters**")
        st.json(design.design_dict)
    with col_right:
        st.markdown("**3D Preview**")
        xml_content = design.mjcf_xml or design.urdf_xml or ""
        if xml_content and ("<mujoco" in xml_content or "<robot" in xml_content):
            geoms = mjcf_to_geometry(xml_content) if "<mujoco" in xml_content else []
            if geoms:
                html = generate_preview_html(geoms, height=400)
                st.components.v1.html(html, height=400, scrolling=False)
            else:
                st.caption("Preview available for MJCF with geometry. Export to view URDF.")
        else:
            st.caption("No MJCF/URDF to preview.")

    # Tabs: MJCF | Parameters | History | Export
    tab_mjcf, tab_params, tab_history, tab_export = st.tabs(["MJCF", "Parameters", "History", "Export"])
    with tab_mjcf:
        raw = design.mjcf_xml or design.urdf_xml or ""
        st.text_area("Model (MJCF/URDF)", value=raw, height=300, key="mjcf_display")
    with tab_params:
        st.json(design.to_dict())
    with tab_history:
        versions = list_versions(design.id)
        if versions:
            for v in versions:
                st.caption(f"v{v['version']} — {v.get('created_at', '')} — {v.get('notes', '')}")
            restore_ver = st.number_input("Restore version", min_value=1, max_value=max((x["version"] for x in versions), default=1), key="restore_ver")
            if st.button("Restore this version"):
                mjcf_restore = restore_version(design.id, restore_ver)
                if mjcf_restore:
                    st.session_state.generated_design = GeneratedDesign(
                        id=design.id, name=design.name, domain=design.domain, spec=design.spec,
                        design_dict=design.design_dict, mjcf_xml=mjcf_restore, urdf_xml=design.urdf_xml, mesh_path=design.mesh_path, errors=[],
                    )
                    st.success("Loaded version into editor. Re-export to save.")
                    st.rerun()
        else:
            st.caption("No version history yet. Save/export to record versions.")
    with tab_export:
        design_id_safe = design.id.replace("-", "_").replace(" ", "_")
        export_dir = Path("outputs") / design_id_safe
        export_btn = st.button("Export all (MJCF, URDF, STL, JSON)")
        if export_btn:
            export_dir.mkdir(parents=True, exist_ok=True)
            paths = exporter.export(design, str(export_dir), base_name="design")
            st.session_state.export_paths = paths
            st.session_state.export_dir = str(export_dir)
            st.success(f"Exported to outputs/{design_id_safe}/")
        if st.session_state.get("export_paths"):
            paths = st.session_state.export_paths
            for fmt in ["mjcf", "urdf", "stl", "json"]:
                if fmt in paths:
                    p = Path(paths[fmt])
                    if p.exists():
                        st.download_button(f"Download {fmt.upper()}", data=p.read_bytes(), file_name=p.name, mime="application/octet-stream" if fmt != "json" else "application/json", key=f"dl_{fmt}")

    # Validate: MuJoCo local + Reality Bridge
    st.markdown("---")
    st.subheader("Validation")
    col_mjcf, col_rb = st.columns(2)
    with col_mjcf:
        if st.button("Validate (MuJoCo local)"):
            if design.mjcf_xml:
                res = validator.validate_mjcf_string(design.mjcf_xml)
                st.session_state.validation_result = res
    with col_rb:
        if st.button("Validate with Reality Bridge"):
            xml_to_send = design.mjcf_xml or design.urdf_xml or ""
            if not xml_to_send:
                st.warning("No MJCF/URDF to validate.")
            else:
                try:
                    r = requests.post(
                        f"{REALITY_BRIDGE_URL.rstrip('/')}/validate",
                        data={"xml_string": xml_to_send, "artifact_id": design.id},
                        timeout=10,
                    )
                    if r.status_code == 200:
                        data = r.json()
                        st.session_state.reality_bridge_result = data
                    else:
                        st.session_state.reality_bridge_result = {"success": False, "message": r.text or f"HTTP {r.status_code}"}
                except Exception as e:
                    st.session_state.reality_bridge_result = {"success": False, "message": str(e)}
                    st.warning("Reality Bridge not reachable. Set REALITY_BRIDGE_URL or start: cd products/reality_bridge && uvicorn app:app --reload --port 18000")

    if st.session_state.validation_result:
        res = st.session_state.validation_result
        st.markdown("**MuJoCo:** " + ("Pass" if res.valid else "Fail") + (" — " + "; ".join(res.errors) if res.errors else ""))
    if st.session_state.reality_bridge_result:
        rb = st.session_state.reality_bridge_result
        passed = rb.get("passed", rb.get("success", False))
        score = rb.get("score")
        st.markdown("**Reality Bridge:** " + ("Pass" if passed else "Fail") + (f" (score: {score})" if score is not None else ""))
        if not passed:
            st.caption("Learn why: use OMEGA Tutor to study validation and physics.")
        if rb.get("message"):
            st.caption(rb["message"])

# Constraint-Based Design
st.divider()
st.header("🔒 Constraint-Based Design")

solver = ConstraintSolver()

st.subheader("Define Constraints")

constraint_set = ConstraintSet(name="user_constraints")

col1, col2 = st.columns(2)

with col1:
    st.write("**Size Constraints**")
    max_length = st.number_input("Max finger length (mm)", 10, 300, 100, key="c_max_length")
    constraint_set.add_max("Max Length", "finger_length", max_length / 1000, "m")

    max_width = st.number_input("Max palm size (mm)", 20, 200, 80, key="c_max_palm")
    constraint_set.add_max("Max Palm", "palm_size", max_width / 1000, "m")

    max_mass = st.number_input("Max mass (g)", 10, 2000, 200, key="c_max_mass")
    constraint_set.add_max("Max Mass", "mass", max_mass / 1000, "kg")

with col2:
    st.write("**Performance Constraints**")
    min_force = st.number_input("Min grip force (N)", 0.5, 100.0, 5.0, key="c_min_force")
    constraint_set.add_min("Min Force", "max_force", min_force, "N")

    num_fingers = st.slider("Number of fingers", 2, 6, 3, key="c_fingers")
    constraint_set.add(Constraint("Fingers", "finger_count", ConstraintType.EXACT, num_fingers))

st.write("**Environment Constraints**")
env_col1, env_col2 = st.columns(2)
with env_col1:
    food_safe = st.checkbox("Food safe required", key="c_food_safe")
    if food_safe:
        constraint_set.add_bool("Food Safe", "food_safe", True)
with env_col2:
    sterilizable = st.checkbox("Sterilizable required", key="c_sterile")
    if sterilizable:
        constraint_set.add_bool("Sterilizable", "sterilizable", True)

if st.button("Check Feasibility", key="check_feasibility"):
    impossibility = solver.check_impossibility(constraint_set)

    if impossibility.is_impossible:
        st.error("❌ **Constraints are impossible to satisfy**")
        for conflict in impossibility.conflicts:
            st.write(f"- {conflict.get('explanation', str(conflict))}")
        st.subheader("Suggestions")
        for suggestion in impossibility.suggestions:
            st.info(suggestion)
    else:
        st.success("✅ **Constraints are feasible**")

        design, result = solver.solve_for_feasible_design(constraint_set)

        if design:
            st.subheader("Feasible Design Parameters")
            for param, value in design.items():
                if isinstance(value, float):
                    st.write(f"- **{param}**: {value:.4f}")
                else:
                    st.write(f"- **{param}**: {value}")

            st.session_state["constrained_design"] = design

        st.subheader("Feasible Ranges")
        for param, range_val in result.feasible_ranges.items():
            if range_val:
                low, high = range_val
                st.write(f"- **{param}**: [{low:.4f}, {high:.4f}]")

        st.subheader("Recommended Materials")
        materials = solver.suggest_material(constraint_set)
        for name, score in materials[:3]:
            st.write(f"- **{name}**: {score:.0%} compatibility")

# Design Evolution
st.divider()
st.header("🧬 Design Evolution")

evolver = DesignEvolver()

evo_col1, evo_col2 = st.columns(2)

with evo_col1:
    design_name = st.text_input("Design name", placeholder="e.g., egg_gripper", key="evo_name")

with evo_col2:
    if design_name:
        summary = evolver.get_evolution_summary(design_name)
        if summary["versions"] > 0:
            st.metric("Versions", summary["versions"])
            st.write(f"Passed: {summary['passed']} | Failed: {summary['failed']}")

if design_name:
    lineage = evolver.get_design_lineage(design_name)

    if lineage:
        st.subheader("Design Lineage")
        for v in lineage:
            status = "✅" if v.validation_passed else "❌" if v.validation_result else "⏳"
            with st.expander(f"{status} {v.version_id} ({v.created_at[:10]})"):
                st.write(f"**Parent:** {v.parent_id or 'None (initial)'}")

                if v.changes_from_parent:
                    st.write("**Changes:**")
                    for change in v.changes_from_parent:
                        st.write(f"- {change['parameter']}: {change['old_value']:.4f} → {change['new_value']:.4f}")
                        st.caption(change.get("reasoning", ""))

                if v.validation_result:
                    st.write(f"**Validation:** {'Passed' if v.validation_passed else 'Failed'}")

    st.subheader("Actions")

    action = st.radio("Action", ["Create Initial", "Evolve from Failure"], key="evo_action")

    if action == "Create Initial":
        if "constrained_design" in st.session_state:
            st.write("Using design from constraint solver")
            init_params = st.session_state["constrained_design"]
        else:
            st.write("Enter initial parameters:")
            init_params = {
                "finger_length": st.number_input("Finger length (m)", 0.01, 0.3, 0.08, key="init_fl"),
                "finger_width": st.number_input("Finger width (m)", 0.005, 0.05, 0.015, key="init_fw"),
                "finger_count": st.number_input("Finger count", 2, 8, 3, key="init_fc"),
                "palm_size": st.number_input("Palm size (m)", 0.02, 0.2, 0.06, key="init_ps"),
                "actuator_force": st.number_input("Actuator force (N)", 1.0, 50.0, 10.0, key="init_af"),
                "damping": st.number_input("Damping", 0.01, 1.0, 0.1, key="init_d"),
            }

        if st.button("Create Initial Version", key="create_init"):
            version = evolver.create_initial_version(design_name, init_params)
            st.success(f"Created version: {version.version_id}")
            st.rerun()

    else:
        if lineage:
            latest = lineage[-1]
            st.write(f"Latest version: **{latest.version_id}**")

            failure_messages = st.text_area(
                "Paste failure messages from Reality Bridge",
                placeholder="e.g., 'stability test failed: tip over detected'\n'grip force insufficient'",
                key="failure_msgs"
            )

            if st.button("Evolve Design", key="evolve"):
                if not failure_messages:
                    st.warning("Please enter failure messages")
                else:
                    messages = [m.strip() for m in failure_messages.split("\n") if m.strip()]
                    new_version, result = evolver.evolve_from_failure(
                        design_name,
                        latest.version_id,
                        messages
                    )

                    if new_version:
                        st.success(f"Created evolved version: {new_version.version_id}")

                        st.subheader("Changes Made")
                        for change in result.changes_made:
                            st.write(f"- **{change['parameter']}**: {change['old_value']:.4f} → {change['new_value']:.4f}")
                            st.caption(change["reasoning"])

                        st.info(f"Expected improvement: {result.expected_improvement}")
                        st.rerun()
                    else:
                        st.warning(result.expected_improvement)
        else:
            st.info("No versions yet. Create an initial version first.")

clear_btn = st.sidebar.button("Clear / New Design")
if clear_btn:
    st.session_state.clear()
    st.rerun()
