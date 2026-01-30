"""
Omega Foundry - Design from natural language intent.
Streamlit app: intent input, parsed spec, generate, validate, export.
"""

import sys
from pathlib import Path

# Ensure core is importable when running from omega_foundry/
_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

import streamlit as st
from core.intent_parser import IntentParser, DesignSpec
from core.design_engine import DesignEngine, GeneratedDesign
from core.validator import PhysicsValidator, ValidationResult
from core.exporter import DesignExporter

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

st.title("Omega Foundry")
st.caption("Design from natural language — physics-validated, fabrication-ready.")

clear_btn = st.button("Clear / New Design", use_container_width=False)
if clear_btn:
    st.session_state.clear()
    st.rerun()

# State
if "parsed_spec" not in st.session_state:
    st.session_state.parsed_spec = None
if "generated_design" not in st.session_state:
    st.session_state.generated_design = None
if "validation_result" not in st.session_state:
    st.session_state.validation_result = None
if "export_paths" not in st.session_state:
    st.session_state.export_paths = {}
if "export_dir" not in st.session_state:
    st.session_state.export_dir = ""

# Intent input
intent = st.text_input(
    "Describe your design",
    placeholder="e.g. small pinch gripper for eggs",
    key="intent_input",
)

col_parse, col_gen = st.columns(2)
with col_parse:
    parse_btn = st.button("Parse intent", type="primary", use_container_width=True)
with col_gen:
    gen_btn = st.button("Generate design", use_container_width=True)

parser = IntentParser()
engine = DesignEngine()
validator = PhysicsValidator()
exporter = DesignExporter()

# Parse
if parse_btn or (gen_btn and not st.session_state.parsed_spec):
    if intent.strip():
        spec = parser.parse(intent)
        st.session_state.parsed_spec = spec
        st.session_state.generated_design = None
        st.session_state.validation_result = None
    else:
        st.warning("Enter an intent first.")

# Show parsed spec
if st.session_state.parsed_spec:
    spec = st.session_state.parsed_spec
    with st.expander("Parsed design spec", expanded=True):
        st.markdown(f"""
        **Domain:** `{spec.domain}` · **Scale:** `{spec.scale}` · **Material:** `{spec.material or "default"}`  
        **Target:** `{spec.target_object or "—"}` · **Params:** `{spec.params}`
        """)
        st.json(spec.to_dict())

# Generate
if gen_btn and st.session_state.parsed_spec:
    spec = st.session_state.parsed_spec
    with st.spinner("Generating design…"):
        design = engine.generate(spec)
        st.session_state.generated_design = design
        if design.errors:
            st.error("Generation had issues: " + "; ".join(design.errors))
        else:
            st.success(f"Generated: **{design.name}** ({design.id})")

# Show generated design + validate
if st.session_state.generated_design:
    design = st.session_state.generated_design
    st.markdown("---")
    st.subheader("Generated design")
    st.json(design.design_dict)

    validate_btn = st.button("Validate (MuJoCo)")
    if validate_btn and design.mjcf_xml:
        res = validator.validate_mjcf_string(design.mjcf_xml)
        st.session_state.validation_result = res
    if st.session_state.validation_result:
        res = st.session_state.validation_result
        st.markdown("#### Validation feedback")
        load_ok = "Pass" if res.loads else "Fail"
        sim_ok = "Pass" if res.simulates else "Fail"
        st.markdown(f"**MuJoCo load:** {load_ok} · **Simulation:** {sim_ok}")
        s = res.stats
        dofs = s.get("dofs", s.get("joints", "—"))
        nu = s.get("actuators", "—")
        ns = s.get("sensors", "—")
        mass = s.get("mass_kg")
        mass_str = f"{mass:.4f} kg" if mass is not None else "N/A"
        st.markdown(f"**DOFs:** {dofs} · **Actuators:** {nu} · **Sensors:** {ns} · **Est. mass:** {mass_str}")
        if res.valid:
            st.success("Validation passed.")
        else:
            st.error("Validation failed: " + "; ".join(res.errors))
        if res.warnings:
            st.warning("Warnings: " + "; ".join(res.warnings))

    # Export
    st.markdown("---")
    st.subheader("Export")
    design_id_safe = design.id.replace("-", "_").replace(" ", "_")
    export_dir = Path("outputs") / design_id_safe
    export_btn = st.button("Export all (MJCF, URDF, STL, JSON)")
    if export_btn:
        export_dir.mkdir(parents=True, exist_ok=True)
        paths = exporter.export(design, str(export_dir), base_name="design")
        st.session_state.export_paths = paths
        st.session_state.export_dir = str(export_dir)
        file_list = ", ".join(f"design.{fmt}" for fmt in paths.keys())
        st.success(f"Exported to outputs/{design_id_safe}/ — Files: {file_list}")

    if st.session_state.get("export_paths"):
        paths = st.session_state.export_paths
        export_dir_str = st.session_state.get("export_dir", "")
        if export_dir_str:
            st.caption(f"Folder: `{export_dir_str}`")
        c1, c2, c3, c4 = st.columns(4)
        for col, fmt in [(c1, "mjcf"), (c2, "urdf"), (c3, "stl"), (c4, "json")]:
            with col:
                if fmt in paths:
                    p = Path(paths[fmt])
                    if p.exists():
                        data = p.read_bytes()
                        st.download_button(
                            f"Download {fmt.upper()}",
                            data=data,
                            file_name=p.name,
                            mime="application/octet-stream" if fmt != "json" else "application/json",
                            key=f"dl_{fmt}",
                        )
