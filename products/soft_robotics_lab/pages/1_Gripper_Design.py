"""
Gripper Design Page - World Class Edition
"""

import streamlit as st
import sys
import json
import numpy as np
import plotly.graph_objects as go
from pathlib import Path

_soft_lab = Path(__file__).resolve().parent.parent
_workbench = _soft_lab / "workbench"
if str(_soft_lab) not in sys.path:
    sys.path.insert(0, str(_soft_lab))
if str(_workbench) not in sys.path:
    sys.path.insert(0, str(_workbench))

from workbench.motion_to_morphology import (
    MotionToMorphology,
    export_design,
    GripperDesign,
)

try:
    from gripper_cad import export_gripper_stl, gripper_from_design
    CAD_AVAILABLE = True
except ImportError:
    CAD_AVAILABLE = False

try:
    from workbench.mujoco_validator import MuJoCoValidator
    MUJOCO_AVAILABLE = True
except Exception:
    MUJOCO_AVAILABLE = False

from utils.lab_os_client import (
    health_check,
    get_hypotheses,
    create_hypothesis,
    get_hypothesis_evidence,
)

st.set_page_config(
    page_title="Gripper Design", page_icon="🎯", layout="wide"
)

# Custom CSS
st.markdown("""
<style>
    .design-card {
        background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
        border-radius: 15px;
        padding: 25px;
        border: 1px solid #333;
        margin: 10px 0;
    }
    .metric-highlight {
        font-size: 2.5rem;
        font-weight: 700;
        background: linear-gradient(90deg, #00d4ff, #00ff88);
        -webkit-background-clip: text;
        -webkit-text-fill-color: transparent;
    }
    .confidence-bar {
        height: 8px;
        border-radius: 4px;
        background: #333;
        overflow: hidden;
    }
    .confidence-fill {
        height: 100%;
        border-radius: 4px;
        transition: width 0.5s ease;
    }
</style>
""", unsafe_allow_html=True)

st.markdown("# 🎯 Gripper Design Studio")
st.markdown(
    "*Transform human grasp gestures into optimized soft gripper morphologies*"
)

st.markdown("---")


@st.cache_resource
def get_m2m():
    return MotionToMorphology()


m2m = get_m2m()


def create_3d_gripper(design: GripperDesign) -> go.Figure:
    """Create interactive 3D visualization of gripper."""
    fig = go.Figure()

    palm_color = "#4a5568"
    finger_colors = [
        "#00d4ff",
        "#00ff88",
        "#ffaa00",
        "#ff6b6b",
        "#a855f7",
    ]

    # Palm - cylinder surface
    theta = np.linspace(0, 2 * np.pi, 30)
    z_palm = np.linspace(0, design.palm_thickness_mm, 10)
    theta_grid, z_grid = np.meshgrid(theta, z_palm)

    x_palm = design.palm_radius_mm * np.cos(theta_grid)
    y_palm = design.palm_radius_mm * np.sin(theta_grid)

    fig.add_trace(
        go.Surface(
            x=x_palm,
            y=y_palm,
            z=z_grid,
            colorscale=[[0, palm_color], [1, palm_color]],
            showscale=False,
            name="Palm",
            opacity=0.9,
        )
    )

    # Palm top cap
    r_cap = np.linspace(0, design.palm_radius_mm, 10)
    theta_cap = np.linspace(0, 2 * np.pi, 30)
    r_grid, theta_grid = np.meshgrid(r_cap, theta_cap)

    x_cap = r_grid * np.cos(theta_grid)
    y_cap = r_grid * np.sin(theta_grid)
    z_cap = np.ones_like(x_cap) * design.palm_thickness_mm

    fig.add_trace(
        go.Surface(
            x=x_cap,
            y=y_cap,
            z=z_cap,
            colorscale=[[0, palm_color], [1, palm_color]],
            showscale=False,
            opacity=0.9,
        )
    )

    # Fingers
    num_fingers = design.num_fingers
    finger = design.finger_designs[0]
    segment_length = finger.length_mm / finger.num_segments

    for i in range(num_fingers):
        angle = 2 * np.pi * i / num_fingers
        color = finger_colors[i % len(finger_colors)]

        base_x = design.palm_radius_mm * 0.7 * np.cos(angle)
        base_y = design.palm_radius_mm * 0.7 * np.sin(angle)
        base_z = design.palm_thickness_mm

        current_x, current_y, current_z = base_x, base_y, base_z

        for seg in range(finger.num_segments):
            taper = 1 - (
                seg * (1 - finger.taper_ratio) / finger.num_segments
            )
            seg_radius = (finger.width_mm / 2) * taper
            bend = 0.15 * seg

            end_z = current_z + segment_length * np.cos(bend)
            radial_extension = segment_length * np.sin(bend)
            end_x = current_x + radial_extension * np.cos(angle)
            end_y = current_y + radial_extension * np.sin(angle)

            fig.add_trace(
                go.Scatter3d(
                    x=[current_x, end_x],
                    y=[current_y, end_y],
                    z=[current_z, end_z],
                    mode="lines",
                    line=dict(color=color, width=max(2, seg_radius)),
                    name=f"Finger {i+1}" if seg == 0 else None,
                    showlegend=(seg == 0),
                )
            )

            fig.add_trace(
                go.Scatter3d(
                    x=[end_x],
                    y=[end_y],
                    z=[end_z],
                    mode="markers",
                    marker=dict(
                        size=max(3, seg_radius * 1.5),
                        color=color,
                        opacity=0.8,
                    ),
                    showlegend=False,
                )
            )

            current_x, current_y, current_z = end_x, end_y, end_z

    max_dim = max(
        design.palm_radius_mm * 2,
        finger.length_mm + design.palm_thickness_mm,
    )

    fig.update_layout(
        scene=dict(
            xaxis=dict(
                range=[-max_dim, max_dim],
                showbackground=False,
                showgrid=False,
                zeroline=False,
                visible=False,
            ),
            yaxis=dict(
                range=[-max_dim, max_dim],
                showbackground=False,
                showgrid=False,
                zeroline=False,
                visible=False,
            ),
            zaxis=dict(
                range=[0, max_dim * 1.5],
                showbackground=False,
                showgrid=False,
                zeroline=False,
                visible=False,
            ),
            bgcolor="rgba(0,0,0,0)",
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2),
                up=dict(x=0, y=0, z=1),
            ),
        ),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        margin=dict(l=0, r=0, t=0, b=0),
        height=500,
        showlegend=True,
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01,
            bgcolor="rgba(26,26,46,0.8)",
            font=dict(color="white"),
        ),
    )

    return fig


def create_confidence_gauge(confidence: float) -> go.Figure:
    """Create a confidence gauge chart."""
    color = (
        "#00ff88"
        if confidence > 0.75
        else "#ffaa00"
        if confidence > 0.5
        else "#ff4444"
    )

    fig = go.Figure(
        go.Indicator(
            mode="gauge+number",
            value=confidence * 100,
            number={
                "suffix": "%",
                "font": {"size": 40, "color": "white"},
            },
            gauge={
                "axis": {
                    "range": [0, 100],
                    "tickcolor": "white",
                    "tickfont": {"color": "white"},
                },
                "bar": {"color": color},
                "bgcolor": "#333",
                "borderwidth": 0,
                "steps": [
                    {"range": [0, 50], "color": "rgba(255,68,68,0.2)"},
                    {"range": [50, 75], "color": "rgba(255,170,0,0.2)"},
                    {"range": [75, 100], "color": "rgba(0,255,136,0.2)"},
                ],
                "threshold": {
                    "line": {"color": "white", "width": 2},
                    "thickness": 0.75,
                    "value": confidence * 100,
                },
            },
        )
    )

    fig.update_layout(
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        font={"color": "white"},
        height=200,
        margin=dict(l=20, r=20, t=30, b=20),
    )

    return fig


# Main layout
col_input, col_output = st.columns([1, 2])

with col_input:
    st.markdown("### Design Parameters")

    with st.container():
        gesture = st.selectbox(
            "🤚 Gesture Type",
            [
                "pinch",
                "power",
                "wrap",
                "hook",
                "lateral",
                "squeeze",
                "spread",
            ],
            help="The type of human grasp to emulate",
        )

        st.markdown("---")

        aperture = st.slider(
            "📏 Grasp Aperture",
            min_value=20,
            max_value=150,
            value=50,
            help="Maximum opening width (mm)",
        )

        force = st.slider(
            "💪 Force Requirement",
            min_value=1.0,
            max_value=30.0,
            value=5.0,
            format="%.1f",
            help="Required grasp force (N)",
        )

        compliance = st.slider(
            "🧸 Object Compliance",
            min_value=0.0,
            max_value=1.0,
            value=0.5,
            format="%.2f",
            help="0 = rigid (metal), 1 = very soft (tissue)",
        )

        compliance_labels = [
            "Rigid",
            "Firm",
            "Medium",
            "Soft",
            "Very Soft",
        ]
        compliance_idx = min(int(compliance * 5), 4)
        st.caption(f"Object type: **{compliance_labels[compliance_idx]}**")

        st.markdown("---")

        environment = st.selectbox(
            "🌍 Environment",
            ["dry", "wet", "surgical"],
            help="Operating conditions",
        )

        st.markdown("---")

        generate_btn = st.button(
            "🚀 Generate Gripper",
            type="primary",
            use_container_width=True,
        )

with col_output:
    if generate_btn:
        with st.spinner("Generating optimized gripper design..."):
            design = m2m.from_gesture(
                gesture=gesture,
                aperture=aperture / 1000,
                force_requirement=force,
                object_compliance=compliance,
                environment=environment,
            )

        st.session_state["current_design"] = design
        if "exports" in st.session_state:
            del st.session_state["exports"]
        if "stl_path" in st.session_state:
            del st.session_state["stl_path"]
        if "mjcf_validation" in st.session_state:
            del st.session_state["mjcf_validation"]
        if "bundle_path" in st.session_state:
            del st.session_state["bundle_path"]

    if "current_design" in st.session_state:
        design = st.session_state["current_design"]

        st.markdown(
            f"""
        <div class="design-card">
            <h2 style="color: #00d4ff; margin: 0 0 10px 0;">{design.name}</h2>
            <p style="color: #888; margin: 0;">ID: {design.id} • Generated for {design.source_gesture.value} gesture</p>
        </div>
        """,
            unsafe_allow_html=True,
        )

        # Link to Hypothesis
        st.subheader("Link to Hypothesis")

        lab_os_online = health_check()

        if lab_os_online:
            st.success("🔗 Connected to OMEGA Lab OS")

            hypotheses = get_hypotheses()

            if hypotheses:
                hyp_options = {h["id"]: f"{h['id']}: {h['claim'][:50]}..." for h in hypotheses}
                selected_hyp = st.selectbox(
                    "Select hypothesis",
                    options=list(hyp_options.keys()),
                    format_func=lambda x: hyp_options[x],
                    key="design_link_hyp_lab",
                )

                if selected_hyp:
                    evidence = get_hypothesis_evidence(selected_hyp)
                    if evidence:
                        st.write(f"**Evidence count:** {len(evidence)}")
                        for e in evidence[-3:]:
                            direction_icon = (
                                "✅" if e["direction"] == "supports"
                                else "❌" if e["direction"] == "refutes"
                                else "❓"
                            )
                            st.write(f"{direction_icon} {e['rationale'][:50]}...")

            with st.expander("Create new hypothesis"):
                new_id = st.text_input(
                    "Hypothesis ID",
                    value=f"H-{st.session_state.get('design_id', design.id)}",
                    key="new_hyp_id",
                )
                new_claim = st.text_area("Claim", key="new_hyp_claim")
                if st.button("Create Hypothesis", key="create_hyp_btn") and new_claim:
                    result = create_hypothesis(new_id, new_claim)
                    if result:
                        st.success(f"Created: {new_id}")
                        st.rerun()
                    else:
                        st.error("Failed to create hypothesis")
        else:
            st.warning(
                "⚠️ OMEGA Lab OS not connected. Run: uvicorn app.main:app --port 8000",
            )
            st.info(
                "Hypothesis linking requires Lab OS. Using local Research Memory as fallback.",
            )

            # Fallback: local Research Memory
            _lab = st.session_state.get("lab_name", "demo_lab")
            _res_path = _soft_lab / "research_system"
            if str(_res_path) not in sys.path:
                sys.path.insert(0, str(_res_path))
            try:
                from research_memory import ResearchMemory
                _mem = ResearchMemory(_lab)
                _hyps = _mem.get_hypotheses()
                if _hyps:
                    _links_path = _mem.base_path / "design_hypothesis_links.json"
                    _links = json.loads(_links_path.read_text()) if _links_path.exists() else {}
                    _sel = st.selectbox(
                        "Link this design to a hypothesis (tests hypothesis X)",
                        ["(none)"] + [f"{h.id}: {h.claim[:50]}..." for h in _hyps],
                        key="design_link_hyp",
                    )
                    if _sel != "(none)" and st.button("Save link", key="design_save_link"):
                        _hyp_id = _hyps[[f"{h.id}: {h.claim[:50]}..." for h in _hyps].index(_sel)].id
                        _links[design.id] = _hyp_id
                        _mem.base_path.mkdir(parents=True, exist_ok=True)
                        _links_path.write_text(json.dumps(_links, indent=2))
                        st.success("Design linked to hypothesis.")
                        st.rerun()
                    if design.id in _links:
                        st.caption(f"Linked to hypothesis: {_links[design.id]}")
            except Exception:
                pass

        col_3d, col_conf = st.columns([2, 1])

        with col_3d:
            st.markdown("#### 3D Preview")
            fig_3d = create_3d_gripper(design)
            st.plotly_chart(
                fig_3d,
                use_container_width=True,
                config={"displayModeBar": False},
            )

        with col_conf:
            st.markdown("#### Confidence")
            fig_gauge = create_confidence_gauge(design.confidence)
            st.plotly_chart(
                fig_gauge,
                use_container_width=True,
                config={"displayModeBar": False},
            )

            st.markdown("**Factors:**")
            for factor in design.uncertainty_factors:
                if "+" in factor:
                    st.success(factor, icon="✅")
                else:
                    st.warning(factor, icon="⚠️")

        st.markdown("---")
        st.markdown("### Specifications")

        spec_col1, spec_col2, spec_col3, spec_col4 = st.columns(4)

        with spec_col1:
            st.metric("Fingers", design.num_fingers)
        with spec_col2:
            st.metric("Actuator", design.primary_actuator.value.title())
        with spec_col3:
            st.metric("Max Force", f"{design.max_force_n:.1f}N")
        with spec_col4:
            st.metric(
                "Speed",
                f"{design.grasp_speed_mm_per_s:.0f}mm/s",
            )

        finger = design.finger_designs[0]

        spec_col5, spec_col6, spec_col7, spec_col8 = st.columns(4)

        with spec_col5:
            st.metric("Finger Length", f"{finger.length_mm:.0f}mm")
        with spec_col6:
            st.metric("Segments", finger.num_segments)
        with spec_col7:
            st.metric(
                "Material",
                finger.material.value.split("_")[0].title(),
            )
        with spec_col8:
            st.metric("Aperture", f"{design.max_aperture_mm:.0f}mm")

        st.markdown("---")
        st.markdown("### Assumptions / Operating Envelope")
        with st.expander("Show assumptions and operating envelope", expanded=False):
            st.markdown(
                "**Confidence is rule-based** (not from physical simulation or experiments). "
                "Key assumptions for this design:"
            )
            st.markdown(
                f"- **Gesture:** {design.source_gesture.value}; **Environment:** {design.source_parameters.get('environment', 'dry')}; "
                f"**Object compliance:** {design.source_parameters.get('object_compliance', 0.5):.2f}"
            )
            st.markdown(
                "- Material and actuator mappings are heuristic; no real physics validation."
            )
            st.markdown(
                "- Failure modes and confidence are from rule-based predictors, not MuJoCo or hardware tests."
            )
            st.markdown(
                "Validate exported MJCF in MuJoCo (button below) for simulation sanity; prototype testing required for real performance."
            )

        st.markdown("---")
        st.markdown("### Top Failure Risks")

        n_failure = len(design.failure_modes[:3])
        n_cols = max(1, n_failure)
        failure_cols = st.columns(n_cols)

        for i, fm in enumerate(design.failure_modes[:3]):
            with failure_cols[i]:
                prob = fm["probability"]
                color = (
                    "#ff4444"
                    if prob > 0.3
                    else "#ffaa00"
                    if prob > 0.15
                    else "#00ff88"
                )
                st.markdown(
                    f"""
                <div style="background: #252540; padding: 15px; border-radius: 10px; text-align: center; border-top: 3px solid {color};">
                    <p style="color: {color}; font-size: 1.5rem; font-weight: bold; margin: 0;">{prob*100:.0f}%</p>
                    <p style="color: #fff; margin: 5px 0; font-size: 0.85rem;">{fm['mode']}</p>
                </div>
                """,
                    unsafe_allow_html=True,
                )

        st.markdown("---")
        st.markdown("### 📦 Export Design")

        output_dir = (
            Path(__file__).resolve().parent.parent
            / "outputs"
            / design.id.lower().replace("-", "_")
        )

        export_col1, export_col2, export_col3 = st.columns(3)

        with export_col1:
            if st.button("💾 Export Simulation Files", type="secondary", key="export_sim_btn"):
                with st.spinner("Exporting MJCF/URDF/USD..."):
                    output_dir.mkdir(parents=True, exist_ok=True)
                    exports = export_design(design, str(output_dir))
                st.session_state["exports"] = exports
                st.success(f"✓ Exported to {output_dir}")
                st.rerun()

        with export_col2:
            if CAD_AVAILABLE:
                if st.button("🖨️ Generate 3D Print STL", type="primary", key="export_stl_btn"):
                    with st.spinner("Generating 3D geometry..."):
                        output_dir.mkdir(parents=True, exist_ok=True)
                        stl_path = output_dir / f"{design.id.lower().replace('-', '_')}.stl"
                        export_gripper_stl(design.to_dict(), str(stl_path))
                    st.success("✓ STL ready!")
                    st.session_state["stl_path"] = str(stl_path)
                    st.rerun()
            else:
                st.warning("CAD module not available")

        with export_col3:
            if "stl_path" in st.session_state and Path(st.session_state["stl_path"]).exists():
                stl_data = Path(st.session_state["stl_path"]).read_bytes()
                st.download_button(
                    "⬇️ Download STL",
                    stl_data,
                    file_name=f"{design.id}.stl",
                    mime="application/octet-stream",
                    key="dl_stl_btn",
                )

        # 3D STL Viewer
        if "stl_path" in st.session_state and Path(st.session_state["stl_path"]).exists():
            st.markdown("---")
            st.markdown("### 🖼️ 3D Preview (Printable Geometry)")

            try:
                import trimesh
                mesh = trimesh.load(st.session_state["stl_path"])

                vertices = mesh.vertices
                faces = mesh.faces

                fig = go.Figure(
                    data=[
                        go.Mesh3d(
                            x=vertices[:, 0],
                            y=vertices[:, 1],
                            z=vertices[:, 2],
                            i=faces[:, 0],
                            j=faces[:, 1],
                            k=faces[:, 2],
                            color="#00d4ff",
                            opacity=0.9,
                            flatshading=True,
                            lighting=dict(
                                ambient=0.5,
                                diffuse=0.8,
                                specular=0.3,
                                roughness=0.5,
                            ),
                            lightposition=dict(x=100, y=100, z=200),
                        )
                    ]
                )

                fig.update_layout(
                    scene=dict(
                        xaxis=dict(visible=False, showbackground=False),
                        yaxis=dict(visible=False, showbackground=False),
                        zaxis=dict(visible=False, showbackground=False),
                        bgcolor="rgba(0,0,0,0)",
                        camera=dict(
                            eye=dict(x=1.5, y=1.5, z=1.0),
                            up=dict(x=0, y=0, z=1),
                        ),
                        aspectmode="data",
                    ),
                    paper_bgcolor="rgba(26,26,46,0.8)",
                    margin=dict(l=0, r=0, t=0, b=0),
                    height=500,
                )

                st.plotly_chart(
                    fig,
                    use_container_width=True,
                    config={
                        "displayModeBar": True,
                        "modeBarButtonsToRemove": ["pan2d", "select2d", "lasso2d"],
                        "displaylogo": False,
                    },
                )

                st.caption(
                    f"📊 Mesh: {len(mesh.vertices):,} vertices, {len(mesh.faces):,} faces"
                )

            except Exception as e:
                st.error(f"Could not render 3D preview: {e}")

        # Download simulation files if exported
        if "exports" in st.session_state:
            st.markdown("---")
            st.markdown("### 📁 Simulation Files")

            dl_cols = st.columns(4)
            exports = st.session_state["exports"]

            for i, (fmt, path) in enumerate(exports.items()):
                path = Path(path)
                if path.exists():
                    with dl_cols[i % 4]:
                        data = path.read_text(encoding="utf-8", errors="replace")
                        st.download_button(
                            f"📄 {fmt.upper()}",
                            data,
                            file_name=path.name,
                            key=f"dl_{fmt}",
                        )

            # Validate in MuJoCo
            mjcf_path = exports.get("mjcf")
            if mjcf_path and Path(mjcf_path).exists() and MUJOCO_AVAILABLE:
                st.markdown("---")
                st.markdown("### MuJoCo Validation")
                if st.button("Validate in MuJoCo", type="secondary", key="validate_mjcf_btn"):
                    with st.spinner("Validating MJCF…"):
                        validator = MuJoCoValidator()
                        result = validator.validate_mjcf(mjcf_path, design_id=design.id)
                        st.session_state["mjcf_validation"] = result
                    st.rerun()
                if "mjcf_validation" in st.session_state:
                    res = st.session_state["mjcf_validation"]
                    if res.valid and res.loads and res.simulates:
                        st.success("Pass: loads and simulates.")
                    elif res.loads and not res.simulates:
                        st.warning("Loads but simulation failed.")
                    else:
                        st.error("Validation failed.")
                    if res.errors:
                        for e in res.errors:
                            st.error(e)
                    if res.warnings:
                        for w in res.warnings:
                            st.warning(w)
                    if res.stats:
                        st.caption(f"Stats: {res.stats}")
            elif mjcf_path and Path(mjcf_path).exists() and not MUJOCO_AVAILABLE:
                st.caption("MuJoCo validator unavailable (install mujoco).")

            # Export Bundle (.zip)
            st.markdown("---")
            st.markdown("### Export Bundle (.zip)")
            st.caption("ZIP with design JSON, sim exports, README (assumptions + parameters + version), optional STL.")
            if st.button("Export Bundle (.zip)", type="secondary", key="export_bundle_btn"):
                import zipfile
                from datetime import datetime
                output_dir = Path(__file__).resolve().parent.parent / "outputs" / design.id.lower().replace("-", "_")
                output_dir.mkdir(parents=True, exist_ok=True)
                zip_path = output_dir / f"{design.id.lower().replace('-', '_')}_bundle.zip"
                with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
                    for fmt, path in exports.items():
                        p = Path(path)
                        if p.exists():
                            zf.write(p, p.name)
                    # README
                    readme = f"""# {design.name}
ID: {design.id}
Generated: {datetime.now().isoformat()}
App: OMEGA Soft Robotics Lab

## Parameters
- Gesture: {design.source_gesture.value}
- Environment: {design.source_parameters.get('environment', 'dry')}
- Object compliance: {design.source_parameters.get('object_compliance', 0.5)}
- Fingers: {design.num_fingers}
- Actuator: {design.primary_actuator.value}
- Confidence (rule-based): {design.confidence:.0%}

## Assumptions
- Confidence is rule-based; no physical simulation or hardware validation.
- Validate MJCF in MuJoCo for simulation sanity.
"""
                    zf.writestr("README.txt", readme)
                    if "stl_path" in st.session_state and Path(st.session_state["stl_path"]).exists():
                        zf.write(st.session_state["stl_path"], Path(st.session_state["stl_path"]).name)
                st.session_state["bundle_path"] = str(zip_path)
                st.success(f"Bundle saved: {zip_path.name}")
                st.rerun()
            if "bundle_path" in st.session_state and Path(st.session_state["bundle_path"]).exists():
                bundle_path = Path(st.session_state["bundle_path"])
                st.download_button(
                    "Download Bundle (.zip)",
                    bundle_path.read_bytes(),
                    file_name=bundle_path.name,
                    mime="application/zip",
                    key="dl_bundle",
                )

    else:
        st.markdown(
            """
        <div style="background: #1a1a2e; padding: 60px; border-radius: 15px; text-align: center; border: 2px dashed #333;">
            <h2 style="color: #00d4ff;">👈 Configure Your Gripper</h2>
            <p style="color: #888;">Set the parameters on the left and click <b>Generate Gripper</b></p>
            <br>
            <p style="color: #666;">You'll get:</p>
            <p style="color: #888;">• Interactive 3D preview</p>
            <p style="color: #888;">• Confidence analysis</p>
            <p style="color: #888;">• Failure mode prediction</p>
            <p style="color: #888;">• Export to MuJoCo, ROS, Omniverse</p>
        </div>
        """,
            unsafe_allow_html=True,
        )
