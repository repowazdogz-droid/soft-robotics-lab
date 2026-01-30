"""
Gripper Zoo Page
"""

import streamlit as st
import json
from pathlib import Path

st.set_page_config(
    page_title="Gripper Zoo", page_icon="🦾", layout="wide"
)

st.markdown("# 🦾 Gripper Zoo")
st.markdown("Browse 50 pre-generated gripper designs")

st.markdown("---")

zoo_path = (
    Path(__file__).resolve().parent.parent
    / "gripper_zoo"
    / "designs"
    / "index.json"
)

if not zoo_path.exists():
    st.error(
        "Gripper zoo not generated. Run `python gripper_zoo/generate_zoo.py` first."
    )
    st.stop()

zoo = json.loads(zoo_path.read_text())
designs = zoo["designs"]

# Filters
col1, col2, col3, col4 = st.columns(4)

with col1:
    gesture_filter = st.selectbox(
        "Gesture",
        ["All"] + sorted(set(d["gesture"] for d in designs)),
    )
with col2:
    env_filter = st.selectbox(
        "Environment",
        ["All"] + sorted(set(d["environment"] for d in designs)),
    )
with col3:
    compliance_filter = st.selectbox(
        "Compliance",
        ["All"] + sorted(set(d["compliance"] for d in designs)),
    )
with col4:
    actuator_filter = st.selectbox(
        "Actuator",
        ["All"] + sorted(set(d["actuator"] for d in designs)),
    )

# Apply filters
filtered = designs
if gesture_filter != "All":
    filtered = [d for d in filtered if d["gesture"] == gesture_filter]
if env_filter != "All":
    filtered = [d for d in filtered if d["environment"] == env_filter]
if compliance_filter != "All":
    filtered = [d for d in filtered if d["compliance"] == compliance_filter]
if actuator_filter != "All":
    filtered = [d for d in filtered if d["actuator"] == actuator_filter]

st.markdown(f"**Showing {len(filtered)} of {len(designs)} designs**")

st.markdown("---")

# Display as grid
cols_per_row = 3
rows = [
    filtered[i : i + cols_per_row]
    for i in range(0, len(filtered), cols_per_row)
]

for row in rows:
    cols = st.columns(cols_per_row)
    for i, design in enumerate(row):
        with cols[i]:
            conf_color = (
                "#00ff88"
                if design["confidence"] > 0.75
                else "#ffaa00"
                if design["confidence"] > 0.5
                else "#ff4444"
            )

            st.markdown(
                f"""
            <div style="background: #1a1a2e; padding: 15px; border-radius: 10px; margin-bottom: 10px; border: 1px solid #333;">
                <h4 style="color: #00d4ff; margin: 0; font-size: 0.9rem;">{design["name"][:35]}</h4>
                <p style="color: #888; margin: 5px 0; font-size: 0.8rem;">{design["id"]}</p>
                <div style="display: flex; justify-content: space-between; margin-top: 10px;">
                    <span style="color: #fff;">{design["num_fingers"]} fingers</span>
                    <span style="color: {conf_color}; font-weight: bold;">{design["confidence"]*100:.0f}%</span>
                </div>
                <p style="color: #888; font-size: 0.8rem; margin-top: 5px;">{design["actuator"]} • {design["environment"]}</p>
            </div>
            """,
                unsafe_allow_html=True,
            )

            design_dir = zoo_path.parent / design["id"].lower().replace(
                "-", "_"
            )

            if design_dir.exists():
                base_name = design["id"].lower().replace("-", "_")
                json_file = design_dir / f"{base_name}.json"
                mjcf_file = design_dir / f"{base_name}.mjcf"

                if json_file.exists():
                    col_a, col_b = st.columns(2)
                    with col_a:
                        st.download_button(
                            "JSON",
                            json_file.read_text(),
                            file_name=json_file.name,
                            key=f"json_{design['id']}_{i}",
                        )
                    with col_b:
                        if mjcf_file.exists():
                            st.download_button(
                                "MJCF",
                                mjcf_file.read_text(),
                                file_name=mjcf_file.name,
                                key=f"mjcf_{design['id']}_{i}",
                            )

# Stats at bottom
st.markdown("---")
st.markdown("### Zoo Statistics")

stat_col1, stat_col2, stat_col3 = st.columns(3)

with stat_col1:
    st.markdown("**By Gesture**")
    gesture_counts = {}
    for d in designs:
        g = d["gesture"]
        gesture_counts[g] = gesture_counts.get(g, 0) + 1
    for g, c in sorted(gesture_counts.items()):
        st.markdown(f"- {g}: {c}")

with stat_col2:
    st.markdown("**By Environment**")
    env_counts = {}
    for d in designs:
        e = d["environment"]
        env_counts[e] = env_counts.get(e, 0) + 1
    for e, c in sorted(env_counts.items()):
        st.markdown(f"- {e}: {c}")

with stat_col3:
    st.markdown("**By Actuator**")
    act_counts = {}
    for d in designs:
        a = d["actuator"]
        act_counts[a] = act_counts.get(a, 0) + 1
    for a, c in sorted(act_counts.items()):
        st.markdown(f"- {a}: {c}")
