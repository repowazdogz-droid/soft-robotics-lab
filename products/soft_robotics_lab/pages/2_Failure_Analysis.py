"""
Failure Analysis Page - World Class Edition
"""

import streamlit as st
import sys
import json
import numpy as np
import plotly.graph_objects as go
from pathlib import Path

_soft_lab = Path(__file__).resolve().parent.parent
if str(_soft_lab) not in sys.path:
    sys.path.insert(0, str(_soft_lab))

from workbench.failure_predictor import FailurePredictor

st.set_page_config(
    page_title="Failure Analysis", page_icon="⚠️", layout="wide"
)

st.markdown("""
<style>
    .risk-card {
        background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
        border-radius: 12px;
        padding: 20px;
        margin: 10px 0;
        border-left: 4px solid;
    }
    .risk-critical { border-color: #ff4444; }
    .risk-high { border-color: #ff6b6b; }
    .risk-medium { border-color: #ffaa00; }
    .risk-low { border-color: #00ff88; }
</style>
""", unsafe_allow_html=True)

st.markdown("# ⚠️ Failure Mode Analysis")
st.markdown(
    "*Comprehensive risk assessment for your gripper design*"
)

st.markdown("---")

predictor = FailurePredictor()


def create_risk_matrix(failures) -> go.Figure:
    """Create a 2D risk matrix (probability vs severity)."""
    severity_map = {"low": 1, "medium": 2, "high": 3, "critical": 4}
    severity_labels = ["Low", "Medium", "High", "Critical"]

    z_background = np.array([
        [1, 2, 3, 4],
        [2, 4, 6, 8],
        [3, 6, 9, 12],
        [4, 8, 12, 16],
    ])

    fig = go.Figure()

    fig.add_trace(
        go.Heatmap(
            z=z_background,
            x=severity_labels,
            y=["0-25%", "25-50%", "50-75%", "75-100%"],
            colorscale=[
                [0, "#1a472a"],
                [0.25, "#2d5a3d"],
                [0.4, "#7d6608"],
                [0.6, "#8b4513"],
                [0.8, "#8b0000"],
                [1, "#4a0000"],
            ],
            showscale=False,
            hoverinfo="skip",
        )
    )

    for idx, f in enumerate(failures):
        sev_idx = severity_map.get(f.severity, 2)
        prob = f.probability

        if prob < 0.25:
            prob_idx = 0
        elif prob < 0.5:
            prob_idx = 1
        elif prob < 0.75:
            prob_idx = 2
        else:
            prob_idx = 3

        # Deterministic jitter from index so chart is stable on rerun
        jitter_x = ((idx * 7) % 11) / 55.0 - 0.1
        jitter_y = ((idx * 13) % 11) / 55.0 - 0.1

        color = (
            "#ff4444"
            if f.severity in ["critical", "high"]
            else "#ffaa00"
            if f.severity == "medium"
            else "#00ff88"
        )

        fig.add_trace(
            go.Scatter(
                x=[sev_idx - 1 + jitter_x],
                y=[prob_idx + jitter_y],
                mode="markers+text",
                marker=dict(
                    size=20,
                    color=color,
                    line=dict(width=2, color="white"),
                ),
                text=[f.mode[:15] + ("..." if len(f.mode) > 15 else "")],
                textposition="top center",
                textfont=dict(size=10, color="white"),
                hovertemplate=(
                    f"<b>{f.mode}</b><br>Probability: {f.probability*100:.0f}%<br>"
                    f"Severity: {f.severity}<extra></extra>"
                ),
                showlegend=False,
            )
        )

    fig.update_layout(
        title=dict(text="Risk Matrix", font=dict(color="white", size=16)),
        xaxis=dict(
            title="Severity",
            tickmode="array",
            tickvals=[0, 1, 2, 3],
            ticktext=severity_labels,
            tickfont=dict(color="white"),
            title_font=dict(color="white"),
            gridcolor="rgba(255,255,255,0.1)",
        ),
        yaxis=dict(
            title="Probability",
            tickmode="array",
            tickvals=[0, 1, 2, 3],
            ticktext=["0-25%", "25-50%", "50-75%", "75-100%"],
            tickfont=dict(color="white"),
            title_font=dict(color="white"),
            gridcolor="rgba(255,255,255,0.1)",
        ),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        height=400,
        margin=dict(l=60, r=20, t=50, b=60),
    )

    return fig


def create_risk_breakdown(failures) -> go.Figure:
    """Create horizontal bar chart of failure probabilities."""
    sorted_failures = sorted(
        failures, key=lambda f: f.probability, reverse=True
    )

    modes = [
        f.mode[:30] + ("..." if len(f.mode) > 30 else "")
        for f in sorted_failures
    ]
    probs = [f.probability * 100 for f in sorted_failures]
    colors = [
        "#ff4444"
        if f.severity == "critical"
        else "#ff6b6b"
        if f.severity == "high"
        else "#ffaa00"
        if f.severity == "medium"
        else "#00ff88"
        for f in sorted_failures
    ]

    fig = go.Figure()

    fig.add_trace(
        go.Bar(
            y=modes,
            x=probs,
            orientation="h",
            marker=dict(color=colors, line=dict(width=0)),
            hovertemplate="<b>%{y}</b><br>Probability: %{x:.0f}%<extra></extra>",
        )
    )

    fig.update_layout(
        title=dict(
            text="Failure Probability Breakdown",
            font=dict(color="white", size=16),
        ),
        xaxis=dict(
            title="Probability (%)",
            range=[0, 100],
            tickfont=dict(color="white"),
            title_font=dict(color="white"),
            gridcolor="rgba(255,255,255,0.1)",
        ),
        yaxis=dict(tickfont=dict(color="white"), autorange="reversed"),
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        height=max(300, len(failures) * 50),
        margin=dict(l=200, r=20, t=50, b=40),
        showlegend=False,
    )

    return fig


# Layout
col_input, col_output = st.columns([1, 3])

with col_input:
    st.markdown("### Load Design")

    if "current_design" in st.session_state:
        design = st.session_state["current_design"]
        st.success(f"✓ {design.name}")
        use_current = st.button(
            "Analyze This Design",
            type="primary",
            use_container_width=True,
        )
    else:
        use_current = False
        st.info("Generate a design in the Design Studio first")

    st.markdown("---")

    st.markdown("### Or Upload Design")
    uploaded = st.file_uploader(
        "Upload JSON", type=["json"], label_visibility="collapsed"
    )

    st.markdown("---")

    st.markdown("### Test Environment")
    env_override = st.radio(
        "Simulate different conditions",
        ["From design", "Dry", "Wet", "Surgical"],
        label_visibility="collapsed",
    )

    st.markdown("---")

    st.markdown("### Risk Legend")
    st.markdown("""
    <div style="font-size: 0.85rem;">
        <p><span style="color: #ff4444;">●</span> Critical - Immediate action required</p>
        <p><span style="color: #ff6b6b;">●</span> High - Address before deployment</p>
        <p><span style="color: #ffaa00;">●</span> Medium - Monitor and mitigate</p>
        <p><span style="color: #00ff88;">●</span> Low - Acceptable risk</p>
    </div>
    """, unsafe_allow_html=True)

with col_output:
    design_to_analyze = None

    if use_current and "current_design" in st.session_state:
        design_to_analyze = st.session_state["current_design"].to_dict()
    elif uploaded:
        design_to_analyze = json.loads(uploaded.read())

    if design_to_analyze:
        env = (
            env_override.lower()
            if env_override != "From design"
            else design_to_analyze.get("source_parameters", {}).get(
                "environment", "dry"
            )
        )

        with st.spinner("Analyzing failure modes..."):
            failures = predictor.predict(
                design_to_analyze, environment=env
            )

        st.markdown(
            f"""
        <div style="background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); padding: 20px; border-radius: 12px; margin-bottom: 20px;">
            <h2 style="color: #00d4ff; margin: 0;">{design_to_analyze.get('name', 'Design Analysis')}</h2>
            <p style="color: #888; margin: 10px 0 0 0;">Environment: <b>{env.title()}</b> • {len(failures)} failure modes identified</p>
        </div>
        """,
            unsafe_allow_html=True,
        )

        critical_count = sum(
            1 for f in failures if f.severity == "critical"
        )
        high_count = sum(1 for f in failures if f.severity == "high")
        medium_count = sum(1 for f in failures if f.severity == "medium")
        low_count = sum(1 for f in failures if f.severity == "low")
        avg_prob = (
            sum(f.probability for f in failures) / len(failures)
            if failures
            else 0
        )

        metric_cols = st.columns(5)
        with metric_cols[0]:
            st.metric(
                "Critical",
                critical_count,
                delta=None if critical_count == 0 else "⚠️",
                delta_color="inverse",
            )
        with metric_cols[1]:
            st.metric("High", high_count)
        with metric_cols[2]:
            st.metric("Medium", medium_count)
        with metric_cols[3]:
            st.metric("Low", low_count)
        with metric_cols[4]:
            st.metric("Avg Probability", f"{avg_prob*100:.0f}%")

        st.markdown("---")

        chart_col1, chart_col2 = st.columns(2)

        with chart_col1:
            st.plotly_chart(
                create_risk_matrix(failures),
                use_container_width=True,
                config={"displayModeBar": False},
            )

        with chart_col2:
            st.plotly_chart(
                create_risk_breakdown(failures),
                use_container_width=True,
                config={"displayModeBar": False},
            )

        st.markdown("---")
        st.markdown("### Detailed Analysis")

        tab_critical, tab_high, tab_medium, tab_low = st.tabs(
            ["⛔ Critical", "🔴 High", "🟡 Medium", "🟢 Low"]
        )

        def render_failure_card(f):
            color = (
                "#ff4444"
                if f.severity == "critical"
                else "#ff6b6b"
                if f.severity == "high"
                else "#ffaa00"
                if f.severity == "medium"
                else "#00ff88"
            )
            st.markdown(
                f"""
            <div class="risk-card risk-{f.severity}">
                <div style="display: flex; justify-content: space-between; align-items: center;">
                    <h3 style="color: white; margin: 0;">{f.mode}</h3>
                    <span style="color: {color}; font-size: 1.5rem; font-weight: bold;">{f.probability*100:.0f}%</span>
                </div>
                <p style="color: #888; margin: 15px 0 5px 0;"><b>Cause:</b></p>
                <p style="color: #ccc; margin: 0;">{f.cause}</p>
                <p style="color: #888; margin: 15px 0 5px 0;"><b>Mitigation:</b></p>
                <p style="color: #00ff88; margin: 0;">{f.mitigation}</p>
                <p style="color: #888; margin: 15px 0 5px 0;"><b>Detection:</b></p>
                <p style="color: #00d4ff; margin: 0;">{f.detection}</p>
            </div>
            """,
                unsafe_allow_html=True,
            )

        with tab_critical:
            critical_failures = [
                f for f in failures if f.severity == "critical"
            ]
            if critical_failures:
                for f in critical_failures:
                    render_failure_card(f)
            else:
                st.success("No critical failures identified")

        with tab_high:
            high_failures = [f for f in failures if f.severity == "high"]
            if high_failures:
                for f in high_failures:
                    render_failure_card(f)
            else:
                st.success("No high-severity failures identified")

        with tab_medium:
            medium_failures = [
                f for f in failures if f.severity == "medium"
            ]
            if medium_failures:
                for f in medium_failures:
                    render_failure_card(f)
            else:
                st.info("No medium-severity failures identified")

        with tab_low:
            low_failures = [f for f in failures if f.severity == "low"]
            if low_failures:
                for f in low_failures:
                    render_failure_card(f)
            else:
                st.info("No low-severity failures identified")

        st.markdown("---")
        st.markdown("### Recommendations")

        rec_cols = st.columns(3)

        with rec_cols[0]:
            st.markdown("""
            <div style="background: #252540; padding: 20px; border-radius: 10px; height: 100%;">
                <h4 style="color: #ff4444;">🛑 Before Prototyping</h4>
                <p style="color: #ccc;">Address all critical and high-severity failure modes before fabricating a prototype.</p>
            </div>
            """, unsafe_allow_html=True)

        with rec_cols[1]:
            st.markdown("""
            <div style="background: #252540; padding: 20px; border-radius: 10px; height: 100%;">
                <h4 style="color: #ffaa00;">📊 Add Sensing</h4>
                <p style="color: #ccc;">Implement detection methods for the top 3 failure modes to enable early warning.</p>
            </div>
            """, unsafe_allow_html=True)

        with rec_cols[2]:
            st.markdown("""
            <div style="background: #252540; padding: 20px; border-radius: 10px; height: 100%;">
                <h4 style="color: #00ff88;">🧪 Simulate First</h4>
                <p style="color: #ccc;">Test mitigations in MuJoCo/Isaac Sim before committing to hardware.</p>
            </div>
            """, unsafe_allow_html=True)

    else:
        st.markdown("""
        <div style="background: #1a1a2e; padding: 80px; border-radius: 15px; text-align: center; border: 2px dashed #333;">
            <h2 style="color: #00d4ff;">Load a Design to Analyze</h2>
            <p style="color: #888; max-width: 400px; margin: 20px auto;">
                Generate a gripper in the <b>Design Studio</b>, then come back here to analyze its failure modes.
            </p>
            <p style="color: #666; margin-top: 30px;">Or upload an existing design JSON file.</p>
        </div>
        """, unsafe_allow_html=True)
