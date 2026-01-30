"""
Reality Bridge Dashboard - Streamlit monitoring UI.
Shows real-time stats, recent validations, failure analysis, performance trends.
Run: streamlit run dashboard.py (from products/reality_bridge or with PYTHONPATH set).
"""

import sys
from pathlib import Path

_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))
_products = _root.parent
if str(_products) not in sys.path:
    sys.path.insert(0, str(_products))

import streamlit as st
import requests
import pandas as pd
from datetime import datetime

API_BASE = "http://localhost:8000"
TUTOR_BASE = "http://localhost:8503"

st.set_page_config(
    page_title="Reality Bridge Dashboard",
    page_icon="🌉",
    layout="wide",
    initial_sidebar_state="expanded",
)

st.markdown("""
<style>
    .main-header { font-size: 2rem; font-weight: 700; color: #00d4ff; margin-bottom: 0; }
    .metric-card { background: #1a1a2e; padding: 1rem; border-radius: 8px; margin: 0.5rem 0; }
</style>
""", unsafe_allow_html=True)


def fetch_json(path: str, default=None):
    try:
        r = requests.get(f"{API_BASE}{path}", timeout=5)
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return default


st.title("REALITY BRIDGE DASHBOARD")
st.caption("Physics validation monitoring — stats, failures, trends.")

# Status
status = fetch_json("/health")
if status and status.get("status") == "ok":
    st.success("Status: 🟢 Online | MuJoCo: " + str(status.get("mujoco_version", "—")))
else:
    st.error("Status: 🔴 Offline — start Reality Bridge: uvicorn app:app --reload --port 8000")

# Stats
stats = fetch_json("/stats") or {}
total = stats.get("total_validations") or stats.get("total") or 0
pass_rate = stats.get("pass_rate", 0.0)
avg_score = stats.get("avg_score", 0.0)
passed = int(total * pass_rate) if total else 0
failed = total - passed
avg_ms = stats.get("avg_validation_ms")
today = stats.get("today") or {}
week = stats.get("this_week") or {}
failure_dist = stats.get("failure_distribution") or {}

col1, col2, col3 = st.columns(3)
with col1:
    st.metric("Total Validations", f"{total:,}", help="All-time count")
with col2:
    st.metric("Passed", f"{passed:,} ({100*pass_rate:.0f}%)", help="Pass rate")
with col3:
    st.metric("Failed", f"{failed:,}", help="Failed count")
st.caption(f"Today: {today.get('count', 0)} validations ({100*(today.get('pass_rate') or 0):.0f}% pass) | This week: {week.get('count', 0)} | Avg response: {avg_ms or '—'} ms")

# Recent validations
st.markdown("---")
st.subheader("Recent Validations")
recent = fetch_json("/validations/recent?limit=20") or []
if recent:
    df = pd.DataFrame(recent)
    df["Status"] = df.get("passed", pd.Series([False] * len(df))).map(lambda x: "✅" if x else "❌")
    cols = [c for c in ["timestamp", "artifact_id", "design_hash", "score", "Status", "validation_time_ms"] if c in df.columns]
    st.dataframe(df[cols], use_container_width=True, hide_index=True)
else:
    st.caption("No validations yet.")

# Failure analysis
st.markdown("---")
st.subheader("Failure Analysis")
st.caption("Top failure types (this week)")
if failure_dist:
    for code, pct in sorted(failure_dist.items(), key=lambda x: -x[1])[:6]:
        st.markdown(f"- **{code}**: {100*pct:.0f}%")
    st.markdown(f"[Learn why →]({TUTOR_BASE}/?topic=physics+simulation+stability)")
else:
    st.caption("No failure distribution data.")
failures_list = fetch_json("/failures?limit=20") or []
if failures_list:
    with st.expander("View All Failures"):
        for f in failures_list[:10]:
            st.caption(f"{f.get('timestamp', '')[:19]} | {f.get('design_hash', '')[:12]}... | score={f.get('score')} | errors: {f.get('errors', [])[:2]}")

# Performance trends (simplified: daily counts from recent)
st.markdown("---")
st.subheader("Performance Trends")
try:
    import plotly.express as px
    import plotly.graph_objects as go
except ImportError:
    plotly = None
if recent and len(recent) > 2 and plotly:
    df = pd.DataFrame(recent)
    if "timestamp" in df.columns and df["timestamp"].notna().any():
        df["date"] = pd.to_datetime(df["timestamp"], errors="coerce").dt.date
        daily = df.groupby("date").agg({"passed": "count", "score": "mean"}).reset_index()
        daily.columns = ["date", "count", "avg_score"]
        fig = go.Figure()
        fig.add_trace(go.Bar(x=daily["date"], y=daily["count"], name="Validations"))
        fig.add_trace(go.Scatter(x=daily["date"], y=daily["avg_score"], name="Avg score", yaxis="y2", mode="lines+markers"))
        fig.update_layout(title="Validations per day & avg score", xaxis_title="Date", yaxis_title="Count", yaxis2=dict(title="Score", overlaying="y", side="right", range=[0, 1.1]))
        st.plotly_chart(fig, use_container_width=True)
else:
    st.caption("Add more validations to see trends, or install plotly.")

# Filters & export
st.sidebar.subheader("Filters")
date_range = st.sidebar.date_input("Date range", [])
status_filter = st.sidebar.selectbox("Status", ["All", "Passed", "Failed"])
if recent:
    csv_data = pd.DataFrame(recent).to_csv(index=False)
    st.sidebar.download_button("Download Report (CSV)", data=csv_data, file_name=f"reality_bridge_report_{datetime.utcnow().strftime('%Y%m%d')}.csv", mime="text/csv", key="dl_report")
else:
    st.sidebar.caption("No data to export yet.")

st.sidebar.caption("Reality Bridge API: " + API_BASE)
st.sidebar.caption("Tutor: " + TUTOR_BASE)
