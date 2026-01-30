"""
Research Memory Page - World Class Edition
==========================================

Features:
- Tag system with filtering
- Instant search-as-you-type
- Visual hypothesis dashboard with confidence over time
- Rich note editing
"""

import streamlit as st
import sys
import re
import json
from pathlib import Path
from datetime import datetime, timedelta

import plotly.graph_objects as go

sys.path.insert(0, str(Path(__file__).parent.parent / "research_system"))

from research_memory import ResearchMemory

st.set_page_config(page_title="Research Memory", page_icon="📚", layout="wide")

st.markdown("""
<style>
    .tag {
        display: inline-block;
        background: #3d5a80;
        color: white;
        padding: 4px 12px;
        border-radius: 15px;
        margin: 2px 4px;
        font-size: 0.85rem;
    }
    .tag-active {
        background: #00d4ff;
    }
    .note-card {
        background: #2d2d44;
        padding: 20px;
        border-radius: 12px;
        margin: 10px 0;
        border-left: 4px solid #00d4ff;
    }
    .hypothesis-card {
        background: #252540;
        padding: 20px;
        border-radius: 12px;
        margin: 15px 0;
    }
    .confidence-high { border-left: 4px solid #00ff88; }
    .confidence-medium { border-left: 4px solid #ffaa00; }
    .confidence-low { border-left: 4px solid #ff4444; }
</style>
""", unsafe_allow_html=True)

st.markdown("# 📚 Research Memory")
st.markdown("*Your lab's knowledge base with hypothesis tracking*")

st.markdown("---")

# Initialize
@st.cache_resource
def get_memory(lab_name):
    return ResearchMemory(lab_name)

lab_name = st.sidebar.text_input("Lab Name", value="rossiter_lab")
memory = get_memory(lab_name)

# Reset notes
st.sidebar.markdown("---")
if st.sidebar.button("🗑️ Reset notes (0 docs/chunks)", type="secondary", use_container_width=True):
    import sqlite3
    conn = sqlite3.connect(memory.db_path)
    try:
        conn.execute("DELETE FROM chunk_embeddings")
    except Exception:
        pass
    conn.execute("DELETE FROM chunks")
    conn.execute("DELETE FROM documents")
    conn.commit()
    conn.close()
    get_memory.clear()
    st.sidebar.success("Reset. Reloading...")
    st.rerun()

# Load or initialize tags
tags_file = memory.base_path / "tags.json"
if tags_file.exists():
    try:
        all_tags = json.loads(tags_file.read_text())
    except Exception:
        all_tags = ["soft-robotics", "gripper", "pneumatic", "tendon", "experiment", "review", "idea"]
else:
    all_tags = ["soft-robotics", "gripper", "pneumatic", "tendon", "experiment", "review", "idea"]


def save_tags():
    memory.base_path.mkdir(parents=True, exist_ok=True)
    tags_file.write_text(json.dumps(list(set(all_tags))))


# Stats row
stats = memory.get_stats()

col1, col2, col3, col4 = st.columns(4)
with col1:
    st.metric("📄 Documents", stats["documents"])
with col2:
    st.metric("🧩 Chunks", stats["chunks"])
with col3:
    st.metric("💡 Hypotheses", stats["hypotheses"])
with col4:
    st.metric("✅ Active", stats["active_hypotheses"])

st.markdown("---")

# Tabs
tab1, tab2, tab3, tab4 = st.tabs(["📝 Notes", "💡 Hypotheses", "📊 Dashboard", "📋 Brief"])

# ══════════════════════════════════════════════════════════════════════════════
# TAB 1: NOTES
# ══════════════════════════════════════════════════════════════════════════════
with tab1:
    col_add, col_browse = st.columns([1, 1])

    with col_add:
        st.markdown("### ✏️ Add Note")

        note_title = st.text_input("Title", placeholder="e.g., Experiment results - variable stiffness")

        # Tag selector
        st.markdown("**Tags:**")
        selected_tags = []
        tag_cols = st.columns(4)
        for i, tag in enumerate(all_tags[:8]):
            with tag_cols[i % 4]:
                if st.checkbox(tag, key=f"tag_{tag}"):
                    selected_tags.append(tag)

        # Add new tag
        new_tag = st.text_input("Add new tag", placeholder="new-tag", label_visibility="collapsed", key="new_tag_input")
        if new_tag and st.button("➕ Add Tag", key="add_tag_btn"):
            all_tags.append(new_tag.lower().replace(" ", "-"))
            save_tags()
            st.rerun()

        note_content = st.text_area(
            "Content (Markdown supported)",
            height=250,
            placeholder="## Summary\n\nWrite your research notes here...\n\n## Key Findings\n\n- Finding 1\n- Finding 2",
            key="note_content_area",
        )

        if st.button("💾 Save Note", type="primary", use_container_width=True, key="save_note_btn"):
            if note_title and note_content:
                notes_dir = memory.base_path / "notes"
                notes_dir.mkdir(exist_ok=True)

                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                safe_title = re.sub(r'[^\w\s-]', '', note_title).replace(' ', '_')[:50]
                filename = f"{timestamp}_{safe_title}.md"

                # Build content with metadata
                tags_str = ", ".join(selected_tags) if selected_tags else "untagged"
                content = f"""---
title: {note_title}
date: {datetime.now().strftime('%Y-%m-%d %H:%M')}
tags: [{tags_str}]
---

# {note_title}

{note_content}
"""

                filepath = notes_dir / filename
                filepath.write_text(content, encoding='utf-8')

                # Ingest into memory
                memory.ingest(str(filepath), project="notes")

                st.success(f"✅ Saved: {filename}")
                get_memory.clear()
                st.rerun()
            else:
                st.warning("Please enter title and content")

    with col_browse:
        st.markdown("### 🔍 Search & Browse")

        # Instant search
        search_query = st.text_input("Search notes", placeholder="Type to search...", key="note_search")

        # Tag filter
        tag_filter = st.multiselect("Filter by tags", all_tags, key="tag_filter")

        # Load notes
        notes_dir = memory.base_path / "notes"
        notes = []

        if notes_dir.exists():
            for note_file in sorted(notes_dir.glob("*.md"), key=lambda f: f.stat().st_mtime, reverse=True):
                content = note_file.read_text(encoding='utf-8')

                # Parse front matter
                title = note_file.stem
                tags = []
                date = ""

                if content.startswith("---"):
                    try:
                        front_matter_end = content.index("---", 3)
                        front_matter = content[3:front_matter_end]

                        for line in front_matter.split("\n"):
                            if line.startswith("title:"):
                                title = line.split(":", 1)[1].strip()
                            elif line.startswith("tags:"):
                                tags_str = line.split(":", 1)[1].strip()
                                tags = [t.strip().strip("[]") for t in tags_str.split(",")]
                            elif line.startswith("date:"):
                                date = line.split(":", 1)[1].strip()

                        body = content[front_matter_end + 3:].strip()
                    except Exception:
                        body = content
                else:
                    body = content

                notes.append({
                    "file": note_file,
                    "title": title,
                    "tags": tags,
                    "date": date,
                    "body": body,
                    "content": content
                })

        # Filter by search
        if search_query:
            notes = [n for n in notes if search_query.lower() in n["content"].lower()]

        # Filter by tags
        if tag_filter:
            notes = [n for n in notes if any(t in n["tags"] for t in tag_filter)]

        st.markdown(f"**{len(notes)} notes**")

        for note in notes[:10]:
            tags_html = " ".join([f'<span class="tag">{t}</span>' for t in note["tags"] if t])

            with st.expander(f"📝 {note['title']}", expanded=False):
                st.markdown(f"<small>📅 {note['date']}</small> {tags_html}", unsafe_allow_html=True)
                st.markdown("---")
                st.markdown(note["body"][:1000] + ("..." if len(note["body"]) > 1000 else ""))

# ══════════════════════════════════════════════════════════════════════════════
# TAB 2: HYPOTHESES
# ══════════════════════════════════════════════════════════════════════════════
with tab2:
    col_create, col_list = st.columns([1, 2])

    with col_create:
        st.markdown("### ➕ New Hypothesis")

        claim = st.text_area(
            "Claim (make it falsifiable)",
            placeholder="e.g., Tendon-driven actuators achieve higher precision than pneumatic for objects under 50g in dry conditions",
            height=100,
            key="new_claim",
        )

        scope = st.text_input("Scope / Conditions", placeholder="e.g., Room temperature, objects 10-50g", key="new_scope")

        initial_conf = st.slider("Initial confidence", 0.0, 1.0, 0.5, 0.05, key="new_conf")

        competing = st.text_input("Competing hypothesis (optional)", placeholder="Alternative explanation", key="new_competing")

        if st.button("Create Hypothesis", type="primary", use_container_width=True, key="create_hyp_btn"):
            if claim:
                hyp = memory.create_hypothesis(
                    claim=claim,
                    scope=scope or "Not specified",
                    assumptions=[],
                    predictions=[],
                    created_by=lab_name,
                )
                memory.update_hypothesis(hyp.id, confidence=initial_conf)

                # Log initial confidence to history
                history_file = memory.base_path / "hypothesis_history.json"
                try:
                    history = json.loads(history_file.read_text()) if history_file.exists() else {}
                except Exception:
                    history = {}
                history[hyp.id] = [{"date": datetime.now().isoformat(), "confidence": initial_conf, "event": "created"}]
                memory.base_path.mkdir(parents=True, exist_ok=True)
                history_file.write_text(json.dumps(history, indent=2))

                st.success(f"✅ Created: {hyp.id}")
                get_memory.clear()
                st.rerun()
            else:
                st.warning("Please enter a claim")

    with col_list:
        st.markdown("### 📋 Active Hypotheses")

        hypotheses = memory.get_hypotheses()

        # Load history for confidence tracking
        history_file = memory.base_path / "hypothesis_history.json"
        try:
            history = json.loads(history_file.read_text()) if history_file.exists() else {}
        except Exception:
            history = {}

        if not hypotheses:
            st.info("No hypotheses yet. Create your first one!")
        else:
            for hyp in hypotheses:
                # Determine card styling
                if hyp.confidence >= 0.7:
                    card_class = "hypothesis-card confidence-high"
                elif hyp.confidence >= 0.4:
                    card_class = "hypothesis-card confidence-medium"
                else:
                    card_class = "hypothesis-card confidence-low"

                status_emoji = {
                    "active": "🔵",
                    "strengthened": "🟢",
                    "weakened": "🟡",
                    "falsified": "🔴"
                }.get(hyp.status, "⚪")

                st.markdown(f"""
                <div class="{card_class}">
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                        <span style="color: #00d4ff; font-weight: bold;">{hyp.id}</span>
                        <span>{status_emoji} {hyp.status.upper()} • <b>{hyp.confidence:.0%}</b></span>
                    </div>
                    <p style="color: #e8e8e8; margin: 15px 0; font-size: 1.05rem;">{hyp.claim}</p>
                    <small style="color: #888;">Scope: {hyp.scope or 'Not specified'}</small>
                </div>
                """, unsafe_allow_html=True)

                # Update controls
                with st.expander("📝 Update", expanded=False):
                    col_a, col_b, col_c = st.columns([2, 2, 1])

                    with col_a:
                        new_conf = st.slider(
                            "Confidence",
                            0.0, 1.0, hyp.confidence, 0.05,
                            key=f"conf_{hyp.id}"
                        )

                    with col_b:
                        status_options = ["active", "strengthened", "weakened", "falsified"]
                        status_index = status_options.index(hyp.status) if hyp.status in status_options else 0
                        new_status = st.selectbox(
                            "Status",
                            status_options,
                            index=status_index,
                            key=f"status_{hyp.id}"
                        )

                    with col_c:
                        reason = st.text_input("Reason", key=f"reason_{hyp.id}", placeholder="Why?")

                    if st.button("Update", key=f"update_{hyp.id}"):
                        memory.update_hypothesis(hyp.id, confidence=new_conf, status=new_status)

                        # Log to history
                        if hyp.id not in history:
                            history[hyp.id] = []
                        history[hyp.id].append({
                            "date": datetime.now().isoformat(),
                            "confidence": new_conf,
                            "status": new_status,
                            "event": reason or "updated"
                        })
                        memory.base_path.mkdir(parents=True, exist_ok=True)
                        history_file.write_text(json.dumps(history, indent=2))

                        st.success("✅ Updated")
                        get_memory.clear()
                        st.rerun()

# ══════════════════════════════════════════════════════════════════════════════
# TAB 3: DASHBOARD
# ══════════════════════════════════════════════════════════════════════════════
with tab3:
    st.markdown("### 📊 Research Dashboard")

    hypotheses = memory.get_hypotheses()

    # Load history
    history_file = memory.base_path / "hypothesis_history.json"
    try:
        history = json.loads(history_file.read_text()) if history_file.exists() else {}
    except Exception:
        history = {}

    if not hypotheses:
        st.info("Create some hypotheses to see the dashboard!")
    else:
        # Confidence distribution
        col1, col2 = st.columns(2)

        with col1:
            st.markdown("#### Hypothesis Status")

            status_counts = {}
            for h in hypotheses:
                status_counts[h.status] = status_counts.get(h.status, 0) + 1

            colors = {
                "active": "#00d4ff",
                "strengthened": "#00ff88",
                "weakened": "#ffaa00",
                "falsified": "#ff4444"
            }

            fig = go.Figure(data=[go.Pie(
                labels=list(status_counts.keys()),
                values=list(status_counts.values()),
                marker=dict(colors=[colors.get(s, "#888") for s in status_counts.keys()]),
                hole=0.4
            )])

            fig.update_layout(
                paper_bgcolor='rgba(0,0,0,0)',
                plot_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white'),
                height=300,
                margin=dict(l=20, r=20, t=30, b=20),
                showlegend=True
            )

            st.plotly_chart(fig, use_container_width=True)

        with col2:
            st.markdown("#### Confidence Distribution")

            confidences = [h.confidence for h in hypotheses]

            fig = go.Figure(data=[go.Histogram(
                x=confidences,
                nbinsx=10,
                marker_color='#00d4ff'
            )])

            fig.update_layout(
                xaxis_title="Confidence",
                yaxis_title="Count",
                paper_bgcolor='rgba(0,0,0,0)',
                plot_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white'),
                height=300,
                margin=dict(l=40, r=20, t=30, b=40),
                xaxis=dict(gridcolor='rgba(255,255,255,0.1)'),
                yaxis=dict(gridcolor='rgba(255,255,255,0.1)')
            )

            st.plotly_chart(fig, use_container_width=True)

        # Confidence over time
        st.markdown("#### Confidence Trajectories")

        if history:
            fig = go.Figure()

            for hyp_id, events in history.items():
                if len(events) > 0:
                    dates = []
                    for e in events:
                        d = e.get("date")
                        if d is None:
                            dates.append("")
                            continue
                        if isinstance(d, (int, float)):
                            try:
                                dates.append(datetime.fromtimestamp(d).strftime("%Y-%m-%d"))
                            except (ValueError, OSError):
                                dates.append("")
                        elif isinstance(d, str):
                            s = d.strip()
                            if len(s) >= 10 and s[4] == "-" and s[7] == "-":
                                dates.append(s[:10])
                            elif s.isdigit() or (s.replace(".", "", 1).replace("-", "", 1).isdigit()):
                                try:
                                    ts = float(s)
                                    if ts > 1e12:
                                        ts = ts / 1000.0
                                    dates.append(datetime.fromtimestamp(ts).strftime("%Y-%m-%d"))
                                except (ValueError, OSError):
                                    dates.append("")
                            else:
                                dates.append("")
                        else:
                            dates.append("")
                    confs = [e.get("confidence", 0) for e in events]

                    # Get hypothesis claim for label
                    hyp_obj = next((h for h in hypotheses if h.id == hyp_id), None)
                    label = f"{hyp_id}: {hyp_obj.claim[:30]}..." if hyp_obj else hyp_id

                    fig.add_trace(go.Scatter(
                        x=dates,
                        y=confs,
                        mode='lines+markers',
                        name=hyp_id,
                        hovertemplate=f"{label}<br>Confidence: %{{y:.0%}}<extra></extra>"
                    ))

            fig.update_layout(
                xaxis_title="Date",
                yaxis_title="Confidence",
                yaxis=dict(range=[0, 1], tickformat='.0%', gridcolor='rgba(255,255,255,0.1)'),
                xaxis=dict(type="category", gridcolor='rgba(255,255,255,0.1)', tickangle=-45),
                paper_bgcolor='rgba(0,0,0,0)',
                plot_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white'),
                height=400,
                margin=dict(l=40, r=20, t=30, b=40),
                legend=dict(
                    bgcolor='rgba(26,26,46,0.8)',
                    font=dict(size=10)
                )
            )

            st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("Update hypothesis confidence to see trajectories over time")

        # Summary metrics
        st.markdown("---")
        st.markdown("#### Key Metrics")

        m_col1, m_col2, m_col3, m_col4 = st.columns(4)

        with m_col1:
            avg_conf = sum(h.confidence for h in hypotheses) / len(hypotheses) if hypotheses else 0
            st.metric("Avg Confidence", f"{avg_conf:.0%}")

        with m_col2:
            high_conf = sum(1 for h in hypotheses if h.confidence >= 0.7)
            st.metric("High Confidence", high_conf)

        with m_col3:
            at_risk = sum(1 for h in hypotheses if h.confidence < 0.3)
            st.metric("At Risk", at_risk, delta=None if at_risk == 0 else "⚠️")

        with m_col4:
            falsified = sum(1 for h in hypotheses if h.status == "falsified")
            st.metric("Falsified", falsified)

# ══════════════════════════════════════════════════════════════════════════════
# TAB 4: WEEKLY BRIEF
# ══════════════════════════════════════════════════════════════════════════════
with tab4:
    st.markdown("### 📋 Weekly Research Brief")
    st.markdown("*AI-generated summary for your lab meeting*")

    col_gen, col_clear, col_options = st.columns([2, 1, 1])

    with col_gen:
        generate_brief = st.button("🚀 Generate Weekly Brief", type="primary", use_container_width=True, key="gen_brief_btn")

    with col_clear:
        clear_brief = st.button("Clear Weekly Brief", use_container_width=True, key="clear_brief_btn")

    with col_options:
        include_charts = st.checkbox("Include charts", value=True, key="brief_include_charts")

    if clear_brief and "current_brief" in st.session_state:
        del st.session_state["current_brief"]
        st.rerun()

    if generate_brief:
        with st.spinner("Analyzing research portfolio..."):
            brief = memory.generate_weekly_brief()
        st.session_state['current_brief'] = brief

    if 'current_brief' in st.session_state:
        brief = st.session_state['current_brief']
        hypotheses = memory.get_hypotheses()

        # Load history for charts
        history_file = memory.base_path / "hypothesis_history.json"
        try:
            history = json.loads(history_file.read_text()) if history_file.exists() else {}
        except Exception:
            history = {}

        # ══════════════════════════════════════════════════════════════════════
        # HEADER
        # ══════════════════════════════════════════════════════════════════════
        st.markdown(f"""
        <div style="background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); padding: 30px; border-radius: 15px; margin-bottom: 25px;">
            <h1 style="color: #00d4ff; margin: 0;">📋 Weekly Brief</h1>
            <p style="color: #888; margin: 10px 0 0 0; font-size: 1.1rem;">
                <b>{lab_name}</b> • Week {brief.week} • Generated {brief.generated[:10]}
            </p>
        </div>
        """, unsafe_allow_html=True)

        # ══════════════════════════════════════════════════════════════════════
        # QUICK STATS ROW
        # ══════════════════════════════════════════════════════════════════════
        stat_cols = st.columns(5)

        active_count = sum(1 for h in hypotheses if h.status == "active")
        strengthened = sum(1 for h in hypotheses if h.status == "strengthened")
        weakened = sum(1 for h in hypotheses if h.status == "weakened")
        falsified = sum(1 for h in hypotheses if h.status == "falsified")
        avg_conf = sum(h.confidence for h in hypotheses) / len(hypotheses) if hypotheses else 0

        with stat_cols[0]:
            st.metric("🔵 Active", active_count)
        with stat_cols[1]:
            st.metric("🟢 Strengthened", strengthened)
        with stat_cols[2]:
            st.metric("🟡 Weakened", weakened)
        with stat_cols[3]:
            st.metric("🔴 Falsified", falsified)
        with stat_cols[4]:
            st.metric("📊 Avg Confidence", f"{avg_conf:.0%}")

        st.markdown("---")

        # ══════════════════════════════════════════════════════════════════════
        # CHARTS ROW (if enabled)
        # ══════════════════════════════════════════════════════════════════════
        if include_charts and hypotheses:
            chart_col1, chart_col2 = st.columns(2)

            with chart_col1:
                st.markdown("#### Hypothesis Portfolio")

                fig = go.Figure()

                colors = {
                    "active": "#00d4ff",
                    "strengthened": "#00ff88",
                    "weakened": "#ffaa00",
                    "falsified": "#ff4444"
                }

                for i, h in enumerate(hypotheses):
                    fig.add_trace(go.Scatter(
                        x=[i],
                        y=[h.confidence],
                        mode='markers+text',
                        marker=dict(
                            size=30,
                            color=colors.get(h.status, "#888"),
                            line=dict(width=2, color='white')
                        ),
                        text=[h.id.split('-')[-1] if '-' in h.id else h.id],
                        textposition="middle center",
                        textfont=dict(size=10, color='white'),
                        name=h.status,
                        hovertemplate=f"<b>{h.id}</b><br>{h.claim[:50]}...<br>Confidence: {h.confidence:.0%}<extra></extra>",
                        showlegend=False
                    ))

                fig.update_layout(
                    xaxis=dict(
                        title="Hypothesis",
                        showgrid=False,
                        showticklabels=False
                    ),
                    yaxis=dict(
                        title="Confidence",
                        range=[0, 1],
                        tickformat=".0%",
                        gridcolor='rgba(255,255,255,0.1)'
                    ),
                    paper_bgcolor='rgba(0,0,0,0)',
                    plot_bgcolor='rgba(0,0,0,0)',
                    font=dict(color='white'),
                    height=250,
                    margin=dict(l=50, r=20, t=20, b=40)
                )

                st.plotly_chart(
                    fig,
                    use_container_width=True,
                    key="brief_portfolio_chart",
                    config={"displayModeBar": True, "displaylogo": False, "modeBarButtonsToRemove": ["select2d", "lasso2d"]},
                )

            with chart_col2:
                st.markdown("#### Confidence Trend")

                if history:
                    from datetime import datetime as dt
                    fig = go.Figure()

                    for hyp_id, events in list(history.items())[:5]:
                        if events:
                            # Normalize dates: ISO string, numeric timestamp, or skip non-dates
                            dates = []
                            for e in events[-10:]:
                                d = e.get("date")
                                if d is None:
                                    dates.append("")
                                    continue
                                if isinstance(d, (int, float)):
                                    try:
                                        dates.append(dt.fromtimestamp(d).strftime("%Y-%m-%d"))
                                    except (ValueError, OSError):
                                        dates.append("")
                                elif isinstance(d, str):
                                    s = d.strip()
                                    # ISO date (e.g. 2025-01-23 or 2025-01-23T...)
                                    if len(s) >= 10 and s[4] == "-" and s[7] == "-":
                                        dates.append(s[:10])
                                    # Numeric string (timestamp seconds or ms)
                                    elif s.isdigit() or (s.replace(".", "", 1).replace("-", "", 1).isdigit()):
                                        try:
                                            ts = float(s)
                                            if ts > 1e12:
                                                ts = ts / 1000.0
                                            dates.append(dt.fromtimestamp(ts).strftime("%Y-%m-%d"))
                                        except (ValueError, OSError):
                                            dates.append("")
                                    else:
                                        dates.append("")
                                else:
                                    dates.append("")
                            confs = [e.get("confidence", 0) for e in events[-10:]]

                            fig.add_trace(go.Scatter(
                                x=dates,
                                y=confs,
                                mode='lines+markers',
                                name=hyp_id.split('-')[-1] if '-' in hyp_id else hyp_id,
                                line=dict(width=2),
                                marker=dict(size=6)
                            ))

                    fig.update_layout(
                        xaxis=dict(
                            type="category",
                            title="Date",
                            gridcolor='rgba(255,255,255,0.1)',
                            tickangle=-45,
                        ),
                        yaxis=dict(
                            title="Confidence",
                            range=[0, 1],
                            tickformat=".0%",
                            gridcolor='rgba(255,255,255,0.1)'
                        ),
                        paper_bgcolor='rgba(0,0,0,0)',
                        plot_bgcolor='rgba(0,0,0,0)',
                        font=dict(color='white'),
                        height=250,
                        margin=dict(l=50, r=20, t=20, b=60),
                        legend=dict(
                            orientation="h",
                            yanchor="bottom",
                            y=1.02,
                            xanchor="right",
                            x=1,
                            bgcolor='rgba(0,0,0,0)'
                        )
                    )

                    st.plotly_chart(
                        fig,
                        use_container_width=True,
                        key="brief_confidence_trend_chart",
                        config={"displayModeBar": True, "displaylogo": False, "modeBarButtonsToRemove": ["select2d", "lasso2d"]},
                    )
                else:
                    st.info("Update hypotheses to see trends")

            st.markdown("---")

        # ══════════════════════════════════════════════════════════════════════
        # WHAT CHANGED
        # ══════════════════════════════════════════════════════════════════════
        st.markdown("### 🔄 What Changed This Week")

        if brief.what_changed and brief.what_changed[0] != "No significant changes this week":
            for item in brief.what_changed:
                st.markdown(f"""
                <div style="background: #252540; padding: 15px; border-radius: 8px; margin: 8px 0; border-left: 4px solid #00d4ff; color: #e8e8e8; font-size: 1rem;">
                    {item}
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("No significant changes this week")

        # ══════════════════════════════════════════════════════════════════════
        # HYPOTHESIS STATUS
        # ══════════════════════════════════════════════════════════════════════
        st.markdown("### 💡 Hypothesis Status")

        for status, emoji, color in [
            ("strengthened", "🟢", "#00ff88"),
            ("active", "🔵", "#00d4ff"),
            ("weakened", "🟡", "#ffaa00"),
            ("falsified", "🔴", "#ff4444")
        ]:
            status_hyps = [h for h in hypotheses if h.status == status]
            if status_hyps:
                st.markdown(f"**{emoji} {status.title()}**")
                for h in status_hyps:
                    st.markdown(f"""
                    <div style="background: #1e1e2e; padding: 12px 15px; border-radius: 8px; margin: 5px 0; border-left: 3px solid {color};">
                        <span style="color: {color}; font-weight: bold;">{h.confidence:.0%}</span>
                        <span style="color: #ccc; margin-left: 10px;">{h.claim[:80]}{'...' if len(h.claim) > 80 else ''}</span>
                    </div>
                    """, unsafe_allow_html=True)

        # ══════════════════════════════════════════════════════════════════════
        # ALERTS
        # ══════════════════════════════════════════════════════════════════════
        at_risk = [h for h in hypotheses if h.confidence < 0.3 and h.status == "active"]
        if at_risk:
            st.markdown("### ⚠️ Attention Needed")
            for h in at_risk:
                st.warning(f"**{h.id}** has low confidence ({h.confidence:.0%}): {h.claim[:60]}...")

        # ══════════════════════════════════════════════════════════════════════
        # DISCUSSION QUESTIONS
        # ══════════════════════════════════════════════════════════════════════
        st.markdown("### ❓ Questions for Discussion")

        for i, q in enumerate(brief.questions_for_pi, 1):
            st.markdown(f"""
            <div style="background: #1a1a2e; padding: 15px; border-radius: 8px; margin: 8px 0; display: flex; align-items: center;">
                <div style="background: #00d4ff; color: #1a1a2e; width: 28px; height: 28px; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-weight: bold; margin-right: 15px; flex-shrink: 0;">{i}</div>
                <span style="color: #e8e8e8;">{q}</span>
            </div>
            """, unsafe_allow_html=True)

        # ══════════════════════════════════════════════════════════════════════
        # EXPORT
        # ══════════════════════════════════════════════════════════════════════
        st.markdown("---")

        export_col1, export_col2, export_col3 = st.columns(3)

        with export_col1:
            st.download_button(
                "📄 Download Markdown",
                brief.to_markdown(),
                file_name=f"brief_{brief.week}.md",
                mime="text/markdown",
                use_container_width=True,
                key="dl_brief_md",
            )

        with export_col2:
            html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Weekly Brief - {brief.week}</title>
    <style>
        body {{ font-family: 'Segoe UI', Arial, sans-serif; max-width: 800px; margin: 40px auto; padding: 20px; background: #1a1a2e; color: #e8e8e8; }}
        h1 {{ color: #00d4ff; }}
        h2 {{ color: #00d4ff; border-bottom: 2px solid #333; padding-bottom: 10px; }}
        .header {{ background: linear-gradient(135deg, #252540 0%, #1a1a2e 100%); padding: 30px; border-radius: 15px; margin-bottom: 30px; }}
        .stat {{ display: inline-block; background: #252540; padding: 15px 25px; border-radius: 10px; margin: 5px; text-align: center; }}
        .stat-value {{ font-size: 2rem; font-weight: bold; color: #00d4ff; }}
        .stat-label {{ color: #888; font-size: 0.9rem; }}
        .item {{ background: #252540; padding: 15px; border-radius: 8px; margin: 10px 0; border-left: 4px solid #00d4ff; }}
        .hypothesis {{ padding: 12px; background: #1e1e2e; border-radius: 8px; margin: 8px 0; }}
        .confidence {{ font-weight: bold; }}
        .high {{ color: #00ff88; }}
        .medium {{ color: #ffaa00; }}
        .low {{ color: #ff4444; }}
        .question {{ display: flex; align-items: center; background: #1e1e2e; padding: 15px; border-radius: 8px; margin: 8px 0; }}
        .question-num {{ background: #00d4ff; color: #1a1a2e; width: 28px; height: 28px; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-weight: bold; margin-right: 15px; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>📋 Weekly Brief</h1>
        <p><b>{lab_name}</b> • Week {brief.week} • {brief.generated[:10]}</p>
    </div>
    <div style="text-align: center; margin-bottom: 30px;">
        <div class="stat"><div class="stat-value">{active_count}</div><div class="stat-label">Active</div></div>
        <div class="stat"><div class="stat-value">{strengthened}</div><div class="stat-label">Strengthened</div></div>
        <div class="stat"><div class="stat-value">{weakened}</div><div class="stat-label">Weakened</div></div>
        <div class="stat"><div class="stat-value">{avg_conf:.0%}</div><div class="stat-label">Avg Confidence</div></div>
    </div>
    <h2>🔄 What Changed</h2>
    {''.join(f'<div class="item">{item}</div>' for item in brief.what_changed)}
    <h2>💡 Hypotheses</h2>
    {''.join(f'<div class="hypothesis"><span class="confidence {"high" if h.confidence >= 0.7 else "medium" if h.confidence >= 0.4 else "low"}">{h.confidence:.0%}</span> - {h.claim[:80]}{"..." if len(h.claim) > 80 else ""}</div>' for h in hypotheses)}
    <h2>❓ Discussion Questions</h2>
    {''.join(f'<div class="question"><div class="question-num">{i}</div>{q}</div>' for i, q in enumerate(brief.questions_for_pi, 1))}
    <p style="text-align: center; color: #666; margin-top: 40px;">Generated by OMEGA Research Memory</p>
</body>
</html>
"""
            st.download_button(
                "🌐 Download HTML",
                html_content,
                file_name=f"brief_{brief.week}.html",
                mime="text/html",
                use_container_width=True,
                key="dl_brief_html",
            )

        with export_col3:
            brief_data = {
                "week": brief.week,
                "generated": brief.generated,
                "lab": lab_name,
                "stats": {
                    "active": active_count,
                    "strengthened": strengthened,
                    "weakened": weakened,
                    "falsified": falsified,
                    "avg_confidence": avg_conf
                },
                "what_changed": brief.what_changed,
                "hypotheses": [{"id": h.id, "claim": h.claim, "confidence": h.confidence, "status": h.status} for h in hypotheses],
                "questions": brief.questions_for_pi
            }

            st.download_button(
                "📊 Download JSON",
                json.dumps(brief_data, indent=2),
                file_name=f"brief_{brief.week}.json",
                mime="application/json",
                use_container_width=True,
                key="dl_brief_json",
            )

        # Footer
        st.markdown(f"""
        <div style="text-align: center; color: #666; margin-top: 30px; padding: 20px;">
            <p>Brief ID: {brief.id} • Sources: {len(brief.sources_used)} chunks analyzed</p>
        </div>
        """, unsafe_allow_html=True)

    else:
        st.markdown("""
        <div style="background: #1a1a2e; padding: 60px; border-radius: 15px; text-align: center; border: 2px dashed #333; margin-top: 20px;">
            <h2 style="color: #00d4ff;">Ready to Generate</h2>
            <p style="color: #888;">Click the button above to create your weekly research brief with charts and insights.</p>
        </div>
        """, unsafe_allow_html=True)
