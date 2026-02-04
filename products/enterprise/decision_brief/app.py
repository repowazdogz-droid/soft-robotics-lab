#!/usr/bin/env python3
"""
OMEGA Decision Brief — Web UI (Streamlit).
Clean form, real-time analysis, structured output, export, history from vector store.
"""

import os
import sys
from pathlib import Path

_DECISION_BRIEF_DIR = Path(__file__).resolve().parent
_PRODUCTS = _DECISION_BRIEF_DIR.parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
if str(_DECISION_BRIEF_DIR) not in sys.path:
    sys.path.insert(0, str(_DECISION_BRIEF_DIR))

import streamlit as st
from decision_brief import (
    generate_brief,
    TEMPORAL_LABELS,
    generate_go_no_go_brief,
    GO_NO_GO_CRITERIA_TEMPLATES,
    GoNoGoBrief,
)
from domains import load_domain_model, list_domains
try:
    from substrate_integration import search_past_decisions
except ImportError:
    search_past_decisions = lambda q, n=10: []

st.set_page_config(page_title="OMEGA Decision Brief", page_icon="📋", layout="wide")

st.title("📋 OMEGA Decision Brief")
st.caption("Domain-aware decision briefs with temporal analysis, stakeholders, scenarios, and substrate context.")

# ----- Input form -----
with st.form("brief_form"):
    st.subheader("Decision input")
    question = st.text_input(
        "Decision question",
        placeholder="e.g. Should we prioritize soft robotics or enterprise tools for Q1?",
        help="Natural language decision or comparison.",
    )
    domains_available = list_domains()
    domain_options = ["Auto-detect"] + [f"{d['name']} ({d['id']})" for d in domains_available]
    domain_ids = ["auto"] + [d["id"] for d in domains_available]
    domain_sel = st.selectbox("Domain", range(len(domain_options)), format_func=lambda i: domain_options[i])
    domain_override = domain_ids[domain_sel] if domain_sel < len(domain_ids) else "auto"
    horizon = st.number_input("Time horizon (months)", min_value=1, max_value=60, value=12, step=1)
    budget = st.number_input("Budget (optional)", min_value=0, value=0, step=1000, help="Leave 0 if N/A")
    submitted = st.form_submit_button("Generate brief")

if submitted and question and question.strip():
    params = {"horizon": horizon}
    if budget > 0:
        params["budget"] = budget
    domain_hint = None if domain_override == "auto" else domain_override

    with st.spinner("Generating brief…"):
        try:
            brief = generate_brief(question.strip(), params=params, enrich=True)
            st.session_state["last_brief"] = brief
            st.session_state["last_question"] = question.strip()
        except Exception as e:
            st.error(f"Generation failed: {e}")
            st.stop()

    b = st.session_state["last_brief"]

    # ----- Structured output -----
    st.divider()
    st.subheader("RECOMMENDATION")
    st.markdown(f"**{getattr(b, 'recommendation_one_line', '') or f'[{b.overall_status}] ' + (b.recommended_next_action or '—')}**")

    with st.expander("TEMPORAL ANALYSIS", expanded=True):
        for key, label in TEMPORAL_LABELS.items():
            d = getattr(b, f"{key}_implications", {}) or {}
            imp = d.get("implications", [])
            risks = d.get("risks", [])
            if isinstance(imp, list):
                imp = "; ".join(str(x) for x in imp[:5])
            if isinstance(risks, list):
                risks = "; ".join(str(x) for x in risks[:3])
            st.markdown(f"**{label}**")
            st.caption(f"Actions: {imp}")
            if risks:
                st.caption(f"Risks: {risks}")
            st.markdown("")

    with st.expander("STAKEHOLDERS", expanded=True):
        st.markdown("- **Pays:** " + ("; ".join(b.who_pays) if b.who_pays else "—"))
        st.markdown("- **Benefits:** " + ("; ".join(b.who_benefits) if b.who_benefits else "—"))
        st.markdown("- **Loses:** " + ("; ".join(b.who_loses) if b.who_loses else "—"))
        who_dec = getattr(b, "who_decides", None) or []
        st.markdown("- **Decides:** " + ("; ".join(who_dec) if who_dec else "—"))
        hidden = getattr(b, "hidden_stakeholders", None) or []
        st.markdown("- **Hidden:** " + ("; ".join(hidden) if hidden else "—"))

    with st.expander("SCENARIOS", expanded=True):
        for s in (getattr(b, "scenarios", None) or []):
            pct = int((s.get("probability") or 0) * 100)
            st.markdown(f"**{s.get('name', '')} ({pct}%):** {s.get('description', '')}")
            st.caption("Signals: " + ", ".join(s.get("signals_to_watch", [])[:3]))
            st.markdown("")

    with st.expander("RELATED CONTEXT"):
        past = getattr(b, "related_past_decisions", None) or []
        st.markdown("**Past decisions:** " + (", ".join([str(r.get("id", r.get("text", ""))[:40]) for r in past[:5]]) or "—"))
        rel = getattr(b, "related_knowledge", None) or []
        st.markdown("**Related knowledge:** " + (", ".join(rel[:5]) or "—"))

    with st.expander("VALIDATION (Translation Trinity)"):
        st.caption("**SRFC** = Can it work physically? | **TSRFC** = What does it replace? (workflow) | **VRFC** = Will it survive reality? (evidence, regulation, reimbursement)")
        st.markdown(f"- **SRFC:** {b.srfc_status} — {b.srfc_reason or '—'}")
        st.markdown(f"- **TSRFC:** {b.tsrfc_status} — {b.tsrfc_reason or '—'}")
        st.markdown(f"- **VRFC:** {b.vrfc_status} — {b.vrfc_reason or '—'}")
        st.caption("Full definitions: products/shared/docs/GLOSSARY.md")

    # ----- Export -----
    st.divider()
    col1, col2, col3 = st.columns(3)
    with col1:
        md = b.to_markdown()
        st.download_button("Download Markdown", md, file_name=f"decision_brief_{b.id}.md", mime="text/markdown")
    with col2:
        js = b.to_json()
        st.download_button("Download JSON", js, file_name=f"decision_brief_{b.id}.json", mime="application/json")
    with col3:
        st.caption(f"ID: {b.id}")

else:
    # ----- History (past decisions from vector store) -----
    st.divider()
    st.subheader("Past decisions")
    try:
        recent = search_past_decisions("decision brief strategy robotics business", n=10)
        if recent:
            for r in recent[:5]:
                st.caption(f"- {r.get('id', '')} — {str(r.get('text', ''))[:80]}…")
        else:
            st.caption("No past decisions in store yet. Generate a brief to populate history.")
    except Exception:
        st.caption("History unavailable (substrate not configured).")

# ----- Go/No-Go Decision (always visible) -----
st.divider()
st.header("⚡ Go/No-Go Decision")

go_nogo_title = st.text_input("Decision Title", placeholder="e.g., Soft Gripper Project Phase 2", key="gonogo_title")
go_nogo_question = st.text_input("Decision Question", placeholder="e.g., Should we proceed to prototype phase?", key="gonogo_question")

template = st.selectbox(
    "Template",
    list(GO_NO_GO_CRITERIA_TEMPLATES.keys()),
    format_func=lambda x: x.replace("_", " ").title(),
    key="gonogo_template",
)

st.subheader("Score Each Criterion")
st.caption("Rate each criterion from 0% (not met) to 100% (fully met)")

template_criteria = GO_NO_GO_CRITERIA_TEMPLATES[template]
scores = {}
evidence = {}

for tc in template_criteria:
    col1, col2 = st.columns([1, 2])
    with col1:
        scores[tc["name"]] = st.slider(
            tc["name"],
            min_value=0.0,
            max_value=1.0,
            value=0.5,
            format="%.0f%%",
            help=tc["description"],
            key=f"gonogo_slider_{tc['name']}",
        )
    with col2:
        evidence[tc["name"]] = st.text_input(
            f"Evidence for {tc['name']}",
            placeholder="Brief evidence or rationale",
            key=f"gonogo_evidence_{tc['name']}",
        )

# SRFC/VRFC override if available
col1, col2 = st.columns(2)
with col1:
    srfc_override = st.selectbox("SRFC Status (optional)", [None, "GREEN", "AMBER", "RED"], key="gonogo_srfc")
with col2:
    vrfc_override = st.selectbox("VRFC Status (optional)", [None, "GREEN", "AMBER", "RED"], key="gonogo_vrfc")

threshold = st.slider("GO Threshold", min_value=0.5, max_value=0.9, value=0.65, format="%.0f%%", key="gonogo_threshold")

if st.button("Generate Go/No-Go Brief", key="generate_gonogo"):
    if not go_nogo_title or not go_nogo_question:
        st.warning("Please provide title and question")
    else:
        brief = generate_go_no_go_brief(
            title=go_nogo_title,
            question=go_nogo_question,
            template=template,
            scores=scores,
            evidence=evidence,
            overall_threshold=threshold,
            srfc_status=srfc_override,
            vrfc_status=vrfc_override,
        )
        st.session_state["go_no_go_brief"] = brief

if "go_no_go_brief" in st.session_state:
    brief = st.session_state["go_no_go_brief"]

    # Display recommendation prominently
    if brief.recommendation == "GO":
        st.success(f"## ✅ Recommendation: GO\n**Confidence:** {brief.confidence:.0%}")
    else:
        st.error(f"## ❌ Recommendation: NO-GO\n**Confidence:** {brief.confidence:.0%}")

    # Score summary
    col1, col2, col3 = st.columns(3)
    col1.metric("Overall Score", f"{brief.overall_score:.0%}")
    col2.metric("Threshold", f"{brief.threshold:.0%}")
    col3.metric("Criteria Passed", f"{brief.criteria_passed}/{brief.criteria_total}")

    # Criteria table
    st.subheader("Criteria Assessment")
    for c in brief.criteria:
        status = "✅" if c.passed else "❌"
        with st.expander(f"{status} {c.name} — {c.score:.0%}"):
            st.write(f"**Threshold:** {c.threshold:.0%} | **Weight:** {c.weight:.0%}")
            st.write(c.description)
            st.write(f"**Evidence:** {c.evidence}")

    # Risks and conditions
    if brief.key_risks:
        st.subheader("⚠️ Key Risks")
        for risk in brief.key_risks:
            st.write(f"- {risk}")

    if brief.conditions:
        st.subheader("📋 Conditions for GO")
        for cond in brief.conditions:
            st.write(f"- {cond}")

    # Export
    st.divider()
    col1, col2 = st.columns(2)
    with col1:
        st.download_button(
            "📄 Download Markdown",
            brief.to_markdown(),
            file_name=f"go_no_go_{brief.title.lower().replace(' ', '_')}.md",
            mime="text/markdown",
            key="dl_gonogo_md",
        )
    with col2:
        import json
        st.download_button(
            "📊 Download JSON",
            json.dumps(brief.to_dict(), indent=2),
            file_name=f"go_no_go_{brief.title.lower().replace(' ', '_')}.json",
            mime="application/json",
            key="dl_gonogo_json",
        )
