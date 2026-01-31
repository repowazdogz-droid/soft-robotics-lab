#!/usr/bin/env python3
"""
OMEGA Breakthrough Engine — Web UI (Streamlit).
Dashboard, single hypothesis view, create form, Tutor link, graph visualization.
"""

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_BREAKTHROUGH_DIR = Path(__file__).resolve().parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BREAKTHROUGH_DIR) not in sys.path:
    sys.path.insert(0, str(_BREAKTHROUGH_DIR))

import streamlit as st
from hypothesis_ledger import HypothesisLedger, HypothesisStatus

try:
    from substrate_integration import find_similar_hypotheses, get_hypothesis_lineage, get_related_concepts_from_graph
except ImportError:
    find_similar_hypotheses = lambda q, n=3: []
    get_hypothesis_lineage = lambda h: []
    get_related_concepts_from_graph = lambda h: []

try:
    from core.visualize import generate_hypothesis_graph, generate_html_graph
except ImportError:
    generate_hypothesis_graph = lambda h: None
    generate_html_graph = lambda h: "<p>Graph unavailable</p>"

try:
    from shared.tutor_links import get_tutor_link
except ImportError:
    get_tutor_link = lambda topic, port=8503: f"http://localhost:{port}/?topic={topic.replace(' ', '+')}"

st.set_page_config(page_title="Breakthrough Engine", page_icon="🧪", layout="wide")

ledger = HypothesisLedger()

# ----- State -----
if "view_hyp_id" not in st.session_state:
    st.session_state["view_hyp_id"] = None
if "show_create" not in st.session_state:
    st.session_state["show_create"] = False

# ----- Sidebar: Hypothesis Health -----
st.sidebar.divider()
st.sidebar.subheader("📊 Hypothesis Health")
if st.sidebar.button("Run Health Check"):
    report = ledger.get_health_report()
    st.session_state["health_report"] = report
    st.rerun()

# ----- Dashboard -----
st.title("BREAKTHROUGH ENGINE")
st.caption("Hypothesis ledger with OMEGA-MAX alignment and substrate integration.")

all_hyps = ledger.list()
active = [h for h in all_hyps if h.status == HypothesisStatus.ACTIVE]
killed = [h for h in all_hyps if h.status == HypothesisStatus.KILLED]
validated = [h for h in all_hyps if h.status in (HypothesisStatus.STRENGTHENED,)]

col1, col2, col3, col4 = st.columns(4)
with col1:
    st.metric("Active Hypotheses", len(active))
with col2:
    st.metric("Killed", len(killed))
with col3:
    st.metric("Validated", len(validated))
with col4:
    if st.button("+ New Hypothesis"):
        st.session_state["show_create"] = True
        st.session_state["view_hyp_id"] = None
        st.rerun()

# ----- Hypothesis Health Report -----
if "health_report" in st.session_state:
    report = st.session_state["health_report"]
    summary = report["summary"]
    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Total", summary["total"])
    col2.metric("Healthy", summary["healthy"], delta_color="normal")
    col3.metric("Stale", summary["stale"], delta=summary["stale"] if summary["stale"] > 0 else None, delta_color="inverse")
    col4.metric("Zombies", summary["zombies"], delta=summary["zombies"] if summary["zombies"] > 0 else None, delta_color="inverse")
    if summary["action_required"] > 0:
        st.warning(f"⚠️ {summary['action_required']} hypotheses need attention")
        for result in report["results"]:
            if result.is_zombie or result.is_stale:
                with st.expander(f"{'🧟' if result.is_zombie else '⏰'} {result.hypothesis_id}"):
                    st.write(f"**Original confidence:** {result.original_confidence:.0%}")
                    st.write(f"**Decayed confidence:** {result.decayed_confidence:.0%}")
                    st.write(f"**Days since evidence:** {result.days_since_evidence}")
                    st.info(result.recommendation)
                    c1, c2 = st.columns(2)
                    with c1:
                        if st.button("Kill", key=f"kill_{result.hypothesis_id}"):
                            ledger.kill(result.hypothesis_id, "Killed due to decay")
                            st.session_state.pop("health_report", None)
                            st.rerun()
                    with c2:
                        if st.button("Review", key=f"review_{result.hypothesis_id}"):
                            st.session_state["view_hyp_id"] = result.hypothesis_id
                            st.rerun()
    st.divider()

# ----- Breakthrough detection -----
st.subheader("🎯 Breakthroughs")
breakthroughs = ledger.get_breakthroughs()
near = ledger.get_near_breakthroughs()
if breakthroughs:
    st.success(f"🎯 {len(breakthroughs)} BREAKTHROUGH(S) DETECTED!")
    for bt in breakthroughs:
        h = bt["hypothesis"]
        claim_preview = (h.claim[:50] + "...") if len(h.claim) > 50 else h.claim
        with st.expander(f"🎯 {h.id}: {claim_preview}"):
            st.write(f"**Confidence:** {h.confidence:.0%}")
            st.write("**Why it's a breakthrough:**")
            for reason in bt["reasons"]:
                st.write(f"✅ {reason}")
            st.info("**Recommended:** Proceed to prototype, publication, or patent application")
else:
    st.info("No breakthroughs yet. Keep testing hypotheses!")
if near:
    st.warning(f"📍 {len(near)} hypothesis(es) close to breakthrough")
    for n in near:
        h = n["hypothesis"]
        claim_preview = (h.claim[:50] + "...") if len(h.claim) > 50 else h.claim
        with st.expander(f"📍 {h.id}: {claim_preview}"):
            st.write("**Already achieved:**")
            for reason in n["reasons"]:
                st.write(f"✅ {reason}")
            st.write("**Still needed:**")
            for m in n["missing"]:
                st.write(f"❌ {m}")
st.divider()

# ----- Create form -----
if st.session_state.get("show_create"):
    st.subheader("Create hypothesis")
    with st.form("create_hypothesis"):
        claim = st.text_area("Claim (required)", placeholder="Precise causal claim")
        domain = st.text_input("Domain", value="general", placeholder="e.g. soft_robotics, drug_discovery")
        horizon = st.selectbox("Temporal horizon (T1–T4)", ["t1", "t2", "t3", "t4"], format_func=lambda x: {"t1": "T1 (0–3mo)", "t2": "T2 (3–12mo)", "t3": "T3 (1–5yr)", "t4": "T4 (5+yr)"}.get(x, x))
        substrates = st.multiselect("Substrates affected", ["materials", "compute", "bio", "mfg", "coordination", "env"])
        falsification_cost = st.selectbox("Falsification cost", ["low", "medium", "high"])
        who_benefits = st.text_input("Who benefits (comma-separated)", placeholder="e.g. Lab, End users")
        who_loses = st.text_input("Who loses (comma-separated)", placeholder="e.g. Incumbent")
        next_step = st.text_input("Next step (required)", placeholder="Concrete action")
        submitted = st.form_submit_button("Create")
    if submitted:
        if not claim or not claim.strip():
            st.error("Claim is required.")
        elif not next_step or not next_step.strip():
            st.error("Next step is required for active hypotheses.")
        else:
            similar = ledger.suggest_similar(claim.strip(), n=3)
            if similar:
                st.info("Similar hypotheses: " + ", ".join([s.get("metadata", {}).get("hypothesis_id", s.get("id", "")) for s in similar]))
            try:
                h = ledger.create(
                    claim.strip(),
                    domain.strip() or "general",
                    horizon=horizon,
                    substrates=substrates,
                    falsification_cost=falsification_cost,
                    who_benefits=[x.strip() for x in who_benefits.split(",") if x.strip()] if who_benefits else None,
                    who_loses=[x.strip() for x in who_loses.split(",") if x.strip()] if who_loses else None,
                    next_step=next_step.strip(),
                )
                st.success(f"Created {h.id}")
                st.session_state["show_create"] = False
                st.session_state["view_hyp_id"] = h.id
                st.rerun()
            except ValueError as e:
                st.error(str(e))
    if st.button("Cancel", key="cancel_create"):
        st.session_state["show_create"] = False
        st.rerun()
    st.stop()

# ----- Single hypothesis view -----
if st.session_state.get("view_hyp_id"):
    h = ledger.get(st.session_state["view_hyp_id"])
    if not h:
        st.session_state["view_hyp_id"] = None
        st.rerun()
    else:
        if st.button("← Back to dashboard"):
            st.session_state["view_hyp_id"] = None
            st.rerun()
        st.subheader(h.id)
        st.markdown(f"**Claim:** {h.claim}")
        st.caption(f"Domain: {h.domain} | Status: {h.status.value} | SRFC: {h.srfc_status} | VRFC: {h.vrfc_status} | Falsification: {h.falsification_cost}")
        st.caption("SRFC = Can it work? | VRFC = Will it survive reality? (See shared/docs/GLOSSARY.md)")
        st.markdown(f"**Next step:** {h.next_step or '—'}")

        # Falsification cost estimate
        if "falsification_estimate" not in st.session_state or st.session_state.get("falsification_estimate_hyp_id") != h.id:
            st.session_state["falsification_estimate"] = None
            st.session_state["falsification_estimate_hyp_id"] = h.id
        if st.button("Estimate Falsification Cost", key="est_falsify"):
            est = ledger.get_falsification_estimate(h.id)
            st.session_state["falsification_estimate"] = est
            st.session_state["falsification_estimate_hyp_id"] = h.id
            st.rerun()
        if st.session_state.get("falsification_estimate") and st.session_state.get("falsification_estimate_hyp_id") == h.id:
            estimate = st.session_state["falsification_estimate"]
            st.subheader("Falsification Paths")
            if estimate["worth_testing"]:
                st.success(f"✅ Worth testing: {estimate['reason']}")
            else:
                st.warning(f"⚠️ May not be worth testing: {estimate['reason']}")
            st.write(f"**Recommended:** {estimate['recommended']}")
            for path in estimate["paths"]:
                icon = "⭐" if path["recommended"] else ""
                with st.expander(f"{icon} {path['type'].replace('_', ' ').title()}"):
                    st.write(path["description"])
                    st.write(f"**Cost:** {path['cost'][0]:,} - {path['cost'][1]:,} USD")
                    st.write(f"**Time:** {path['time'][0]} - {path['time'][1]} days")
                    st.write(f"**Resolution probability:** {path['resolution_prob']:.0%}")
                    st.write(f"**Resources:** {', '.join(path['resources'])}")
        st.markdown(f"**Confidence:** {h.confidence:.0%} ({h.confidence_low:.0%}–{h.confidence_high:.0%})")
        st.markdown("**Who benefits:** " + ", ".join(h.who_benefits or []) or "—")
        st.markdown("**Who loses:** " + ", ".join(h.who_loses or []) or "—")
        st.markdown("**Substrates:** " + ", ".join(h.substrates or []) or "—")
        st.markdown("**Horizon:** " + {"t1": "T1 (0–3mo)", "t2": "T2 (3–12mo)", "t3": "T3 (1–5yr)", "t4": "T4 (5+yr)"}.get(h.temporal_horizon or "t2", h.temporal_horizon or "—"))
        tutor_url = get_tutor_link(h.domain or h.claim[:50])
        st.link_button("📚 Learn about [domain]", tutor_url)
        with st.expander("Timeline"):
            for entry in ledger.get_history(h.id):
                st.caption(f"{entry.get('timestamp', '')[:16]} — {entry.get('action', '')}: {entry.get('reason', '')}")
        with st.expander("Related hypotheses"):
            similar = find_similar_hypotheses(h.claim, n=5)
            for s in similar:
                sid = s.get("metadata", {}).get("hypothesis_id") or s.get("id", "")
                if sid != h.id:
                    st.caption(f"- {sid}: {str(s.get('text', ''))[:60]}…")
        with st.expander("Related concepts (knowledge graph)"):
            related = get_related_concepts_from_graph(h.id)
            if related:
                for r in related:
                    st.caption(f"- {r.get('node_id', r.get('node', {}).get('id', ''))}")
            else:
                st.caption("—")
        with st.expander("Update / Kill"):
            if st.session_state.get("show_kill_form"):
                reason = st.text_input("Reason for killing", key="kill_reason_input")
                c1, c2 = st.columns(2)
                with c1:
                    if st.button("Confirm kill", key="confirm_kill") and reason:
                        ledger.kill(h.id, reason)
                        st.session_state["view_hyp_id"] = None
                        st.session_state["show_kill_form"] = False
                        st.rerun()
                with c2:
                    if st.button("Cancel", key="cancel_kill"):
                        st.session_state["show_kill_form"] = False
                        st.rerun()
            else:
                if st.button("Kill hypothesis", key="kill_hyp"):
                    st.session_state["show_kill_form"] = True
                    st.rerun()
        st.stop()

# ----- Hypothesis cards -----
st.subheader("Hypotheses")
with st.expander("What is SRFC / VRFC? (Translation Trinity)"):
    st.caption("**SRFC** = Soft Robotics Feasibility Compiler — *Can it work physically?* (geometry, actuation, materials, safety). **VRFC** = Validation & Risk Feasibility Compiler — *Will it survive reality?* (evidence, regulation, reimbursement, adoption). Full glossary: products/shared/docs/GLOSSARY.md")
for h in (active + [x for x in all_hyps if x not in active])[:50]:
    with st.container():
        st.markdown(f"**{h.id}** — {h.claim[:80]}{'…' if len(h.claim) > 80 else ''}")
        st.caption(f"Domain: {h.domain} | Status: {h.status.value} | SRFC: {h.srfc_status} | VRFC: {h.vrfc_status} | Falsification: {h.falsification_cost} | Next: {h.next_step[:40] if h.next_step else '—'}")
        c1, c2, c3, _ = st.columns([1, 1, 1, 2])
        with c1:
            if st.button("View", key=f"view_{h.id}"):
                st.session_state["view_hyp_id"] = h.id
                st.rerun()
        with c2:
            if st.button("Update", key=f"upd_{h.id}"):
                st.session_state["view_hyp_id"] = h.id
                st.rerun()
        with c3:
            if h.status == HypothesisStatus.ACTIVE and st.button("Kill", key=f"kill_{h.id}"):
                st.session_state["view_hyp_id"] = h.id
                st.session_state["show_kill_form"] = True
                st.rerun()
        st.divider()

# ----- Graph -----
with st.expander("Hypothesis relationship graph"):
    try:
        html = generate_html_graph(all_hyps)
        if html and "unavailable" not in html.lower():
            st.components.v1.html(html, height=450)
        else:
            fig = generate_hypothesis_graph(all_hyps)
            if fig is not None:
                st.pyplot(fig)
    except Exception as e:
        st.caption(f"Graph unavailable: {e}")
