"""
OMEGA Scientist - Paper Analysis & Discovery Engine
"""
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import streamlit as st

try:
    from core.paper_parser import parse_paper, parse_from_doi, save_parsed, load_parsed, ParsedPaperOutput
except ImportError:
    parse_paper = None
    parse_from_doi = None
    save_parsed = None
    load_parsed = None
    ParsedPaperOutput = None

# Validation Trinity from Decision Brief (SRFC/TSRFC/VRFC)
sys.path.insert(0, str(_ROOT.parent))
try:
    from enterprise.decision_brief.decision_brief import _compute_validation_trinity
    VALIDATION_TRINITY_AVAILABLE = True
except ImportError:
    VALIDATION_TRINITY_AVAILABLE = False

try:
    from integration.ledger_integration import (
        bulk_create_from_scientist_session,
        create_hypothesis_from_contradiction,
        create_hypothesis_from_cross_domain,
        create_hypothesis_from_failure,
        create_hypothesis_from_revival,
        LEDGER_AVAILABLE,
    )
except ImportError:
    LEDGER_AVAILABLE = False
    bulk_create_from_scientist_session = None
    create_hypothesis_from_contradiction = None
    create_hypothesis_from_cross_domain = None
    create_hypothesis_from_failure = None
    create_hypothesis_from_revival = None

st.set_page_config(page_title="OMEGA Scientist", page_icon="🔬", layout="wide")

st.title("🔬 OMEGA Scientist")
st.caption("Discovery + Validation + Translation")

# Dashboard summary
parsed_dir_dash = _ROOT / "outputs" / "parsed"
parsed_count = len(list(parsed_dir_dash.glob("*.json"))) if parsed_dir_dash.exists() else 0
hyp_count = len(st.session_state.get("ranked_hypotheses", []))
_cont_report = st.session_state.get("contradiction_report")
contradiction_count = len(_cont_report.contradictions) if _cont_report else 0
_trans_report = st.session_state.get("translation_report")
revival_count = len(_trans_report.revival_candidates) if _trans_report else 0

col1, col2, col3, col4 = st.columns(4)
with col1:
    st.metric("Papers Parsed", parsed_count)
with col2:
    st.metric("Hypotheses Ranked", hyp_count)
with col3:
    st.metric("Contradictions", contradiction_count)
with col4:
    st.metric("Revival Candidates", revival_count)
st.divider()

tab1, tab2, tab3 = st.tabs(["📄 Parse Paper", "🔍 Discover", "✅ Validate"])

with tab1:
    st.header("Parse Paper")

    upload = st.file_uploader("Upload PDF", type=["pdf"])
    doi_input = st.text_input("Or enter DOI", placeholder="10.1038/s41586-023-00001-1")

    do_claims = st.checkbox("Extract claims", value=True)
    do_data = st.checkbox("Find dataset links", value=True)

    if st.button("Parse Paper"):
        if upload:
            st.info("Parsing uploaded PDF...")
            try:
                papers_dir = _ROOT / "data" / "papers"
                papers_dir.mkdir(parents=True, exist_ok=True)
                path = papers_dir / upload.name
                with open(path, "wb") as f:
                    f.write(upload.getvalue())
                if parse_paper and save_parsed:
                    out = parse_paper(str(path), do_extract_claims=do_claims, find_data=do_data)
                    st.session_state["last_parsed"] = out
                    out_dir = _ROOT / "outputs" / "parsed"
                    out_dir.mkdir(parents=True, exist_ok=True)
                    json_path = out_dir / (path.stem + "_parsed.json")
                    save_parsed(out, str(json_path))
                    st.success(f"Parsed: **{out.paper.title}**")
                    st.session_state["last_parsed_path"] = str(json_path)
                else:
                    st.error("Parser not available. Install core dependencies.")
            except Exception as e:
                st.error(str(e))
        elif doi_input:
            st.info(f"Fetching and parsing DOI: {doi_input}")
            if parse_from_doi:
                out = parse_from_doi(doi_input)
                if out:
                    st.session_state["last_parsed"] = out
                    st.success("Parsed from DOI.")
                else:
                    st.warning("DOI PDF fetch not available. Upload the PDF instead.")
            else:
                st.warning("DOI parsing not available. Upload the PDF instead.")
        else:
            st.warning("Upload a PDF or enter a DOI")

    if st.session_state.get("last_parsed"):
        out = st.session_state["last_parsed"]
        st.markdown("---")
        st.subheader("Parsed result")
        st.markdown(f"**Title:** {out.paper.title}")
        st.caption(f"Authors: {', '.join(out.paper.authors[:5])}{'…' if len(out.paper.authors) > 5 else ''}")
        st.markdown("**Abstract**")
        st.caption(out.paper.abstract[:1500] + ("…" if len(out.paper.abstract) > 1500 else ""))
        st.markdown("**Claims**")
        for i, c in enumerate(out.claims[:15]):
            st.caption(f"{i+1}. [{c.claim_type.value}] {c.text[:200]}{'…' if len(c.text) > 200 else ''}")
        if len(out.claims) > 15:
            st.caption(f"… and {len(out.claims) - 15} more")
        st.markdown("**Dataset links**")
        for d in out.datasets:
            st.markdown(f"- **{d.repository}:** [{d.accession}]({d.url}) ({d.data_type})")
        if not out.datasets:
            st.caption("No dataset links found.")
        if st.session_state.get("last_parsed_path"):
            st.caption(f"Saved to: `{st.session_state['last_parsed_path']}`")

with tab2:
    st.header("🔍 Discover")

    discover_mode = st.selectbox(
        "Discovery Mode",
        ["Contradiction Mining", "Cross-Domain Collision", "Failure Analysis", "Translation Gap"],
        key="discover_mode",
    )

    if discover_mode == "Contradiction Mining":
        st.subheader("Contradiction Mining")
        st.caption("Find where papers disagree — contradictions are discovery opportunities")

        uploaded_papers = st.file_uploader(
            "Upload PDFs to compare",
            type=["pdf"],
            accept_multiple_files=True,
            key="contradiction_pdfs",
        )

        parsed_dir = _ROOT / "outputs" / "parsed"
        existing_parsed = list(parsed_dir.glob("*.json")) if parsed_dir.exists() else []
        selected_parsed = []
        if existing_parsed:
            st.write("Or use previously parsed papers:")
            selected_parsed = st.multiselect(
                "Select parsed papers",
                [p.name for p in existing_parsed],
                key="selected_parsed",
            )

        use_llm = st.checkbox("Use LLM for semantic comparison", value=True, key="use_llm_contradiction")

        if st.button("Find Contradictions", key="find_contradictions"):
            claims_by_paper = {}

            if uploaded_papers:
                papers_dir = _ROOT / "data" / "papers"
                papers_dir.mkdir(parents=True, exist_ok=True)
                for pdf in uploaded_papers:
                    with st.spinner(f"Parsing {pdf.name}..."):
                        temp_path = papers_dir / pdf.name
                        temp_path.write_bytes(pdf.read())
                        if parse_paper:
                            result = parse_paper(str(temp_path))
                            claims_by_paper[pdf.name] = result.claims
                        else:
                            st.error("Parser not available.")
                            break

            if selected_parsed and load_parsed:
                for name in selected_parsed:
                    path = parsed_dir / name
                    if path.exists():
                        result = load_parsed(str(path))
                        claims_by_paper[name] = result.claims

            if len(claims_by_paper) < 2:
                st.warning("Need at least 2 papers to find contradictions.")
            else:
                try:
                    from core.contradiction_miner import (
                        find_contradictions,
                        summarize_contradictions,
                    )

                    with st.spinner("Mining contradictions..."):
                        report = find_contradictions(claims_by_paper, use_llm=use_llm)

                    st.session_state["contradiction_report"] = report
                    st.success(f"Found {len(report.contradictions)} contradictions!")
                except Exception as e:
                    st.error(str(e))

        if st.session_state.get("contradiction_report"):
            report = st.session_state["contradiction_report"]
            st.markdown(summarize_contradictions(report))

            for i, c in enumerate(report.contradictions):
                with st.expander(
                    f"Contradiction {i+1}: {c.contradiction_type.value} (Potential: {c.discovery_potential:.0%})"
                ):
                    col1, col2 = st.columns(2)
                    with col1:
                        st.markdown(f"**{c.paper_a}**")
                        st.info(c.claim_a.text)
                    with col2:
                        st.markdown(f"**{c.paper_b}**")
                        st.info(c.claim_b.text)

                    st.markdown(f"**Analysis:** {c.description}")
                    if c.resolution_hypothesis:
                        st.markdown(f"**Resolution Hypothesis:** {c.resolution_hypothesis}")

                    col1, col2, col3 = st.columns(3)
                    with col1:
                        if st.button("Create Hypothesis", key=f"hyp_{i}"):
                            st.session_state["pending_hypothesis"] = c
                    with col2:
                        if st.button("VRFC Check", key=f"vrfc_{i}"):
                            st.info("VRFC integration coming soon")
                    with col3:
                        if st.button("Learn More", key=f"learn_{i}"):
                            st.info("Tutor integration coming soon")

    elif discover_mode == "Cross-Domain Collision":
        st.subheader("Cross-Domain Collision")
        st.caption("Find unexpected connections between research domains")

        cross_uploaded = st.file_uploader(
            "Upload PDFs from different domains",
            type=["pdf"],
            accept_multiple_files=True,
            key="cross_domain_upload",
        )

        parsed_dir = _ROOT / "outputs" / "parsed"
        existing_parsed = list(parsed_dir.glob("*.json")) if parsed_dir.exists() else []
        cross_selected = []
        if existing_parsed:
            st.write("Or use previously parsed papers:")
            cross_selected = st.multiselect(
                "Select parsed papers",
                [p.name for p in existing_parsed],
                key="cross_domain_select",
            )
        else:
            cross_selected = []

        use_llm_cross = st.checkbox("Use LLM for deeper analysis", value=True, key="cross_llm")

        if st.button("Find Cross-Domain Connections", key="cross_btn"):
            claims_by_paper = {}
            papers_dir = _ROOT / "data" / "papers"
            papers_dir.mkdir(parents=True, exist_ok=True)

            if cross_uploaded:
                for pdf in cross_uploaded:
                    with st.spinner(f"Parsing {pdf.name}..."):
                        temp_path = papers_dir / pdf.name
                        temp_path.write_bytes(pdf.read())
                        if parse_paper:
                            result = parse_paper(str(temp_path))
                            claims_by_paper[pdf.name] = result.claims
                        else:
                            st.error("Parser not available.")
                            break

            if cross_selected and load_parsed:
                for name in cross_selected:
                    path = parsed_dir / name
                    if path.exists():
                        result = load_parsed(str(path))
                        claims_by_paper[name] = result.claims

            if not claims_by_paper:
                st.warning("Please upload or select at least one paper.")
            else:
                try:
                    from core.cross_domain import (
                        find_cross_domain_connections,
                        summarize_collisions,
                    )

                    with st.spinner("Finding cross-domain connections..."):
                        report = find_cross_domain_connections(
                            claims_by_paper, use_llm=use_llm_cross
                        )

                    st.session_state["collision_report"] = report
                    st.success(
                        f"Found {len(report.connections)} connections across {len(report.domains_found)} domains!"
                    )
                except Exception as e:
                    st.error(str(e))

        if st.session_state.get("collision_report"):
            report = st.session_state["collision_report"]
            st.write(
                "**Domains detected:**",
                ", ".join(d.value for d in report.domains_found),
            )
            st.markdown(summarize_collisions(report))

            if report.synthesis_opportunities:
                st.subheader("Synthesis Opportunities")
                for opp in report.synthesis_opportunities:
                    key_safe = f"synth_{opp['domains'][0]}_{opp['domains'][1]}".replace(" ", "_")
                    with st.expander(
                        f"{opp['domains'][0]} + {opp['domains'][1]} ({opp['connection_count']} connections)"
                    ):
                        st.write(f"**Average strength:** {opp['avg_strength']:.0%}")
                        st.write(f"**Key concepts:** {', '.join(opp['concepts'])}")
                        st.write(f"**Idea:** {opp['synthesis_idea']}")

                        if st.button("Generate Hypothesis", key=key_safe):
                            st.info("Hypothesis generation coming soon")

    elif discover_mode == "Failure Analysis":
        st.subheader("Failure Analysis")
        st.caption("Mine failure patterns — what didn't work and why")

        failure_uploaded = st.file_uploader(
            "Upload PDFs to analyze",
            type=["pdf"],
            accept_multiple_files=True,
            key="failure_upload",
        )

        parsed_dir = _ROOT / "outputs" / "parsed"
        existing_parsed = list(parsed_dir.glob("*.json")) if parsed_dir.exists() else []
        failure_selected = []
        if existing_parsed:
            st.write("Or use previously parsed papers:")
            failure_selected = st.multiselect(
                "Select parsed papers",
                [p.name for p in existing_parsed],
                key="failure_select",
            )
        else:
            failure_selected = []

        use_llm_failure = st.checkbox("Use LLM for deeper analysis", value=True, key="failure_llm")

        if st.button("Analyze Failures", key="failure_btn"):
            papers = {}
            claims_by_paper = {}
            papers_dir = _ROOT / "data" / "papers"
            papers_dir.mkdir(parents=True, exist_ok=True)

            if failure_uploaded:
                for pdf in failure_uploaded:
                    with st.spinner(f"Parsing {pdf.name}..."):
                        temp_path = papers_dir / pdf.name
                        temp_path.write_bytes(pdf.read())
                        if parse_paper:
                            parsed = parse_paper(str(temp_path))
                            papers[pdf.name] = parsed.paper.full_text
                            claims_by_paper[pdf.name] = parsed.claims
                        else:
                            st.error("Parser not available.")
                            break

            if failure_selected and load_parsed:
                for name in failure_selected:
                    path = parsed_dir / name
                    if path.exists():
                        result = load_parsed(str(path))
                        base = name.replace("_parsed.json", "").replace(".json", "")
                        pdf_path = papers_dir / (base + ".pdf")
                        if pdf_path.exists():
                            try:
                                from core.pdf_reader import read_pdf
                                paper_content = read_pdf(str(pdf_path))
                                papers[name] = paper_content.full_text
                            except Exception:
                                papers[name] = " ".join(c.text for c in result.claims)
                        else:
                            papers[name] = " ".join(c.text for c in result.claims)
                        claims_by_paper[name] = result.claims

            if not papers:
                st.warning("Please upload or select at least one paper.")
            else:
                try:
                    from core.failure_analyzer import analyze_failures, summarize_failures

                    with st.spinner("Analyzing failures..."):
                        report = analyze_failures(
                            papers, claims_by_paper=claims_by_paper, use_llm=use_llm_failure
                        )

                    st.session_state["failure_report"] = report
                    st.success(
                        f"Found {report.total_failures} failures across {len(report.papers_analyzed)} papers!"
                    )
                except Exception as e:
                    st.error(str(e))

        if st.session_state.get("failure_report"):
            report = st.session_state["failure_report"]
            st.markdown(summarize_failures(report))

            if report.fixable_opportunities:
                st.subheader("🔧 Fixable Opportunities")
                for i, f in enumerate(report.fixable_opportunities[:10]):
                    with st.expander(
                        f"{f.what_failed} — {f.failure_type.value} (Severity: {f.severity:.0%})"
                    ):
                        st.write(f"**Description:** {f.description}")
                        st.write(f"**Source:** {f.paper}")
                        if f.why_failed:
                            st.write(f"**Why it failed:** {f.why_failed}")
                        if f.conditions:
                            st.write(f"**Conditions:** {f.conditions}")
                        if f.fix_hypothesis:
                            st.info(f"**Potential fix:** {f.fix_hypothesis}")

                        if st.button("Create Fix Hypothesis", key=f"fix_{i}"):
                            st.session_state["pending_fix_hypothesis"] = f

    elif discover_mode == "Translation Gap":
        st.subheader("Translation Gap Finder")
        st.caption("Find discoveries stuck in translation — they may be ready for revival")

        trans_uploaded = st.file_uploader(
            "Upload PDFs (ideally 5-10 years old)",
            type=["pdf"],
            accept_multiple_files=True,
            key="translation_upload",
        )

        parsed_dir = _ROOT / "outputs" / "parsed"
        existing_parsed = list(parsed_dir.glob("*.json")) if parsed_dir.exists() else []
        trans_selected = []
        if existing_parsed:
            st.write("Or use previously parsed papers:")
            trans_selected = st.multiselect(
                "Select parsed papers",
                [p.name for p in existing_parsed],
                key="translation_select",
            )
        else:
            trans_selected = []

        use_llm_trans = st.checkbox("Use LLM for deeper analysis", value=True, key="translation_llm")

        if st.button("Find Translation Gaps", key="translation_btn"):
            papers = {}
            claims_by_paper = {}
            papers_dir = _ROOT / "data" / "papers"
            papers_dir.mkdir(parents=True, exist_ok=True)

            if trans_uploaded:
                for pdf in trans_uploaded:
                    with st.spinner(f"Parsing {pdf.name}..."):
                        temp_path = papers_dir / pdf.name
                        temp_path.write_bytes(pdf.read())
                        if parse_paper:
                            parsed = parse_paper(str(temp_path))
                            papers[pdf.name] = parsed.paper.full_text
                            claims_by_paper[pdf.name] = parsed.claims
                        else:
                            st.error("Parser not available.")
                            break

            if trans_selected and load_parsed:
                for name in trans_selected:
                    path = parsed_dir / name
                    if path.exists():
                        result = load_parsed(str(path))
                        base = name.replace("_parsed.json", "").replace(".json", "")
                        pdf_path = papers_dir / (base + ".pdf")
                        if pdf_path.exists():
                            try:
                                from core.pdf_reader import read_pdf
                                paper_content = read_pdf(str(pdf_path))
                                papers[name] = paper_content.full_text
                            except Exception:
                                papers[name] = " ".join(c.text for c in result.claims)
                        else:
                            papers[name] = " ".join(c.text for c in result.claims)
                        claims_by_paper[name] = result.claims

            if not papers:
                st.warning("Please upload or select at least one paper.")
            else:
                try:
                    from core.translation_gap import find_translation_gaps, summarize_translation_gaps

                    with st.spinner("Analyzing translation gaps..."):
                        report = find_translation_gaps(
                            papers, claims_by_paper, use_llm=use_llm_trans
                        )

                    st.session_state["translation_report"] = report
                    st.success(
                        f"Analyzed {len(report.papers_analyzed)} papers, found {len(report.revival_candidates)} revival candidates!"
                    )
                except Exception as e:
                    st.error(str(e))

        if st.session_state.get("translation_report"):
            report = st.session_state["translation_report"]
            st.markdown(summarize_translation_gaps(report))

            if report.revival_candidates:
                st.subheader("🔄 Revival Candidates")
                for i, gap in enumerate(report.revival_candidates):
                    with st.expander(
                        f"{gap.paper} — Revival potential: {gap.revival_potential:.0%}"
                    ):
                        st.write(f"**Discovery:** {gap.discovery}")
                        st.write(f"**Year:** {gap.year or 'Unknown'}")
                        st.write(f"**Stage reached:** {gap.stage_reached.value}")
                        st.write(f"**Current status:** {gap.current_status}")

                        if gap.blocker:
                            st.warning(f"**Blocker:** {gap.blocker.value} — {gap.blocker_detail}")

                        if gap.revival_reason:
                            st.info(f"**Revival rationale:** {gap.revival_reason}")

                        st.write("**VRFC Factors:**")
                        for factor, value in gap.vrfc_factors.items():
                            st.write(f"- {factor}: {value}")

                        col1, col2 = st.columns(2)
                        with col1:
                            if st.button("Run VRFC Check", key=f"vrfc_trans_{i}"):
                                st.info("VRFC integration coming soon")
                        with col2:
                            if st.button("Create Revival Hypothesis", key=f"revive_{i}"):
                                st.session_state["pending_revival"] = gap

    else:
        st.info("No other discovery modes configured.")

with tab3:
    st.header("✅ Validate")

    validate_mode = st.selectbox(
        "Validation Mode",
        ["Rank Hypotheses", "Reproduce Claims", "SRFC Check", "VRFC Check"],
        key="validate_mode",
    )

    if validate_mode == "Rank Hypotheses":
        st.subheader("Hypothesis Ranker")
        st.caption("Score and rank all discovered hypotheses")

        hypotheses = []

        if "contradiction_report" in st.session_state:
            report = st.session_state["contradiction_report"]
            for c in report.contradictions:
                hypotheses.append({
                    "claim": c.resolution_hypothesis or f"Resolve: {c.description}",
                    "source": "contradiction",
                    "source_detail": f"From {c.paper_a} vs {c.paper_b}",
                    "source_data": {"severity": c.severity},
                    "papers": [c.paper_a, c.paper_b],
                    "evidence": [c.claim_a.text[:100], c.claim_b.text[:100]],
                })

        if "collision_report" in st.session_state:
            report = st.session_state["collision_report"]
            for conn in report.novel_connections:
                if conn.hypothesis:
                    hypotheses.append({
                        "claim": conn.hypothesis,
                        "source": "cross_domain",
                        "source_detail": f"{conn.domain_a.value} + {conn.domain_b.value}",
                        "source_data": {"strength": conn.strength},
                        "papers": conn.paper_sources,
                        "tags": [conn.domain_a.value, conn.domain_b.value],
                    })

        if "failure_report" in st.session_state:
            report = st.session_state["failure_report"]
            for f in report.fixable_opportunities[:10]:
                if f.fix_hypothesis:
                    hypotheses.append({
                        "claim": f.fix_hypothesis,
                        "source": "failure_fix",
                        "source_detail": f"Fix for: {f.what_failed}",
                        "source_data": {"severity": f.severity},
                        "papers": [f.paper],
                        "evidence": [f.description[:100]],
                    })

        if "translation_report" in st.session_state:
            report = st.session_state["translation_report"]
            for gap in report.revival_candidates:
                hypotheses.append({
                    "claim": f"Revive: {gap.discovery[:100]}",
                    "source": "revival",
                    "source_detail": f"From {gap.paper} ({gap.year or 'unknown year'})",
                    "source_data": {
                        "revival_potential": gap.revival_potential,
                        "stage_reached": gap.stage_reached.value,
                    },
                    "papers": [gap.paper],
                    "evidence": [gap.current_status],
                })

        if not hypotheses:
            st.info("No hypotheses found. Run discovery modes first (Discover tab).")
        else:
            st.write(f"**Found {len(hypotheses)} hypotheses from discovery modes**")

            if st.button("Rank All Hypotheses", key="rank_btn"):
                try:
                    from core.hypothesis_ranker import (
                        rank_all_hypotheses,
                        summarize_rankings,
                        hypothesis_to_ledger_format,
                    )
                    with st.spinner("Ranking hypotheses..."):
                        ranked = rank_all_hypotheses(hypotheses)
                        st.session_state["ranked_hypotheses"] = ranked
                except Exception as e:
                    st.error(str(e))

        if st.session_state.get("ranked_hypotheses"):
            ranked = st.session_state["ranked_hypotheses"]
            st.markdown(summarize_rankings(ranked))

            from core.hypothesis_ranker import hypothesis_to_ledger_format
            for h in ranked:
                claim_preview = h.claim[:60] + ("..." if len(h.claim) > 60 else "")
                with st.expander(f"#{h.rank} — {claim_preview} (Score: {h.overall_score:.0%})"):
                    col1, col2 = st.columns(2)
                    with col1:
                        st.write(f"**Source:** {h.source.value}")
                        st.write(f"**Detail:** {h.source_detail}")
                        st.write(f"**Falsification cost:** {h.falsification_cost}")
                    with col2:
                        st.metric("Novelty", f"{h.novelty_score:.0%}")
                        st.metric("Feasibility", f"{h.feasibility_score:.0%}")
                        st.metric("Translation", f"{h.translation_score:.0%}")
                        st.metric("Impact", f"{h.impact_score:.0%}")

                    st.write("**Next steps:**")
                    for step in h.next_steps:
                        st.write(f"- {step}")
                    st.write("**Related papers:**", ", ".join(h.related_papers) if h.related_papers else "None")

                    col1, col2, col3 = st.columns(3)
                    with col1:
                        if st.button("Add to Ledger", key=f"ledger_{h.id}"):
                            ledger_entry = hypothesis_to_ledger_format(h)
                            st.session_state["pending_ledger_entry"] = ledger_entry
                            st.success(f"Ready to add {h.id} to Hypothesis Ledger")
                    with col2:
                        if st.button("Deep Dive", key=f"dive_{h.id}"):
                            st.info("Tutor integration coming soon")
                    with col3:
                        if st.button("Export", key=f"export_{h.id}"):
                            st.json(hypothesis_to_ledger_format(h))

    elif validate_mode == "Reproduce Claims":
        st.subheader("Claim Reproduction")
        st.caption("Validate claims by reproducing analysis")

        parsed_dir = _ROOT / "outputs" / "parsed"
        existing_parsed = list(parsed_dir.glob("*.json")) if parsed_dir.exists() else []

        if not existing_parsed:
            st.info("No parsed papers. Upload and parse papers first.")
        else:
            options = [p.stem for p in existing_parsed]
            selected = st.selectbox("Select paper", options, key="repro_select")
            if selected:
                path = parsed_dir / f"{selected}.json"
                if path.exists() and load_parsed:
                    result = load_parsed(str(path))
                    st.write(f"**Claims in paper:** {len(result.claims)}")
                    for i, claim in enumerate(result.claims):
                        claim_preview = claim.text[:60] + ("..." if len(claim.text) > 60 else "")
                        with st.expander(f"Claim {i+1}: {claim_preview}"):
                            st.write(f"**Type:** {claim.claim_type.value}")
                            st.write(f"**Confidence:** {claim.confidence:.0%}")
                            st.write(f"**Quantified:** {'Yes' if claim.quantified else 'No'}")
                            if claim.stats:
                                st.write("**Statistics:**", claim.stats)
                            col1, col2 = st.columns(2)
                            with col1:
                                status = st.selectbox(
                                    "Reproduction status",
                                    ["Not tested", "Reproduced", "Partially reproduced", "Failed to reproduce"],
                                    key=f"status_{i}",
                                )
                            with col2:
                                if st.button("Log result", key=f"log_{i}"):
                                    st.success(f"Logged: {status}")

    elif validate_mode == "SRFC Check":
        st.subheader("SRFC Check")
        st.caption("Soft Robotics Feasibility Compiler — Can it work physically?")

        if not VALIDATION_TRINITY_AVAILABLE:
            st.error("Validation Trinity not available. Check enterprise/decision_brief module.")
        else:
            st.markdown("""
            **SRFC evaluates:**
            - Geometry constraints (lumen clearance, bend radius)
            - Actuation limits (pressure, response time)
            - Materials (Young's modulus, biocompatibility)
            - Safety (contact pressure, dwell time)
            """)

            query = st.text_area(
                "Describe the design/hypothesis to check",
                height=100,
                placeholder="e.g., A soft gripper with 3 fingers using DragonSkin 10 silicone for picking eggs",
                key="srfc_input",
            )

            col1, col2 = st.columns(2)
            with col1:
                domain = st.selectbox(
                    "Domain",
                    ["robotics", "synthetic_biology", "materials", "medical_device"],
                    key="srfc_domain",
                )
            with col2:
                params_complete = st.checkbox("Parameters complete?", value=False, key="srfc_complete")

            params = {}
            with st.expander("Additional Parameters"):
                pressure = st.number_input("Max pressure (kPa)", value=100, key="srfc_pressure")
                modulus = st.number_input("Young's modulus (kPa)", value=100, key="srfc_modulus")
                scale = st.selectbox("Scale", ["micro", "meso", "macro"], index=1, key="srfc_scale")
                params["max_pressure_kpa"] = pressure
                params["youngs_modulus_kpa"] = modulus
                params["scale"] = scale

            if st.button("Run SRFC Check", key="run_srfc"):
                if not query:
                    st.warning("Please describe the design/hypothesis")
                else:
                    with st.spinner("Running SRFC analysis..."):
                        try:
                            srfc, srfc_reason, tsrfc, tsrfc_reason, vrfc, vrfc_reason = _compute_validation_trinity(
                                query=query,
                                params=params,
                                complete=params_complete,
                                domains=[domain],
                            )

                            st.subheader("SRFC Result")
                            if srfc == "GREEN":
                                st.success(f"✅ **GREEN** — {srfc_reason}")
                            elif srfc == "AMBER":
                                st.warning(f"⚠️ **AMBER** — {srfc_reason}")
                            else:
                                st.error(f"❌ **RED** — {srfc_reason}")

                            st.subheader("TSRFC (Workflow)")
                            if tsrfc == "GREEN":
                                st.success(f"✅ **GREEN** — {tsrfc_reason}")
                            elif tsrfc == "AMBER":
                                st.warning(f"⚠️ **AMBER** — {tsrfc_reason}")
                            else:
                                st.error(f"❌ **RED** — {tsrfc_reason}")

                            st.session_state["last_srfc"] = {
                                "query": query,
                                "srfc": srfc,
                                "srfc_reason": srfc_reason,
                                "tsrfc": tsrfc,
                                "tsrfc_reason": tsrfc_reason,
                            }
                        except Exception as e:
                            st.error(f"SRFC check failed: {e}")

    elif validate_mode == "VRFC Check":
        st.subheader("VRFC Check")
        st.caption("Validation & Risk Feasibility Compiler — Will it survive reality?")

        if not VALIDATION_TRINITY_AVAILABLE:
            st.error("Validation Trinity not available. Check enterprise/decision_brief module.")
        else:
            st.markdown("""
            **VRFC evaluates:**
            - Evidence grade (RCT, registry, bench)
            - Regulatory path (510k, PMA, CE)
            - Reimbursement (CPT, DRG)
            - Adoption friction & stakeholder incentives
            - Litigation risk
            """)

            query = st.text_area(
                "Describe the discovery/intervention to check",
                height=100,
                placeholder="e.g., A novel drug delivery system using soft robotic capsules for targeted release",
                key="vrfc_input",
            )

            col1, col2 = st.columns(2)
            with col1:
                domain = st.selectbox(
                    "Domain",
                    ["medical_device", "therapeutic", "diagnostic", "robotics", "research"],
                    key="vrfc_domain",
                )
            with col2:
                evidence_level = st.selectbox(
                    "Evidence level",
                    ["bench", "preclinical", "clinical_pilot", "rct"],
                    key="vrfc_evidence",
                )

            params = {}
            with st.expander("Additional Parameters"):
                regulatory_class = st.selectbox(
                    "Regulatory class",
                    ["Class I", "Class II (510k)", "Class III (PMA)", "Unknown"],
                    key="vrfc_reg",
                )
                existing_predicates = st.checkbox("Existing predicates?", value=False, key="vrfc_pred")
                reimbursement_path = st.checkbox("Clear reimbursement path?", value=False, key="vrfc_reimb")
                params["evidence_level"] = evidence_level
                params["regulatory_class"] = regulatory_class
                params["has_predicates"] = existing_predicates
                params["reimbursement_clear"] = reimbursement_path

            if st.button("Run VRFC Check", key="run_vrfc"):
                if not query:
                    st.warning("Please describe the discovery/intervention")
                else:
                    with st.spinner("Running VRFC analysis..."):
                        try:
                            srfc, srfc_reason, tsrfc, tsrfc_reason, vrfc, vrfc_reason = _compute_validation_trinity(
                                query=query,
                                params=params,
                                complete=True,
                                domains=[domain],
                            )

                            st.subheader("VRFC Result")
                            if vrfc == "GREEN":
                                st.success(f"✅ **GREEN** — {vrfc_reason}")
                            elif vrfc == "AMBER":
                                st.warning(f"⚠️ **AMBER** — {vrfc_reason}")
                            else:
                                st.error(f"❌ **RED** — {vrfc_reason}")

                            if vrfc != "GREEN":
                                st.subheader("Potential Failure Surfaces")
                                failure_surfaces = []
                                if "regulatory" in vrfc_reason.lower():
                                    failure_surfaces.append("🚫 **Regulatory bottleneck** — May need additional evidence or different pathway")
                                if "evidence" in vrfc_reason.lower():
                                    failure_surfaces.append("📊 **Evidence gap** — Current evidence may not support translation")
                                if "reimbursement" in vrfc_reason.lower() or "adoption" in vrfc_reason.lower():
                                    failure_surfaces.append("💰 **Adoption barrier** — Commercial/reimbursement pathway unclear")
                                if "coordination" in vrfc_reason.lower():
                                    failure_surfaces.append("🤝 **Coordination challenge** — Multi-stakeholder alignment needed")
                                if not failure_surfaces:
                                    failure_surfaces.append("⚠️ **General uncertainty** — Further analysis recommended")
                                for fs in failure_surfaces:
                                    st.markdown(fs)

                            st.session_state["last_vrfc"] = {
                                "query": query,
                                "vrfc": vrfc,
                                "vrfc_reason": vrfc_reason,
                                "domain": domain,
                            }

                            st.subheader("Suggested Next Steps")
                            if vrfc == "GREEN":
                                st.markdown("1. Proceed with detailed development plan")
                                st.markdown("2. Identify key milestones and go/no-go criteria")
                                st.markdown("3. Secure resources for next phase")
                            elif vrfc == "AMBER":
                                st.markdown("1. Address identified uncertainty factors")
                                st.markdown("2. Gather additional evidence or data")
                                st.markdown("3. Re-run VRFC after addressing gaps")
                            else:
                                st.markdown("1. Identify fundamental blockers")
                                st.markdown("2. Consider alternative approaches")
                                st.markdown("3. Consult domain experts before proceeding")
                        except Exception as e:
                            st.error(f"VRFC check failed: {e}")

    # Send to Ledger section (always visible in Validate tab)
    st.divider()
    st.subheader("📤 Send to Hypothesis Ledger")

    if not LEDGER_AVAILABLE:
        st.warning("Hypothesis Ledger integration not available. Ensure breakthrough_engine is on PYTHONPATH.")
    else:
        _cr = st.session_state.get("contradiction_report")
        _cl = st.session_state.get("collision_report")
        _fr = st.session_state.get("failure_report")
        _tr = st.session_state.get("translation_report")
        counts = {
            "contradictions": len(getattr(_cr, "contradictions", [])) if _cr else 0,
            "cross_domain": len(getattr(_cl, "novel_connections", [])) if _cl else 0,
            "failures": len(getattr(_fr, "fixable_opportunities", [])) if _fr else 0,
            "revivals": len(getattr(_tr, "revival_candidates", [])) if _tr else 0,
        }
        total = sum(counts.values())

        if total == 0:
            st.info("No discoveries to send. Run discovery modes first (Discover tab).")
        else:
            st.write(f"**Available discoveries:** {total}")
            col1, col2, col3, col4 = st.columns(4)
            col1.metric("Contradictions", counts["contradictions"])
            col2.metric("Cross-Domain", counts["cross_domain"])
            col3.metric("Failures", counts["failures"])
            col4.metric("Revivals", counts["revivals"])
            preview = st.checkbox("Preview before sending", value=True, key="ledger_preview_cb")

            if st.button("🚀 Send All to Hypothesis Ledger", key="send_all_ledger") and bulk_create_from_scientist_session:
                if preview:
                    results = bulk_create_from_scientist_session(st.session_state, auto_add=False)
                    st.session_state["ledger_preview_results"] = results
                    st.rerun()
                else:
                    with st.spinner("Creating hypotheses..."):
                        results = bulk_create_from_scientist_session(st.session_state, auto_add=True)
                    st.success(f"✅ Created {results.get('total', 0)} hypotheses in Ledger!")
                    if results.get("total", 0) > 0:
                        st.write("**Created:**")
                        for source in ("contradictions", "cross_domain", "failures", "revivals"):
                            n = len(results.get(source, []))
                            if n:
                                st.write(f"- {source}: {n} hypotheses")
                        st.info("View them in Breakthrough Engine → Hypothesis Ledger")
                    st.balloons()

            if st.session_state.get("ledger_preview_results") and preview:
                results = st.session_state["ledger_preview_results"]
                st.write(f"**Preview: {results.get('total', 0)} hypotheses to be created**")
                for source in ("contradictions", "cross_domain", "failures", "revivals"):
                    hyps = results.get(source, [])
                    if hyps:
                        st.write(f"**{source.replace('_', ' ').title()}:**")
                        for h in hyps[:3]:
                            claim = (h.get("claim") or "")[:80]
                            st.write(f"- {claim}...")
                        if len(hyps) > 3:
                            st.write(f"  ... and {len(hyps) - 3} more")
                if st.button("✅ Confirm and Send", key="confirm_send_ledger") and bulk_create_from_scientist_session:
                    results = bulk_create_from_scientist_session(st.session_state, auto_add=True)
                    st.session_state.pop("ledger_preview_results", None)
                    st.success(f"Created {results.get('total', 0)} hypotheses in Ledger!")
                    st.balloons()
                    st.rerun()

    # Full Validation Trinity (always visible in Validate tab)
    st.divider()
    st.subheader("🔺 Full Validation Trinity")
    st.caption("Run SRFC + TSRFC + VRFC together")

    if not VALIDATION_TRINITY_AVAILABLE:
        st.error("Validation Trinity not available.")
    else:
        trinity_query = st.text_area(
            "Describe the hypothesis/design/intervention",
            height=100,
            key="trinity_query",
            placeholder="e.g., A soft robotic surgical tool using pneumatic actuation for minimally invasive procedures",
        )

        trinity_domain = st.selectbox(
            "Primary domain",
            ["robotics", "medical_device", "synthetic_biology", "therapeutic", "research"],
            key="trinity_domain",
        )

        if st.button("Run Full Trinity Check", key="run_trinity"):
            if not trinity_query:
                st.warning("Please describe the hypothesis")
            else:
                with st.spinner("Running Validation Trinity..."):
                    try:
                        srfc, srfc_r, tsrfc, tsrfc_r, vrfc, vrfc_r = _compute_validation_trinity(
                            query=trinity_query,
                            params={},
                            complete=True,
                            domains=[trinity_domain],
                        )

                        col1, col2, col3 = st.columns(3)
                        with col1:
                            st.markdown("### SRFC")
                            st.markdown("*Can it work?*")
                            if srfc == "GREEN":
                                st.success(f"✅ {srfc}")
                            elif srfc == "AMBER":
                                st.warning(f"⚠️ {srfc}")
                            else:
                                st.error(f"❌ {srfc}")
                            st.caption(srfc_r)
                        with col2:
                            st.markdown("### TSRFC")
                            st.markdown("*What does it replace?*")
                            if tsrfc == "GREEN":
                                st.success(f"✅ {tsrfc}")
                            elif tsrfc == "AMBER":
                                st.warning(f"⚠️ {tsrfc}")
                            else:
                                st.error(f"❌ {tsrfc}")
                            st.caption(tsrfc_r)
                        with col3:
                            st.markdown("### VRFC")
                            st.markdown("*Will it survive reality?*")
                            if vrfc == "GREEN":
                                st.success(f"✅ {vrfc}")
                            elif vrfc == "AMBER":
                                st.warning(f"⚠️ {vrfc}")
                            else:
                                st.error(f"❌ {vrfc}")
                            st.caption(vrfc_r)

                        st.divider()
                        statuses = [srfc, tsrfc, vrfc]
                        if "RED" in statuses:
                            overall = "RED"
                            st.error("**Overall: RED** — Critical blockers identified. Do not proceed without addressing.")
                        elif "AMBER" in statuses:
                            overall = "AMBER"
                            st.warning("**Overall: AMBER** — Uncertainties exist. Proceed with caution and gather more data.")
                        else:
                            overall = "GREEN"
                            st.success("**Overall: GREEN** — Translation pathway appears viable. Proceed to next phase.")

                        st.session_state["last_trinity"] = {
                            "query": trinity_query,
                            "domain": trinity_domain,
                            "srfc": srfc,
                            "srfc_reason": srfc_r,
                            "tsrfc": tsrfc,
                            "tsrfc_reason": tsrfc_r,
                            "vrfc": vrfc,
                            "vrfc_reason": vrfc_r,
                            "overall": overall,
                        }
                    except Exception as e:
                        st.error(f"Trinity check failed: {e}")
