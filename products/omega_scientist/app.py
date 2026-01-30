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

st.set_page_config(page_title="OMEGA Scientist", page_icon="🔬", layout="wide")

st.title("🔬 OMEGA Scientist")
st.caption("Discovery + Validation + Translation")

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
    st.header("Discover")
    st.info("Discovery modes coming soon: Contradiction Mining, Cross-Domain Collision, Failure Analysis")

with tab3:
    st.header("Validate")
    st.info("Validation modes coming soon: Reproduce claims, SRFC/VRFC assessment")
