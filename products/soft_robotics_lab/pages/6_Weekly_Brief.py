"""
Weekly Brief Page
"""

import streamlit as st
import sys
from pathlib import Path
from datetime import datetime

_soft_lab = Path(__file__).resolve().parent.parent
_research_system = _soft_lab / "research_system"
if str(_research_system) not in sys.path:
    sys.path.insert(0, str(_research_system))

from research_memory import ResearchMemory

st.set_page_config(
    page_title="Weekly Brief", page_icon="📋", layout="wide"
)

st.markdown("# 📋 Weekly Brief")
st.markdown(
    "Auto-generated insight briefing for your lab meeting"
)

st.markdown("---")


@st.cache_resource
def get_memory(lab_name):
    return ResearchMemory(lab_name)


def generate_brief_html(brief, lab_name: str) -> str:
    """Generate print-friendly HTML for the brief (open in browser → Print → Save as PDF)."""
    sections_html = []
    # 1. What Changed
    items = (
        brief.what_changed
        if brief.what_changed
        and brief.what_changed[0] != "No significant changes this week"
        else ["No significant changes this week"]
    )
    sections_html.append(
        '<div class="section"><h2>1. What Changed This Week</h2>'
        + "".join(f'<div class="item">{x}</div>' for x in items)
        + "</div>"
    )
    # 2. Contradictions
    items = (
        brief.top_contradictions
        if brief.top_contradictions
        and brief.top_contradictions[0] != "No contradictions detected"
        else ["No contradictions detected"]
    )
    sections_html.append(
        '<div class="section"><h2>2. Contradictions &amp; Tensions</h2>'
        + "".join(
            f'<div class="item item-warning">{x}</div>'
            if x != "No contradictions detected"
            else '<div class="item item-success">No contradictions detected</div>'
            for x in items
        )
        + "</div>"
    )
    # 3. Discriminators
    items = (
        brief.top_discriminators
        if brief.top_discriminators
        and brief.top_discriminators[0] != "No clear discriminators identified"
        else ["No clear discriminators identified"]
    )
    sections_html.append(
        '<div class="section"><h2>3. Top Discriminators</h2><p><em>Smallest experiments that would distinguish between competing hypotheses</em></p>'
        + "".join(f'<div class="item item-success">{x}</div>' for x in items)
        + "</div>"
    )
    # 4. Fragility
    items = (
        brief.fragility_alerts
        if brief.fragility_alerts
        and brief.fragility_alerts[0] != "No fragility alerts"
        else ["No fragility alerts - all hypotheses well-supported"]
    )
    sections_html.append(
        '<div class="section"><h2>4. Fragility Alerts</h2>'
        + "".join(f'<div class="item item-danger">{x}</div>' for x in items)
        + "</div>"
    )
    # 5. Portfolio
    items = (
        brief.portfolio_shifts
        if brief.portfolio_shifts
        and brief.portfolio_shifts[0] != "Portfolio stable"
        else ["Portfolio stable - no major shifts recommended"]
    )
    portfolio_divs = []
    for x in items:
        cls = "item-danger" if "killing" in x.lower() else "item-success" if "pursue" in x.lower() else ""
        portfolio_divs.append(f'<div class="item {cls}">{x}</div>')
    sections_html.append('<div class="section"><h2>5. Portfolio Shifts</h2>' + "".join(portfolio_divs) + "</div>")
    # 6. Questions
    q_html = "".join(
        f'<div class="question"><span class="question-number">{i}</span>{q}</div>'
        for i, q in enumerate(brief.questions_for_pi, 1)
    )
    sections_html.append('<div class="section"><h2>6. Questions for Discussion</h2>' + q_html + "</div>")
    spec_html = ""
    if brief.speculation_flags:
        spec_html = (
            '<div class="speculation"><h3>⚠ Speculation Flags</h3><ul>'
            + "".join(f"<li>{f}</li>" for f in brief.speculation_flags)
            + "</ul></div>"
        )
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Weekly Brief — {lab_name}</title>
<style>
@page {{ size: A4; margin: 2cm; }}
@media print {{ body {{ margin: 0; }} .no-print {{ display: none; }} }}
body {{ font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif; font-size: 11pt; line-height: 1.6; color: #333; max-width: 800px; margin: 0 auto; padding: 20px; }}
.header {{ background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); color: white; padding: 30px; border-radius: 10px; margin-bottom: 30px; }}
.header h1 {{ margin: 0; font-size: 28pt; color: #00d4ff; }}
.header p {{ margin: 10px 0 0 0; color: #ccc; }}
.section {{ margin: 25px 0; page-break-inside: avoid; }}
.section h2 {{ color: #1a1a2e; border-bottom: 2px solid #00d4ff; padding-bottom: 10px; margin-bottom: 15px; }}
.item {{ background: #f8f9fa; padding: 15px; border-radius: 8px; margin: 10px 0; border-left: 4px solid #00d4ff; }}
.item-warning {{ border-color: #ffaa00; background: #fffbf0; }}
.item-danger {{ border-color: #dc3545; background: #fff5f5; }}
.item-success {{ border-color: #28a745; background: #f0fff5; }}
.question {{ background: #f0f7ff; padding: 15px; border-radius: 8px; margin: 10px 0; }}
.question-number {{ display: inline-block; width: 30px; height: 30px; background: #00d4ff; color: white; border-radius: 50%; text-align: center; line-height: 30px; margin-right: 10px; font-weight: bold; }}
.speculation {{ background: #fff5f5; border: 1px solid #ffcccc; padding: 15px; border-radius: 8px; margin-top: 20px; }}
.speculation h3 {{ color: #dc3545; margin-top: 0; }}
.footer {{ margin-top: 40px; padding-top: 20px; border-top: 1px solid #ddd; text-align: center; color: #888; font-size: 9pt; }}
</style>
</head>
<body>
<div class="header">
<h1>Weekly Insight Brief</h1>
<p><strong>{lab_name}</strong> — Week {brief.week} — Generated {brief.generated[:10]}</p>
</div>
{"".join(sections_html)}
{spec_html}
<div class="footer">
<p>Brief ID: {brief.id} — Sources: {len(brief.sources_used)}</p>
<p>Generated by OMEGA Research Memory — Open in browser and use Print → Save as PDF</p>
</div>
</body>
</html>"""


lab_name = st.sidebar.text_input(
    "Lab Name", value="rossiter_lab", key="brief_lab"
)
memory = get_memory(lab_name)

col1, col2, col3 = st.columns([2, 1, 1])

with col1:
    st.markdown(f"**Lab:** {lab_name}")
    st.markdown(f"**Week:** {datetime.now().strftime('%Y-W%W')}")

with col2:
    generate_btn = st.button("📋 Generate Brief", type="primary")

with col3:
    pass

if generate_btn:
    with st.spinner("Generating weekly brief..."):
        brief = memory.generate_weekly_brief()

    st.session_state["current_brief"] = brief

st.markdown("---")

if "current_brief" in st.session_state:
    brief = st.session_state["current_brief"]

    st.markdown(
        f"""
    <div style="background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%); padding: 30px; border-radius: 15px; margin-bottom: 30px;">
        <h1 style="color: #00d4ff; margin: 0;">Weekly Insight Brief</h1>
        <p style="color: #888; margin: 10px 0 0 0;">Week {brief.week} • Generated {brief.generated[:10]}</p>
    </div>
    """,
        unsafe_allow_html=True,
    )

    col1, col2, col3 = st.columns([1, 1, 4])
    with col1:
        st.download_button(
            "📥 Download Markdown",
            brief.to_markdown(),
            file_name=f"weekly_brief_{brief.week.replace('-', '_')}.md",
            mime="text/markdown",
            key="dl_brief_md",
        )
    with col2:
        html_content = generate_brief_html(brief, lab_name)
        st.download_button(
            "📄 Download HTML",
            html_content,
            file_name=f"weekly_brief_{brief.week.replace('-', '_')}.html",
            mime="text/html",
            key="dl_brief_html",
            help="Open in browser, then Print → Save as PDF",
        )

    st.markdown("---")

    st.markdown("## 1. What Changed This Week")

    if (
        brief.what_changed
        and brief.what_changed[0] != "No significant changes this week"
    ):
        for item in brief.what_changed:
            st.markdown(
                f"""
            <div style="background: #252540; padding: 15px; border-radius: 8px; margin: 10px 0; border-left: 4px solid #00d4ff;">
                {item}
            </div>
            """,
                unsafe_allow_html=True,
            )
    else:
        st.info("No significant changes this week")

    st.markdown("---")

    st.markdown("## 2. Top Contradictions / Tensions")

    if (
        brief.top_contradictions
        and brief.top_contradictions[0] != "No contradictions detected"
    ):
        for item in brief.top_contradictions:
            st.warning(item)
    else:
        st.success("No contradictions detected")

    st.markdown("---")

    st.markdown("## 3. Top Discriminators")
    st.markdown(
        "*Smallest experiments that would distinguish between competing hypotheses*"
    )

    if (
        brief.top_discriminators
        and brief.top_discriminators[0]
        != "No clear discriminators identified"
    ):
        for item in brief.top_discriminators:
            st.info(item)
    else:
        st.info("No clear discriminators identified this week")

    st.markdown("---")

    st.markdown("## 4. Fragility Alerts")
    st.markdown(
        "*Hypotheses with thin evidence that may need attention*"
    )

    if (
        brief.fragility_alerts
        and brief.fragility_alerts[0] != "No fragility alerts"
    ):
        for item in brief.fragility_alerts:
            st.error(item)
    else:
        st.success("No fragility alerts")

    st.markdown("---")

    st.markdown("## 5. Portfolio Shifts")
    st.markdown(
        "*Recommended changes to research priorities*"
    )

    if (
        brief.portfolio_shifts
        and brief.portfolio_shifts[0] != "Portfolio stable"
    ):
        for item in brief.portfolio_shifts:
            if "killing" in item.lower():
                st.error(item)
            elif "pursue" in item.lower():
                st.success(item)
            else:
                st.info(item)
    else:
        st.info(
            "Portfolio stable - no major shifts recommended"
        )

    st.markdown("---")

    st.markdown("## 6. Questions for Discussion")

    for i, q in enumerate(brief.questions_for_pi, 1):
        st.markdown(
            f"""
        <div style="background: #1a1a2e; padding: 15px; border-radius: 8px; margin: 10px 0;">
            <span style="color: #00d4ff; font-weight: bold;">{i}.</span> {q}
        </div>
        """,
            unsafe_allow_html=True,
        )

    if brief.speculation_flags:
        st.markdown("---")
        st.markdown("## ⚠️ Speculation Flags")
        st.markdown(
            "*The following insights are flagged as speculative:*"
        )

        for flag in brief.speculation_flags:
            st.warning(flag)

    st.markdown("---")
    st.markdown(
        f"""
    <div style="text-align: center; color: #666; padding: 20px;">
        <p>Brief ID: {brief.id} • Sources used: {len(brief.sources_used)}</p>
        <p><em>Generated by OMEGA Research Memory</em></p>
    </div>
    """,
        unsafe_allow_html=True,
    )

else:
    st.markdown(
        """
    <div style="background: #1a1a2e; padding: 40px; border-radius: 15px; text-align: center;">
        <h2 style="color: #00d4ff;">Ready to Generate</h2>
        <p style="color: #888;">Click the button above to generate your weekly insight brief.</p>
        <p style="color: #888;">The brief will analyze:</p>
        <ul style="color: #888; text-align: left; max-width: 400px; margin: 20px auto;">
            <li>What changed in your hypotheses</li>
            <li>Contradictions between claims</li>
            <li>Experiments that would discriminate</li>
            <li>Fragile hypotheses needing evidence</li>
            <li>Recommended portfolio shifts</li>
        </ul>
    </div>
    """,
        unsafe_allow_html=True,
    )

    st.markdown("---")
    st.markdown("### Current Research Memory")

    stats = memory.get_stats()

    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.metric("Documents", stats["documents"])
    with col2:
        st.metric("Hypotheses", stats["hypotheses"])
    with col3:
        st.metric("Active", stats["active_hypotheses"])
    with col4:
        st.metric("Results", stats["results"])

    if stats["hypotheses"] == 0:
        st.info(
            "💡 Add some hypotheses in the Research Memory page to get meaningful briefs."
        )
