"""
OMEGA Tutor — Streamlit chat interface.
Ask anything. Learn at your level. Zero friction.
"""

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

# Load .env from project root so GEMINI_API_KEY / ANTHROPIC_API_KEY are available
import os
try:
    from dotenv import load_dotenv
    load_dotenv(_ROOT / ".env")
except ImportError:
    pass

import streamlit as st
from core import TutorEngine

st.set_page_config(page_title="OMEGA Tutor", page_icon="🎓", layout="wide")

# Query param ?topic= for failure → Tutor auto-links (topic used after level + engine defined)
try:
    _params = st.query_params
    topic = _params.get("topic")
except AttributeError:
    _params = st.experimental_get_query_params()
    topic = (_params.get("topic") or [None])[0]
if topic:
    topic = topic.replace("+", " ")

st.title("🎓 OMEGA Tutor")
st.caption("Ask anything. Learn at your level.")

# Level selector in sidebar
level = st.sidebar.selectbox(
    "Learning Level",
    ["child", "teen", "adult", "expert", "researcher"],
    index=2,
    format_func=lambda x: {
        "child": "👶 Child (5-10)",
        "teen": "🎮 Teen (11-17)",
        "adult": "👤 Adult",
        "expert": "🎓 Expert",
        "researcher": "🔬 Researcher",
    }[x],
)

st.sidebar.markdown("---")
st.sidebar.markdown("""
**Levels explained:**
- **Child**: Simple words, fun examples
- **Teen**: Real-world connections
- **Adult**: Clear and practical
- **Expert**: Nuance and depth
- **Researcher**: Full OMEGA-MAX analysis
""")

# API key: env (or .env) or sidebar
env_has_key = bool(os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY") or os.environ.get("ANTHROPIC_API_KEY"))
if env_has_key:
    st.sidebar.caption("Using API key from environment (.env or shell)")
api_key = st.sidebar.text_input(
    "API key (optional)",
    type="password",
    placeholder="Paste here if not using .env",
    help="Or put GEMINI_API_KEY=... in omega_tutor/.env so it works from any terminal.",
)
if not env_has_key and not api_key:
    st.sidebar.caption("No key set. Add omega_tutor/.env with GEMINI_API_KEY=... or paste above.")

# Backend: LM Studio (local) or Gemini (cloud)
try:
    _engine = TutorEngine(api_key=api_key or None)
    backend_label = "🖥️ Local (LM Studio)" if _engine.use_local else "☁️ Cloud (Gemini)"
except Exception:
    _engine = None
    backend_label = "⚠️ No backend (start LM Studio or set GEMINI_API_KEY)"
st.sidebar.caption(backend_label)

# Topic from ?topic= (e.g. from Console "Learn why" link)
if topic:
    st.info(f"You were sent here to learn about: **{topic}**")
    _topic_q = st.text_input("Ask about this topic (edit if you like):", value=f"Explain {topic} in simple terms", key="topic_q")
    if st.button("Ask Tutor about this", key="ask_topic") and _topic_q:
        if "messages" not in st.session_state:
            st.session_state.messages = []
        st.session_state.messages.append({"role": "user", "content": _topic_q})
        _eng = _engine if _engine is not None else TutorEngine(api_key=api_key or None)
        _resp = _eng.teach(_topic_q, level=level)
        st.session_state.messages.append({"role": "assistant", "content": _resp.to_markdown()})
        st.rerun()
    st.markdown("---")

# Chat interface
if "messages" not in st.session_state:
    st.session_state.messages = []

# Display chat history
for msg in st.session_state.messages:
    with st.chat_message(msg["role"]):
        st.markdown(msg["content"])

# Input
if prompt := st.chat_input("What would you like to learn about?"):
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)

    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            engine = _engine if _engine is not None else TutorEngine(api_key=api_key or None)
            response = engine.teach(prompt, level=level)
            st.markdown(response.to_markdown())

    st.session_state.messages.append({"role": "assistant", "content": response.to_markdown()})

# Quick actions
st.sidebar.markdown("---")
if st.sidebar.button("🔄 New Topic"):
    st.session_state.messages = []
    st.rerun()

if st.sidebar.button("📚 Go Deeper") and st.session_state.messages:
    user_msgs = [m for m in st.session_state.messages if m["role"] == "user"]
    if user_msgs:
        last_topic = user_msgs[-1]["content"]
        engine = _engine if _engine is not None else TutorEngine(api_key=api_key or None)
        response = engine.deeper(last_topic, level)
        st.session_state.messages.append({"role": "assistant", "content": response.to_markdown()})
    st.rerun()
