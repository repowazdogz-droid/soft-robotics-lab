"""
OMEGA Tutor — Streamlit chat interface.
Ask anything. Learn at your level. Zero friction.
First run: welcome screen (age or Professional/Researcher). Then chat with level-adaptive teaching.
"""

import os
import sys
from pathlib import Path
from datetime import datetime

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

try:
    from dotenv import load_dotenv
    load_dotenv(_ROOT / ".env")
except ImportError:
    pass

import streamlit as st
from core import TutorEngine
from core.user_profile import (
    has_profile,
    load_profile,
    save_profile,
    get_current_level,
    get_display_label,
)
from core.level_adjust import adjust_level
try:
    from core.progress import progress_tracker
    from core.knowledge_decay import get_review_priority
except ImportError:
    progress_tracker = None
    get_review_priority = None

st.set_page_config(page_title="OMEGA Tutor", page_icon="🎓", layout="wide")

# Query param ?topic= for failure → Tutor auto-links
try:
    _params = st.query_params
    topic = _params.get("topic")
except AttributeError:
    _params = st.experimental_get_query_params()
    topic = (_params.get("topic") or [None])[0]
if topic:
    topic = topic.replace("+", " ")


def _render_welcome_or_change_age(sidebar: bool = False):
    """Render age/role form. Used for first-time welcome (main) or 'Change age' (sidebar)."""
    profile = load_profile()
    current_age = profile.get("age")
    container = st.sidebar if sidebar else st
    sk = "_sb" if sidebar else ""
    if "welcome_role" not in st.session_state:
        st.session_state["welcome_role"] = None
    age = container.number_input(
        "I'm _____ years old",
        min_value=5,
        max_value=120,
        value=int(current_age) if current_age is not None else 10,
        step=1,
        key="age_input" + sk,
    )
    container.caption("Or choose:")
    col1, col2 = container.columns(2)
    with col1:
        if container.button("Professional", key="role_pro" + sk):
            st.session_state["welcome_role"] = "professional"
    with col2:
        if container.button("Researcher", key="role_res" + sk):
            st.session_state["welcome_role"] = "researcher"
    use_role = st.session_state.get("welcome_role")
    if use_role:
        container.caption(f"Selected: **{use_role.title()}** — click Get Started to continue.")
    btn_label = "Save" if sidebar else "Get Started"
    if container.button(btn_label, type="primary", key="submit_profile" + sk):
        role_to_save = use_role if use_role else None
        created = profile.get("created_at") or datetime.now().isoformat()
        if role_to_save:
            save_profile({"age": None, "name": profile.get("name"), "role": role_to_save, "created_at": created})
        else:
            save_profile({"age": age, "name": profile.get("name"), "role": None, "created_at": created})
        st.session_state.pop("welcome_role", None)
        if sidebar:
            st.session_state["change_age"] = False
        st.rerun()
    return age, use_role


# ----- First-time welcome (no profile) -----
if not has_profile():
    st.title("🎓 Welcome to OMEGA Tutor")
    st.caption("Tell us a bit about you so we can teach at the right level. You won't see this again.")
    st.markdown("---")
    _render_welcome_or_change_age(sidebar=False)
    st.stop()

# ----- Profile exists: show chat -----
level = get_current_level()
display_label = get_display_label()

st.title("🎓 OMEGA Tutor")
st.caption("Ask anything. Learn at your level.")

# Sidebar: Learning as, Change age, API key, backend
st.sidebar.markdown(f"**Learning as:** {display_label}")
if st.sidebar.button("Change age", key="btn_change_age"):
    st.session_state["change_age"] = True
if st.session_state.get("change_age"):
    st.sidebar.markdown("---")
    st.sidebar.caption("Update your age or role:")
    _render_welcome_or_change_age(sidebar=True)
    st.sidebar.markdown("---")

st.sidebar.markdown("---")
env_has_key = bool(os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY") or os.environ.get("ANTHROPIC_API_KEY"))
if env_has_key:
    st.sidebar.caption("Using API key from environment (.env or shell)")
api_key = st.sidebar.text_input(
    "API key (optional)",
    type="password",
    placeholder="Paste here if not using .env",
    help="Or put GEMINI_API_KEY=... in omega_tutor/.env",
    key="api_key",
)
if not env_has_key and not api_key:
    st.sidebar.caption("No key set. Add omega_tutor/.env with GEMINI_API_KEY=... or paste above.")

try:
    _engine = TutorEngine(api_key=api_key or None)
    backend_label = "🖥️ Local (LM Studio)" if _engine.use_local else "☁️ Cloud (Gemini)"
except Exception:
    _engine = None
    backend_label = "⚠️ No backend (start LM Studio or set GEMINI_API_KEY)"
st.sidebar.caption(backend_label)

# Start progress session once per run/session
if progress_tracker and "progress_session_id" not in st.session_state:
    st.session_state["progress_session_id"] = progress_tracker.start_session()

# Progress Dashboard
if progress_tracker:
    st.sidebar.markdown("---")
    st.sidebar.markdown("**📊 This Week**")
    st.sidebar.markdown("─────────────")
    try:
        ws = progress_tracker.get_weekly_stats()
        st.sidebar.metric("Topics learned", ws.get("topics", 0))
        st.sidebar.metric("Questions", f"{ws.get('questions', 0)} ({ws.get('correct_rate', 0):.0f}% correct)")
        streak = progress_tracker.get_streak()
        st.sidebar.metric("Streak", f"{streak} days 🔥" if streak else "0 days")
        strong = progress_tracker.get_strongest_topics(3)
        if strong:
            st.sidebar.caption("💪 Strongest: " + ", ".join(strong[:3]))
        weak = progress_tracker.get_weakest_topics(3)
        if weak:
            st.sidebar.caption("📚 Review: " + ", ".join(weak[:3]))
    except Exception:
        st.sidebar.caption("Progress stats unavailable.")
    if st.sidebar.button("What should I review?"):
        st.session_state["show_review"] = True
    if st.session_state.get("show_review"):
        st.sidebar.markdown("---")
        st.sidebar.caption("**Due for review**")
        try:
            due = progress_tracker.get_due_for_review()
            if due:
                for d in due[:5]:
                    st.sidebar.markdown(f"- {d.get('name', '')} (confidence: {d.get('decayed_confidence', 0):.2f})")
            else:
                st.sidebar.caption("Nothing due. Keep learning!")
            if st.sidebar.button("Close", key="close_review"):
                st.session_state["show_review"] = False
                st.rerun()
        except Exception:
            st.sidebar.caption("Review list unavailable.")

# Handle Simpler/Deeper re-explain (before rendering messages)
if "messages" in st.session_state and "re_explain" in st.session_state:
    re = st.session_state.pop("re_explain")
    msg_index = re.get("msg_index", 0)
    direction = re.get("direction", "simpler")
    messages = st.session_state.messages
    if msg_index > 0 and msg_index < len(messages) and messages[msg_index].get("role") == "assistant":
        question = messages[msg_index - 1].get("content", "")
        msg_level = messages[msg_index].get("level") or level
        new_level = adjust_level(msg_level, direction)
        if question and new_level and _engine:
            engine = _engine
            response = engine.teach(question, new_level)
            label = "simpler" if direction == "simpler" else "deeper"
            new_content = f"**Here's a {label} explanation (adjusted to {new_level} level):**\n\n{response.to_markdown()}"
            st.session_state.messages.append({"role": "assistant", "content": new_content, "level": new_level})
    st.rerun()

# Topic from ?topic=
if topic:
    st.info(f"You were sent here to learn about: **{topic}**")
    _topic_q = st.text_input("Ask about this topic (edit if you like):", value=f"Explain {topic} in simple terms", key="topic_q")
    if st.button("Ask Tutor about this", key="ask_topic") and _topic_q:
        if "messages" not in st.session_state:
            st.session_state.messages = []
        st.session_state.messages.append({"role": "user", "content": _topic_q})
        _eng = _engine if _engine is not None else TutorEngine(api_key=api_key or None)
        _resp = _eng.teach(_topic_q, level=level)
        st.session_state.messages.append({"role": "assistant", "content": _resp.to_markdown(), "level": level})
        if progress_tracker:
            try:
                progress_tracker.record_topic(_topic_q.strip()[:80].replace("\n", " ") or "general")
                progress_tracker.record_question(_topic_q.strip()[:80] or "general", True)
            except Exception:
                pass
        st.rerun()
    st.markdown("---")

if "messages" not in st.session_state:
    st.session_state.messages = []

messages = st.session_state.messages
for i, msg in enumerate(messages):
    with st.chat_message(msg["role"]):
        st.markdown(msg["content"])
        if msg.get("role") == "assistant":
            msg_level = msg.get("level") or level
            st.caption(f"Explaining at: **{msg_level}** level")
            simpler_next = adjust_level(msg_level, "simpler")
            deeper_next = adjust_level(msg_level, "deeper")
            btn_col1, btn_col2, _ = st.columns([1, 1, 2])
            with btn_col1:
                simpler_clicked = st.button("😕 Simpler", key=f"simpler_{i}", disabled=(simpler_next is None))
            with btn_col2:
                deeper_clicked = st.button("🔬 Deeper", key=f"deeper_{i}", disabled=(deeper_next is None))
            if simpler_clicked and simpler_next:
                st.session_state["re_explain"] = {"direction": "simpler", "msg_index": i}
                st.rerun()
            if deeper_clicked and deeper_next:
                st.session_state["re_explain"] = {"direction": "deeper", "msg_index": i}
                st.rerun()

if prompt := st.chat_input("What would you like to learn about?"):
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)
    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            engine = _engine if _engine is not None else TutorEngine(api_key=api_key or None)
            response = engine.teach(prompt, level=level)
            st.markdown(response.to_markdown())
    st.session_state.messages.append({"role": "assistant", "content": response.to_markdown(), "level": level})
    if progress_tracker:
        try:
            topic_name = prompt.strip()[:80].replace("\n", " ").strip() or "general"
            progress_tracker.record_topic(topic_name)
            progress_tracker.record_question(topic_name, True)
        except Exception:
            pass

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
        st.session_state.messages.append({"role": "assistant", "content": response.to_markdown(), "level": level})
        if progress_tracker:
            try:
                progress_tracker.record_topic(last_topic.strip()[:80].replace("\n", " ") or "general")
            except Exception:
                pass
    st.rerun()
