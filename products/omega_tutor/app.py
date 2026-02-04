"""
OMEGA Tutor — Streamlit chat interface.
Ask anything. Learn at your level. Zero friction.
First run: welcome screen (pick mode: Simple & Fun, Clear & Direct, Technical, Research-Level). Then chat.
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
    PERSONA_CONFIG,
)
from core.level_adjust import adjust_level
try:
    from core.progress import progress_tracker
    from core.knowledge_decay import get_review_priority
except ImportError:
    progress_tracker = None
    get_review_priority = None
try:
    from core.quiz_generator import generate_quiz, generate_quiz_from_topics
    from core.explain_back import evaluate_explanation
except ImportError:
    generate_quiz = None
    generate_quiz_from_topics = None
    evaluate_explanation = None
try:
    from core.curriculum import curriculum_engine
except ImportError:
    curriculum_engine = None
try:
    from core.voice import voice_engine, detect_voice_command
except ImportError:
    voice_engine = None
    detect_voice_command = None
try:
    from core.session_summary import generate_summary
except ImportError:
    generate_summary = None
try:
    from core.cognitive_load import (
        get_cognitive_state,
        should_show_quiz,
        should_show_dashboard,
        is_overwhelmed,
    )
except ImportError:
    get_cognitive_state = None
    should_show_quiz = None
    should_show_dashboard = None
    is_overwhelmed = None
try:
    from core.learning_fingerprint import (
        record_learning_event,
        get_learning_recommendations,
        get_fingerprint_summary,
        load_fingerprint,
    )
except ImportError:
    record_learning_event = None
    get_learning_recommendations = None
    get_fingerprint_summary = None
    load_fingerprint = None
try:
    from core.misconception_detector import (
        detect_misconceptions,
        check_explanation_for_misconceptions,
        generate_preemptive_warning,
        get_topic_misconceptions,
    )
except ImportError:
    detect_misconceptions = None
    check_explanation_for_misconceptions = None
    generate_preemptive_warning = None
    get_topic_misconceptions = None

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


STARTER_QUESTIONS = {
    "soft-robotics-101": "What is soft robotics and why does it matter?",
    "machine-learning-101": "What is machine learning in simple terms?",
    "synthetic-biology-101": "What is synthetic biology and what can it do?",
}


def _clear_chat():
    """Clear chat messages and session scores. Does not reset progress_tracker (learning history)."""
    st.session_state["messages"] = []
    st.session_state.pop("quiz_questions", None)
    st.session_state.pop("quiz_index", None)
    st.session_state.pop("quiz_answers", None)
    st.session_state.pop("quiz_topic", None)
    st.session_state.pop("quiz_mode", None)
    st.session_state.pop("quiz_state", None)
    st.session_state.pop("show_quiz_summary", None)
    st.session_state.pop("explain_back_topic", None)
    st.session_state.pop("explain_back_index", None)
    st.session_state.pop("explain_back_last_score", None)
    st.session_state.pop("re_explain", None)
    st.session_state.pop("pending_starter_question", None)
    st.session_state.pop("pending_starter_curriculum_id", None)
    st.session_state.pop("misconception_warned_topics", None)
    st.session_state.pop("pending_misconception_warning", None)


def _save_persona(persona: str):
    """Save profile with persona, clear chat for fresh start, and rerun."""
    profile = load_profile()
    created = profile.get("created_at") or datetime.now().isoformat()
    save_profile({"persona": persona, "name": profile.get("name"), "created_at": created})
    _clear_chat()
    st.rerun()


def _render_welcome_personas():
    """First-time welcome: 4 persona cards. Click one → save and go to topic picker."""
    st.title("🎓 OMEGA Tutor")
    st.markdown("**Pick who you are and start learning:**")
    st.markdown("---")
    for pid, config in PERSONA_CONFIG.items():
        icon = config.get("icon", "")
        label = config.get("label", "").replace(" Mode", "")
        tagline = config.get("tagline", "")
        quote = config.get("quote", "")
        persona_label = {"kid": "Kid", "student": "Student", "professional": "Professional", "researcher": "Researcher"}.get(pid, "Student")
        btn_text = f"**{icon} I'm a {persona_label}**\n\n{tagline}\n\n\"{quote}\""
        if st.button(btn_text, key=f"persona_{pid}", use_container_width=True):
            _save_persona(pid)
    st.markdown("---")


def _render_topic_picker(level: str):
    """After persona picked: topic quick-picks + type anything. Click topic → starter question + teach."""
    st.subheader("What do you want to learn about?")
    curricula = curriculum_engine.list_curricula() if curriculum_engine else []
    # Quick-pick buttons: first 3 curricula by default or Soft Robotics, Machine Learning, Synthetic Biology by name
    picks = [c for c in curricula if c.get("id") in STARTER_QUESTIONS][:3]
    if not picks:
        picks = curricula[:3]
    col1, col2, col3 = st.columns(3)
    for i, c in enumerate(picks):
        col = [col1, col2, col3][i % 3]
        with col:
            name = c.get("name", c.get("id", ""))
            short = name.replace(" Fundamentals", "").replace(" 101", "") if name else c.get("id", "")
            if st.button(f"**{short}**", key=f"topic_pick_{c.get('id', i)}", use_container_width=True):
                q = STARTER_QUESTIONS.get(c.get("id", ""), f"What is {short} and why does it matter?")
                st.session_state["selected_curriculum_id"] = c.get("id")
                st.session_state["pending_starter_curriculum_id"] = c.get("id")
                st.session_state["pending_starter_question"] = q
                st.rerun()
    st.markdown("**Or type anything:**")
    go_col, _ = st.columns([1, 3])
    with go_col:
        custom_q = st.text_input("Question", key="topic_custom_q", placeholder="e.g. How do muscles work?")
        if st.button("Go", key="topic_go") and custom_q and custom_q.strip():
            st.session_state["pending_starter_question"] = custom_q.strip()
            st.rerun()


# ----- First-time welcome (no profile) -----
if not has_profile():
    _render_welcome_personas()
    st.stop()

# ----- Profile exists: show chat -----
level = get_current_level()
display_label = get_display_label()

# ----- Exit summary (frictionless exit) -----
if st.session_state.get("show_exit_summary") and st.session_state.get("exit_summary"):
    _sum = st.session_state["exit_summary"]
    st.subheader("💤 Session Complete")
    st.markdown("Today you learned:")
    st.markdown("─────────────────")
    for t in _sum.get("topics", []):
        label = t.get("confidence_label", "")
        name = t.get("name", "")
        if label == "solid understanding":
            st.markdown(f"✅ **{name}** (solid understanding)")
        elif label == "good progress":
            st.markdown(f"📚 **{name}** (good progress)")
        else:
            st.markdown(f"🔄 **{name}** (might need review)")
    st.markdown("")
    st.markdown(f"⏱️ Time: {_sum.get('time_minutes', 0)} minutes")
    st.markdown(f"📊 Topics: {len(_sum.get('topics', []))}")
    st.markdown("")
    _next = _sum.get("suggested_next", [])
    if _next:
        st.markdown("Next time you could explore:")
        for n in _next:
            st.markdown(f"- {n}")
        st.markdown("")
    st.markdown(_sum.get("encouragement", "Great session! See you next time. 🌟"))
    st.markdown("")
    col1, col2, _ = st.columns([1, 1, 2])
    with col1:
        if st.button("Start Fresh", key="exit_start_fresh"):
            st.session_state.pop("show_exit_summary", None)
            st.session_state.pop("exit_summary", None)
            st.session_state.pop("progress_session_id", None)
            if "messages" in st.session_state:
                st.session_state["messages"] = []
            st.rerun()
    with col2:
        if st.button("Continue later", key="exit_continue"):
            st.session_state.pop("show_exit_summary", None)
            st.session_state.pop("exit_summary", None)
            st.rerun()
    st.stop()

st.title("🎓 OMEGA Tutor")
st.caption("Ask anything. Learn at your level.")

# Sidebar: Persona, Switch, API key, backend
st.sidebar.markdown(f"**{display_label}**")
if st.sidebar.button("Switch", key="btn_switch_persona"):
    st.session_state["show_persona_switch"] = not st.session_state.get("show_persona_switch", False)
if st.session_state.get("show_persona_switch"):
    st.sidebar.caption("Pick persona:")
    for pid in PERSONA_CONFIG:
        cfg = PERSONA_CONFIG[pid]
        if st.sidebar.button(f"{cfg.get('icon', '')} {cfg.get('label', '').replace(' Mode', '')}", key=f"switch_{pid}"):
            _save_persona(pid)
            st.session_state["show_persona_switch"] = False
            st.rerun()

if st.sidebar.button("🗑️ Clear Chat", key="btn_clear_chat"):
    _clear_chat()
    st.session_state["chat_cleared"] = True
    st.rerun()
if st.session_state.get("chat_cleared"):
    st.sidebar.success("Chat cleared.")
    st.session_state.pop("chat_cleared", None)

_selected_cid = st.session_state.get("selected_curriculum_id")
if curriculum_engine and _selected_cid:
    _curr = curriculum_engine.load_curriculum(_selected_cid) or st.session_state.get("custom_curriculum")
    if _curr:
        _prog = curriculum_engine.get_progress(_selected_cid)
        _pct = _prog.get("percent_complete", 0)
        st.sidebar.markdown(f"**📚 {_curr.get('name', _selected_cid)}**")
        st.sidebar.progress(_pct / 100.0)
        st.sidebar.caption(f"{_pct:.0f}% complete")
st.sidebar.caption("💡 Ask anything below.")

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

# ----- Topic picker (no messages yet) + pending starter question -----
if "messages" not in st.session_state:
    st.session_state["messages"] = []
_messages = st.session_state["messages"]
if len(_messages) == 0:
    if st.session_state.get("pending_starter_question"):
        _q = st.session_state["pending_starter_question"]
        st.session_state["messages"].append({"role": "user", "content": _q})
        _topic_for_fp = st.session_state.get("pending_starter_curriculum_id", "").replace("-", "_") or "general"
        if _engine:
            _resp = _engine.teach(_q, level=level)
            _content_md = _resp.to_markdown()
            st.session_state["messages"].append({"role": "assistant", "content": _content_md, "level": level})
            if record_learning_event:
                try:
                    record_learning_event(
                        topic=_topic_for_fp or "general",
                        level=level,
                        event_type="explanation",
                        content=_content_md[:200] if _content_md else _q[:200],
                        outcome="understood",
                    )
                except Exception:
                    pass
            if generate_preemptive_warning and _topic_for_fp:
                _warn = generate_preemptive_warning(_topic_for_fp)
                if _warn:
                    _warned = st.session_state.get("misconception_warned_topics") or set()
                    if _topic_for_fp not in _warned:
                        st.session_state["pending_misconception_warning"] = _warn
                        st.session_state["misconception_warned_topics"] = _warned | {_topic_for_fp}
        else:
            st.session_state["messages"].append({"role": "assistant", "content": "**No backend available.** Start LM Studio with a model, or set GEMINI_API_KEY.", "level": level})
        st.session_state.pop("pending_starter_question", None)
        st.session_state.pop("pending_starter_curriculum_id", None)
        st.rerun()
    else:
        _render_topic_picker(level)
        st.sidebar.markdown("---")
        st.sidebar.caption("💡 Pick a topic above or type your question.")
        st.stop()

# ----- Voice Mode -----
_profile = load_profile()
_default_voice_on = _profile.get("persona") == "kid"
if "voice_input" not in st.session_state:
    st.session_state["voice_input"] = _default_voice_on
if "voice_output" not in st.session_state:
    st.session_state["voice_output"] = _default_voice_on

st.sidebar.markdown("---")
st.sidebar.markdown("**🎤 Voice Mode**")
st.sidebar.markdown("─────────────")
st.session_state["voice_input"] = st.sidebar.checkbox(
    "Voice input (speak to ask)",
    value=st.session_state["voice_input"],
    key="voice_input_cb",
)
st.session_state["voice_output"] = st.sidebar.checkbox(
    "Voice output (hear responses)",
    value=st.session_state["voice_output"],
    key="voice_output_cb",
)

_voice_selection = "British Female"
if voice_engine and voice_engine.tts_available:
    _voices = voice_engine.get_available_voices()
    if _voices:
        _voice_options = [v[1] for v in _voices]  # display names
        _voice_ids = [v[0] for v in _voices]
        _sel = st.sidebar.selectbox(
            "Voice",
            range(len(_voice_options)),
            format_func=lambda i: _voice_options[i],
            key="voice_select",
        )
        _voice_selection = _voice_ids[_sel] if _sel < len(_voice_ids) else (_voice_ids[0] if _voice_ids else "default")
else:
    st.sidebar.caption("Voice output: pip install edge-tts")
if voice_engine and not voice_engine.whisper_available:
    st.sidebar.caption("Voice input: pip install faster-whisper")

# Voice input: process uploaded audio or from microphone (st.audio_input if available)
if voice_engine and st.session_state.get("voice_input") and voice_engine.whisper_available:
    _audio_upload = st.sidebar.file_uploader("Upload audio", type=["wav", "mp3", "ogg", "webm", "m4a"], key="voice_upload")
    if _audio_upload is not None:
        import tempfile
        _bytes = _audio_upload.read()
        _ext = Path(_audio_upload.name).suffix if getattr(_audio_upload, "name", None) else ".wav"
        if not _ext or _ext not in (".wav", ".mp3", ".ogg", ".webm", ".m4a"):
            _ext = ".wav"
        with tempfile.NamedTemporaryFile(suffix=_ext, delete=False) as _tf:
            _tf.write(_bytes)
            _path = _tf.name
        try:
            _text = voice_engine.transcribe(_path)
            if _text and _text.strip():
                st.session_state["pending_voice_prompt"] = _text.strip()
                st.rerun()
        finally:
            try:
                os.unlink(_path)
            except Exception:
                pass

# Start progress session once per run/session
if progress_tracker and "progress_session_id" not in st.session_state:
    st.session_state["progress_session_id"] = progress_tracker.start_session()

# Your Learning + Progress (clarity: what the system knows)
if progress_tracker:
    st.sidebar.markdown("---")
    st.sidebar.markdown("**Your Learning**")
    st.sidebar.markdown("─────────────")
    try:
        quizzable = progress_tracker.get_quizzable_topics(50)
        due_list = progress_tracker.get_due_reviews()
        topics_learned = len(quizzable)
        ready_to_quiz = topics_learned
        due_count = len(due_list)
        st.sidebar.metric("Topics learned", topics_learned)
        st.sidebar.metric("Ready to quiz", ready_to_quiz)
        st.sidebar.metric("Due for review", due_count)
        if due_count > 0 and st.sidebar.button("Review due topics", key="btn_review_due"):
            st.session_state["show_review"] = True
            st.rerun()
        if st.sidebar.button("See all topics", key="btn_see_topics"):
            st.session_state["show_all_topics"] = not st.session_state.get("show_all_topics", False)
            st.rerun()
        if st.session_state.get("show_all_topics") and quizzable:
            st.sidebar.caption(", ".join(quizzable[:15]) + ("..." if len(quizzable) > 15 else ""))
    except Exception:
        st.sidebar.caption("Progress unavailable.")
    _show_dash = should_show_dashboard(st.session_state) if should_show_dashboard else True
    if _show_dash:
        st.sidebar.markdown("**📊 This Week**")
        st.sidebar.markdown("─────────────")
        try:
            ws = progress_tracker.get_weekly_stats()
            st.sidebar.metric("Questions", f"{ws.get('questions', 0)} ({ws.get('correct_rate', 0):.0f}% correct)")
            streak = progress_tracker.get_streak()
            st.sidebar.metric("Streak", f"{streak} days 🔥" if streak else "0 days")
        except Exception:
            pass

# Learning Fingerprint section
if get_fingerprint_summary:
    st.sidebar.divider()
    st.sidebar.subheader("🧠 Learning Profile")
    if st.sidebar.button("View My Profile", key="btn_show_fingerprint"):
        st.session_state["show_fingerprint"] = not st.session_state.get("show_fingerprint", False)
    if st.session_state.get("show_fingerprint"):
        try:
            summary = get_fingerprint_summary()
            st.sidebar.write(f"**Topics studied:** {summary.get('topics_studied', 0)}")
            st.sidebar.write(f"**Total events:** {summary.get('total_events', 0)}")
            if summary.get("best_topics"):
                st.sidebar.write(f"**Strong areas:** {', '.join(summary['best_topics'][:3])}")
            if summary.get("struggling_topics"):
                st.sidebar.write(f"**Areas to work on:** {', '.join(summary['struggling_topics'][:3])}")
            style = summary.get("learning_style", {})
            prefs = []
            if style.get("prefers_analogies"):
                prefs.append("analogies")
            if style.get("prefers_examples"):
                prefs.append("examples")
            if style.get("prefers_formal"):
                prefs.append("formal definitions")
            if prefs:
                st.sidebar.write(f"**You learn best with:** {', '.join(prefs)}")
        except Exception:
            st.sidebar.caption("Profile unavailable.")

# Quiz button and Due for review (always show; not tied to fingerprint)
_show_quiz_btn = should_show_quiz(st.session_state) if should_show_quiz else True
if _show_quiz_btn and st.sidebar.button("🧠 Test Yourself"):
    st.session_state["quiz_mode"] = True
    st.session_state.pop("quiz_questions", None)
    st.session_state.pop("quiz_index", None)
    st.session_state.pop("quiz_answers", None)
    st.session_state.pop("quiz_topic", None)
    st.session_state.pop("quiz_topic_list", None)
    st.session_state.pop("quiz_source_type", None)
    st.session_state.pop("quiz_n_questions", None)
    st.rerun()
if st.session_state.get("show_review"):
    st.sidebar.markdown("---")
    st.sidebar.markdown("**Due for review**")
    st.sidebar.markdown("─────────────")
    try:
        sm2_due = progress_tracker.get_due_reviews() if progress_tracker else []
        due = progress_tracker.get_due_for_review() if progress_tracker else []
        combined = list({d.get("name", ""): d for d in (sm2_due + due) if d.get("name")}.values())[:10]
        review_selected = st.session_state.get("review_selected_topic")
        for i, d in enumerate(combined):
            name = d.get("name", "")
            if st.sidebar.button(f"📌 {name}", key=f"review_btn_{i}_{name[:25]}", type="secondary"):
                st.session_state["review_selected_topic"] = name if review_selected != name else None
                st.rerun()
            if review_selected == name:
                c1, c2 = st.sidebar.columns(2)
                with c1:
                    if c1.button("Quiz me", key=f"review_quiz_{i}_{name[:20]}"):
                        st.session_state["quiz_mode"] = True
                        st.session_state["quiz_topic_list"] = [name]
                        st.session_state["quiz_n_questions"] = 5
                        st.session_state.pop("quiz_questions", None)
                        st.session_state.pop("quiz_index", None)
                        st.session_state.pop("quiz_answers", None)
                        st.session_state["review_selected_topic"] = None
                        st.session_state["show_review"] = False
                        st.rerun()
                with c2:
                    if c2.button("Re-learn", key=f"review_teach_{i}_{name[:20]}"):
                        st.session_state["pending_starter_question"] = f"Explain {name} again in simple terms."
                        st.session_state["messages"] = []
                        st.session_state["review_selected_topic"] = None
                        st.session_state["show_review"] = False
                        st.rerun()
        if not combined:
            st.sidebar.caption("Nothing due. Keep learning!")
        if st.sidebar.button("Close", key="close_review"):
            st.session_state["show_review"] = False
            st.session_state.pop("review_selected_topic", None)
            st.rerun()
    except Exception:
        st.sidebar.caption("Review list unavailable.")

# ----- Learning Paths (Curriculum) -----
if curriculum_engine:
    st.sidebar.markdown("---")
    st.sidebar.markdown("**📚 Learning Paths**")
    st.sidebar.markdown("─────────────────")
    curricula = curriculum_engine.list_curricula()
    options = ["— Select a curriculum —"] + [f"{c['name']} ({c['topic_count']} topics)" for c in curricula]
    curriculum_ids = [None] + [c["id"] for c in curricula]
    sel_idx = st.sidebar.selectbox(
        "Path",
        range(len(options)),
        format_func=lambda i: options[i],
        key="curriculum_select",
    )
    selected_curriculum_id = curriculum_ids[sel_idx] if sel_idx < len(curriculum_ids) else None
    st.session_state["selected_curriculum_id"] = selected_curriculum_id
    if st.sidebar.button("+ Create Custom Path", key="custom_curriculum_btn"):
        st.session_state["show_custom_curriculum"] = True
    if st.session_state.get("show_custom_curriculum"):
        st.sidebar.caption("What do you want to learn? (comma-separated topics)")
        custom_topics = st.sidebar.text_input("Topics", key="custom_topics", placeholder="e.g. tendons, collagen, actuators")
        if st.sidebar.button("Generate Path", key="gen_path") and custom_topics:
            topics_list = [t.strip() for t in custom_topics.split(",") if t.strip()]
            custom = curriculum_engine.generate_custom_curriculum(topics_list, api_key=os.environ.get("GEMINI_API_KEY") or st.session_state.get("api_key"))
            custom["id"] = "custom"
            st.session_state["custom_curriculum"] = custom
            st.session_state["selected_curriculum_id"] = "custom"
            st.session_state["show_custom_curriculum"] = False
            try:
                import json
                path = Path(__file__).resolve().parent / "data" / "curricula" / "custom.json"
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(custom, indent=2), encoding="utf-8")
            except Exception:
                pass
            st.rerun()
        if st.sidebar.button("Cancel", key="cancel_custom"):
            st.session_state["show_custom_curriculum"] = False
            st.rerun()

# ----- Frictionless exit -----
st.sidebar.markdown("---")
if st.sidebar.button("💤 That's enough for now", key="btn_exit_session"):
    _sid = st.session_state.get("progress_session_id")
    if progress_tracker and _sid is not None:
        progress_tracker.end_session(_sid)
    if generate_summary and _sid is not None:
        try:
            _summary = generate_summary(_sid, st.session_state.get("selected_curriculum_id"))
            st.session_state["exit_summary"] = _summary
            st.session_state["show_exit_summary"] = True
        except Exception:
            st.session_state["exit_summary"] = {"topics": [], "time_minutes": 0, "suggested_next": [], "encouragement": "Great session! See you next time. 🌟"}
            st.session_state["show_exit_summary"] = True
    else:
        st.session_state["exit_summary"] = {"topics": [], "time_minutes": 0, "suggested_next": [], "encouragement": "Great session! See you next time. 🌟"}
        st.session_state["show_exit_summary"] = True
    st.session_state.pop("progress_session_id", None)
    st.rerun()

# ----- Quiz Mode -----
if st.session_state.get("quiz_mode") and progress_tracker and (generate_quiz or generate_quiz_from_topics):
    # Setup: show source + n_questions unless we already have quiz_topic_list (e.g. from "Quiz me" on review)
    quiz_topic_list = st.session_state.get("quiz_topic_list")
    if ("quiz_questions" not in st.session_state or not st.session_state["quiz_questions"]) and not quiz_topic_list:
        st.subheader("🧠 Quiz Mode")
        st.markdown("─────────────")
        st.markdown("**What do you want to be quizzed on?**")
        quizzable = progress_tracker.get_quizzable_topics(50)
        due_list = progress_tracker.get_due_reviews()
        due_names = [d["name"] for d in due_list]
        curricula = curriculum_engine.list_curricula() if curriculum_engine else []
        options = ["Topics I've learned (from chat history)"]
        option_sources = ["learned"]
        for c in curricula[:5]:
            options.append(f"Curriculum: {c.get('name', c.get('id', ''))}")
            option_sources.append(f"curriculum:{c.get('id', '')}")
        options.append(f"Topics due for review ({len(due_names)} topics)")
        option_sources.append("due")
        options.append("Everything")
        option_sources.append("everything")
        idx_sel = st.radio("Source", range(len(options)), format_func=lambda i: options[i], key="quiz_source_radio")
        source_type = option_sources[idx_sel] if idx_sel < len(option_sources) else "learned"
        n_opts = [3, 5, 10, 20]
        n_sel = st.selectbox("How many questions?", n_opts, index=0, key="quiz_n_select")
        if st.button("Start Quiz", key="quiz_start_btn"):
            if source_type == "learned":
                topics = quizzable[:20] if quizzable else ["general"]
            elif source_type == "due":
                topics = due_names[:20] if due_names else (quizzable[:5] if quizzable else ["general"])
            elif source_type == "everything":
                topics = quizzable[:20] if quizzable else ["general"]
            elif source_type.startswith("curriculum:"):
                cid = source_type.split(":", 1)[1]
                completed = curriculum_engine.get_completed_topics_for_quiz(cid) if curriculum_engine else []
                topics = [t.get("name", t.get("id", "")) for t in completed] if completed else ["general"]
            else:
                topics = quizzable[:20] if quizzable else ["general"]
            st.session_state["quiz_topic_list"] = topics
            st.session_state["quiz_n_questions"] = n_sel
            st.session_state["quiz_source_type"] = source_type
            try:
                if generate_quiz_from_topics:
                    qs = generate_quiz_from_topics(topics, level, n_questions=n_sel, api_key=os.environ.get("GEMINI_API_KEY") or st.session_state.get("api_key"))
                else:
                    qs = generate_quiz(topics[0] if topics else "general", level, n_questions=n_sel, api_key=os.environ.get("GEMINI_API_KEY") or st.session_state.get("api_key"))
                    for q in qs:
                        q["topic"] = topics[0] if topics else "general"
                st.session_state["quiz_questions"] = qs
            except Exception:
                st.session_state["quiz_questions"] = []
            st.session_state["quiz_index"] = 0
            st.session_state["quiz_answers"] = []
            st.session_state.pop("quiz_show_feedback", None)
            st.rerun()
        st.stop()
    # Generate from quiz_topic_list if we have it but no questions yet
    if ("quiz_questions" not in st.session_state or not st.session_state["quiz_questions"]) and quiz_topic_list:
        n_quiz = st.session_state.get("quiz_n_questions", 5)
        try:
            if generate_quiz_from_topics:
                qs = generate_quiz_from_topics(quiz_topic_list, level, n_questions=n_quiz, api_key=os.environ.get("GEMINI_API_KEY") or st.session_state.get("api_key"))
            else:
                qs = generate_quiz(quiz_topic_list[0], level, n_questions=n_quiz, api_key=os.environ.get("GEMINI_API_KEY") or st.session_state.get("api_key"))
                for q in qs:
                    q["topic"] = quiz_topic_list[0]
            st.session_state["quiz_questions"] = qs
        except Exception:
            st.session_state["quiz_questions"] = []
        st.session_state["quiz_index"] = 0
        st.session_state["quiz_answers"] = []
        st.session_state.pop("quiz_show_feedback", None)
        st.rerun()
    qs = st.session_state.get("quiz_questions", [])
    if not qs:
        st.caption("No quiz questions could be generated. Try learning a topic first.")
        if st.button("Back to Learning", key="quiz_back_empty"):
            st.session_state["quiz_mode"] = False
            st.session_state.pop("quiz_questions", None)
            st.session_state.pop("quiz_index", None)
            st.session_state.pop("quiz_answers", None)
            st.session_state.pop("quiz_topic", None)
            st.session_state.pop("quiz_topic_list", None)
            st.rerun()
        st.stop()
    st.subheader("🧠 Quiz Time!")
    idx = st.session_state.get("quiz_index", 0)
    quiz_answers = st.session_state.get("quiz_answers", [])
    show_feedback = st.session_state.get("quiz_show_feedback", False)
    # Show feedback for current question (after submit, before continue)
    if idx < len(qs) and show_feedback and idx < len(quiz_answers):
        a = quiz_answers[idx]
        st.markdown(f"**Question {idx + 1} of {len(qs)}**")
        st.markdown(a.get("question", ""))
        if a.get("was_correct"):
            st.success("✅ Correct!")
            st.markdown(f"**{a.get('correct_answer', '')}**")
            if a.get("explanation"):
                st.caption("💡 " + (a["explanation"] or ""))
            if st.button("Continue to next question", key="quiz_continue"):
                st.session_state["quiz_show_feedback"] = False
                st.session_state["quiz_index"] = idx + 1
                st.rerun()
        else:
            st.error("❌ Incorrect")
            st.markdown(f"**Your answer:** {a.get('user_answer', '')}")
            st.markdown(f"**Correct answer:** {a.get('correct_answer', '')}")
            if a.get("explanation"):
                st.info("💡 " + (a["explanation"] or ""))
            col_c, col_l = st.columns(2)
            with col_c:
                if st.button("Continue", key="quiz_continue"):
                    st.session_state["quiz_show_feedback"] = False
                    st.session_state["quiz_index"] = idx + 1
                    st.rerun()
            with col_l:
                if st.button("🎓 Learn more about this", key="quiz_learn_more"):
                    concept = a.get("topic", "this concept")
                    st.session_state["quiz_state"] = {
                        "active": True,
                        "topic": concept,
                        "questions": qs,
                        "current_index": idx + 1,
                        "results": list(quiz_answers),
                        "paused_for_learning": True,
                        "completed": False,
                    }
                    st.session_state["pending_starter_question"] = f"Explain {concept} in detail. I got a quiz question wrong about this."
                    st.session_state["messages"] = []
                    st.session_state["quiz_mode"] = False
                    st.session_state.pop("quiz_questions", None)
                    st.session_state.pop("quiz_index", None)
                    st.session_state.pop("quiz_answers", None)
                    st.session_state.pop("quiz_show_feedback", None)
                    st.rerun()
        st.stop()
    if idx < len(qs):
        q = qs[idx]
        q_topic = q.get("topic", st.session_state.get("quiz_topic", ""))
        st.markdown(f"**Question {idx + 1} of {len(qs)}** _(topic: {q_topic})_")
        st.markdown(q.get("question", ""))
        options = q.get("options", ["A", "B", "C", "D"])
        correct_letter = str(q.get("correct", "A")).upper()
        correct_idx = ord(correct_letter) - ord("A") if correct_letter in "ABCD" else 0
        correct_answer_text = options[correct_idx] if 0 <= correct_idx < len(options) else ""
        choice = st.radio("Choose one:", options, key="quiz_choice", format_func=lambda x: x)
        if st.button("Submit Answer", key="quiz_submit"):
            correct = choice in options and options.index(choice) == correct_idx
            quality = 5 if correct else (2 if choice else 0)
            progress_tracker.schedule_review(q_topic, quality)
            progress_tracker.record_question(q_topic, correct)
            result = {
                "question": q.get("question", ""),
                "topic": q_topic,
                "options": options,
                "user_answer": choice,
                "correct_answer": correct_answer_text,
                "correct_letter": correct_letter,
                "explanation": q.get("explanation", ""),
                "was_correct": correct,
            }
            st.session_state["quiz_answers"] = st.session_state.get("quiz_answers", []) + [result]
            st.session_state["quiz_show_feedback"] = True
            if record_learning_event:
                try:
                    record_learning_event(
                        topic=(q_topic or "general").replace(" ", "_").replace("-", "_"),
                        level=level,
                        event_type="quiz",
                        content=(q.get("question", "") or "")[:200],
                        outcome="correct" if correct else "incorrect",
                    )
                except Exception:
                    pass
            st.rerun()
        st.stop()
    # Results: all questions answered
    answers = st.session_state.get("quiz_answers", [])
    correct_count = sum(1 for a in answers if a.get("was_correct"))
    total = len(answers)
    pct = (100 * correct_count // total) if total else 0
    quiz_topics = list({a.get("topic", "") for a in answers if a.get("topic")})
    title_topic = quiz_topics[0] if len(quiz_topics) == 1 else "Mixed topics"
    # Store quiz_state so we can "Return to quiz summary" after learning
    st.session_state["quiz_state"] = {
        "active": False,
        "topic": title_topic,
        "questions": qs,
        "current_index": total,
        "results": list(answers),
        "paused_for_learning": False,
        "completed": True,
        "viewing_summary": False,
    }
    st.markdown("**Quiz Complete:** " + (f"_{title_topic}_" if title_topic else ""))
    st.markdown("─────────────")
    st.metric("Score", f"{correct_count}/{total} ({pct}%)")
    wrong_count = 0
    for i, a in enumerate(answers):
        q_num = i + 1
        if a.get("was_correct"):
            st.success(f"✅ Q{q_num}: {a.get('question', '')[:80]}{'…' if len(a.get('question', '')) > 80 else ''}")
            st.caption(f"Correct! {a.get('correct_answer', '')}")
            if a.get("explanation"):
                st.caption("💡 " + (a["explanation"] or "")[:200] + ("…" if len(a.get("explanation", "")) > 200 else ""))
        else:
            wrong_count += 1
            st.warning(f"❌ Q{q_num}: {a.get('question', '')}")
            st.caption(f"**Your answer:** {a.get('user_answer', '')} → **Correct:** {a.get('correct_answer', '')}")
            if a.get("explanation"):
                st.info("💡 " + (a["explanation"] or ""))
            learn_col, _ = st.columns([1, 3])
            with learn_col:
                if st.button("🎓 Learn more", key=f"final_learn_{i}"):
                    concept = a.get("topic", "this concept")
                    st.session_state["quiz_state"] = {**st.session_state["quiz_state"], "viewing_summary": True}
                    st.session_state["pending_starter_question"] = f"Explain {concept} in detail. I got a quiz question wrong about this."
                    st.session_state["messages"] = []
                    st.session_state["quiz_mode"] = False
                    st.session_state.pop("quiz_questions", None)
                    st.session_state.pop("quiz_index", None)
                    st.session_state.pop("quiz_answers", None)
                    st.session_state.pop("quiz_show_feedback", None)
                    st.rerun()
        st.markdown("")
    st.caption("Topics have been scheduled for review.")
    col1, col2, col3 = st.columns(3)
    with col1:
        if wrong_count > 0 and st.button("🎓 Review all wrong answers", key="review_all_wrong"):
            wrong_topics = list({a.get("topic", "this concept") for a in answers if not a.get("was_correct")})
            st.session_state["quiz_state"] = {**st.session_state["quiz_state"], "viewing_summary": True}
            st.session_state["pending_starter_question"] = (
                "I got these quiz questions wrong: " + ", ".join(wrong_topics) + ". Please explain each concept."
            )
            st.session_state["messages"] = []
            st.session_state["quiz_mode"] = False
            st.session_state.pop("quiz_questions", None)
            st.session_state.pop("quiz_index", None)
            st.session_state.pop("quiz_answers", None)
            st.session_state.pop("quiz_show_feedback", None)
            st.rerun()
    with col2:
        if st.button("🔄 Quiz again", key="quiz_again"):
            st.session_state.pop("quiz_state", None)
            st.session_state.pop("quiz_questions", None)
            st.session_state.pop("quiz_index", None)
            st.session_state.pop("quiz_answers", None)
            st.session_state.pop("quiz_show_feedback", None)
            st.session_state.pop("quiz_topic_list", None)
            st.rerun()
    with col3:
        if st.button("← Back to learning", key="quiz_back"):
            st.session_state["quiz_mode"] = False
            st.session_state.pop("quiz_questions", None)
            st.session_state.pop("quiz_index", None)
            st.session_state.pop("quiz_answers", None)
            st.session_state.pop("quiz_show_feedback", None)
            st.session_state.pop("quiz_topic", None)
            st.session_state.pop("quiz_topic_list", None)
            st.session_state.pop("quiz_source_type", None)
            st.session_state.pop("quiz_n_questions", None)
            st.session_state.pop("quiz_state", None)
            st.rerun()
    st.stop()

# ----- Quiz paused / return from learning -----
_quiz_state = st.session_state.get("quiz_state")
if _quiz_state and not st.session_state.get("quiz_mode"):
    if st.session_state.get("show_quiz_summary") and _quiz_state.get("results") is not None:
        # Re-display quiz summary after "Return to quiz summary"
        st.subheader("🧠 Quiz Complete: " + (_quiz_state.get("topic") or "Quiz"))
        st.markdown("─────────────")
        _results = _quiz_state["results"]
        _total = len(_results)
        _correct = sum(1 for a in _results if a.get("was_correct"))
        _wrong_count = _total - _correct
        _pct = (100 * _correct // _total) if _total else 0
        st.metric("Score", f"{_correct}/{_total} ({_pct}%)")
        for i, a in enumerate(_results):
            q_num = i + 1
            if a.get("was_correct"):
                st.success(f"✅ Q{q_num}: {a.get('question', '')[:80]}{'…' if len(a.get('question', '')) > 80 else ''}")
                if a.get("explanation"):
                    st.caption("💡 " + (a["explanation"] or "")[:200] + ("…" if len(a.get("explanation", "")) > 200 else ""))
            else:
                st.warning(f"❌ Q{q_num}: {a.get('question', '')}")
                st.caption(f"**Your answer:** {a.get('user_answer', '')} → **Correct:** {a.get('correct_answer', '')}")
                if a.get("explanation"):
                    st.info("💡 " + (a["explanation"] or ""))
                if st.button("🎓 Learn more", key=f"summary_learn_{i}"):
                    concept = a.get("topic", "this concept")
                    st.session_state["quiz_state"] = {**_quiz_state, "viewing_summary": True, "completed": True}
                    st.session_state["pending_starter_question"] = f"Explain {concept} in detail. I got a quiz question wrong about this."
                    st.session_state["messages"] = []
                    st.session_state.pop("show_quiz_summary", None)
                    st.rerun()
            st.markdown("")
        col1, col2, col3, _ = st.columns([1, 1, 1, 2])
        with col1:
            if _wrong_count > 0 and st.button("🎓 Review all wrong answers", key="summary_review_all"):
                wrong_topics = list({a.get("topic", "this concept") for a in _results if not a.get("was_correct")})
                st.session_state["quiz_state"] = {**_quiz_state, "viewing_summary": True}
                st.session_state["pending_starter_question"] = (
                    "I got these quiz questions wrong: " + ", ".join(wrong_topics) + ". Please explain each concept."
                )
                st.session_state["messages"] = []
                st.session_state.pop("show_quiz_summary", None)
                st.rerun()
        with col2:
            if st.button("🔄 Quiz again", key="summary_quiz_again"):
                st.session_state.pop("quiz_state", None)
                st.session_state.pop("show_quiz_summary", None)
                st.session_state["quiz_mode"] = True
                st.session_state.pop("quiz_questions", None)
                st.session_state.pop("quiz_index", None)
                st.session_state.pop("quiz_answers", None)
                st.rerun()
        with col3:
            if st.button("← Back to learning", key="summary_back"):
                st.session_state.pop("quiz_state", None)
                st.session_state.pop("show_quiz_summary", None)
                st.rerun()
        st.stop()
    if _quiz_state.get("paused_for_learning"):
        st.info("**You paused the quiz to learn.** Continue where you left off or start a new quiz.")
        col1, col2, _ = st.columns([1, 1, 2])
        with col1:
            if st.button("Return to quiz", key="return_to_quiz"):
                qs = _quiz_state.get("questions", [])
                st.session_state["quiz_questions"] = qs
                st.session_state["quiz_index"] = _quiz_state.get("current_index", 0)
                st.session_state["quiz_answers"] = list(_quiz_state.get("results", []))
                st.session_state["quiz_mode"] = True
                st.session_state["quiz_state"] = {**_quiz_state, "paused_for_learning": False}
                st.session_state.pop("quiz_show_feedback", None)
                st.rerun()
        with col2:
            if st.button("Start new quiz", key="start_new_quiz"):
                st.session_state.pop("quiz_state", None)
                st.session_state["quiz_mode"] = True
                st.session_state.pop("quiz_questions", None)
                st.session_state.pop("quiz_index", None)
                st.session_state.pop("quiz_answers", None)
                st.session_state.pop("quiz_topic_list", None)
                st.rerun()
        st.markdown("---")
    elif _quiz_state.get("completed") and _quiz_state.get("viewing_summary"):
        st.info("**You came from the quiz summary.** Return to see your results again.")
        if st.button("Return to quiz summary", key="return_to_summary"):
            st.session_state["show_quiz_summary"] = True
            st.rerun()
        st.markdown("---")

# ----- Curriculum panel (when a path is selected) -----
selected_cid = st.session_state.get("selected_curriculum_id")
if curriculum_engine and selected_cid:
    curr = curriculum_engine.load_curriculum(selected_cid) or st.session_state.get("custom_curriculum")
    if curr:
        progress = curriculum_engine.get_progress(selected_cid)
        st.subheader(curr.get("name", "Learning Path"))
        pct = progress.get("percent_complete", 0)
        st.progress(pct / 100.0)
        st.caption(f"{pct:.0f}% complete")
        completed = progress.get("completed", [])
        available = progress.get("available", [])
        locked = progress.get("locked", [])
        for t in completed:
            topic_name = t.get("name", t.get("id", ""))
            st.success(f"✅ {topic_name}")
            c1, c2 = st.columns(2)
            with c1:
                if st.button(f"Quiz this topic", key=f"curr_quiz_{selected_cid}_{t.get('id', '')}"):
                    st.session_state["quiz_mode"] = True
                    st.session_state["quiz_topic_list"] = [topic_name]
                    st.session_state["quiz_n_questions"] = 5
                    st.session_state.pop("quiz_questions", None)
                    st.session_state.pop("quiz_index", None)
                    st.session_state.pop("quiz_answers", None)
                    st.rerun()
            with c2:
                if st.button(f"Review notes", key=f"curr_review_{selected_cid}_{t.get('id', '')}"):
                    st.session_state["pending_starter_question"] = f"What were the key points of {topic_name}?"
                    st.session_state["messages"] = []
                    st.rerun()
        for t in available:
            topic_name = t.get("name", t.get("id", ""))
            if st.button(f"▶️ Learn: {topic_name}", key=f"learn_{selected_cid}_{t.get('id', '')}"):
                st.session_state["curriculum_learn"] = {"curriculum_id": selected_cid, "topic_id": t.get("id", ""), "topic_name": topic_name}
                st.rerun()
        for t in locked:
            prereqs = t.get("prerequisites", [])
            curr_topics = {x.get("id"): x.get("name") for x in (curr.get("topics") or [])}
            needs = ", ".join(curr_topics.get(p, p) for p in prereqs)
            st.caption(f"🔒 {t.get('name', t.get('id', ''))} (needs: {needs})")
        if st.session_state.get("curriculum_learn"):
            cl = st.session_state["curriculum_learn"]
            st.info(f"**Learning:** {cl.get('topic_name', '')}. Ask below or click to teach.")
            if st.button(f"Ask about {cl.get('topic_name', '')}", key="curriculum_ask_btn"):
                q = f"Teach me about {cl.get('topic_name', '')}"
                if "messages" not in st.session_state:
                    st.session_state.messages = []
                st.session_state.messages.append({"role": "user", "content": q})
                if _engine:
                    eng = _engine
                    resp = eng.teach(q, level=level)
                    st.session_state.messages.append({"role": "assistant", "content": resp.to_markdown(), "level": level})
                curriculum_engine.mark_complete(cl["curriculum_id"], cl["topic_id"])
                if progress_tracker:
                    progress_tracker.record_topic(cl.get("topic_name", ""))
                    progress_tracker.record_question(cl.get("topic_name", ""), True)
                st.session_state.pop("curriculum_learn", None)
                st.rerun()
    st.markdown("---")

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
        _resp_md = _resp.to_markdown()
        st.session_state.messages.append({"role": "assistant", "content": _resp_md, "level": level})
        if record_learning_event:
            try:
                _topic_fp = (topic or "general").replace(" ", "_").replace("-", "_")
                record_learning_event(
                    topic=_topic_fp or "general",
                    level=level,
                    event_type="explanation",
                    content=_resp_md[:200] if _resp_md else _topic_q[:200],
                    outcome="understood",
                )
            except Exception:
                pass
        if generate_preemptive_warning and topic:
            try:
                _warn = generate_preemptive_warning((topic or "").replace(" ", "_").replace("-", "_"))
                if _warn:
                    _warned = st.session_state.get("misconception_warned_topics") or set()
                    _tkey = (topic or "general").replace(" ", "_")
                    if _tkey not in _warned:
                        st.session_state["pending_misconception_warning"] = _warn
                        st.session_state["misconception_warned_topics"] = _warned | {_tkey}
            except Exception:
                pass
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

# Gentler UI when overwhelmed (cognitive load)
if is_overwhelmed is not None and is_overwhelmed(st.session_state):
    st.info("Take your time. Want me to explain that differently? Use **Simpler** below or just ask in your own words.")

# Optional: microphone in main area (Streamlit 1.32+)
if st.session_state.get("voice_input") and voice_engine and voice_engine.whisper_available and getattr(st, "audio_input", None):
    _mic_audio = st.audio_input("Speak to ask")
    if _mic_audio is not None:
        import tempfile
        _mic_bytes = _mic_audio.read()
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as _mtf:
            _mtf.write(_mic_bytes)
            _mpath = _mtf.name
        try:
            _mtext = voice_engine.transcribe(_mpath)
            if _mtext and _mtext.strip():
                st.session_state["pending_voice_prompt"] = _mtext.strip()
                st.rerun()
        finally:
            try:
                os.unlink(_mpath)
            except Exception:
                pass

messages = st.session_state.messages
if st.session_state.get("pending_misconception_warning"):
    st.warning(st.session_state["pending_misconception_warning"])
    st.session_state.pop("pending_misconception_warning", None)
for i, msg in enumerate(messages):
    with st.chat_message(msg["role"]):
        st.markdown(msg["content"])
        if msg.get("role") == "assistant":
            _tts_audio = st.session_state.get("last_tts_audio")
            if st.session_state.get("voice_output") and i == st.session_state.get("last_tts_message_idx") and _tts_audio:
                _fmt = "audio/mp3" if _tts_audio[:3] == b"ID3" else "audio/wav"
                st.audio(_tts_audio, format=_fmt)
            msg_level = msg.get("level") or level
            st.caption(f"Explaining at: **{msg_level}** level")
            simpler_next = adjust_level(msg_level, "simpler")
            deeper_next = adjust_level(msg_level, "deeper")
            btn_col1, btn_col2, btn_col3, _ = st.columns([1, 1, 1, 2])
            with btn_col1:
                simpler_clicked = st.button("😕 Simpler", key=f"simpler_{i}", disabled=(simpler_next is None))
            with btn_col2:
                deeper_clicked = st.button("🔬 Deeper", key=f"deeper_{i}", disabled=(deeper_next is None))
            with btn_col3:
                explain_back_clicked = st.button("🧠 Explain it back", key=f"explain_{i}") if st.session_state.get("explain_back_index") != i else False
            if simpler_clicked and simpler_next:
                st.session_state["re_explain"] = {"direction": "simpler", "msg_index": i}
                st.rerun()
            if deeper_clicked and deeper_next:
                st.session_state["re_explain"] = {"direction": "deeper", "msg_index": i}
                st.rerun()
            if explain_back_clicked:
                st.session_state["explain_back_index"] = i
                st.session_state["explain_back_topic"] = messages[i - 1].get("content", "").strip()[:80] if i > 0 else "this topic"
                st.rerun()
            if st.session_state.get("explain_back_index") == i and evaluate_explanation:
                st.markdown("**Explain it back**")
                user_exp = st.text_area("Explain in your own words:", key=f"explain_text_{i}", placeholder="What did you learn?")
                if st.button("Submit", key=f"explain_submit_{i}") and user_exp.strip():
                    topic_name = st.session_state.get("explain_back_topic", "this topic")
                    _topic_key = (topic_name or "general").replace(" ", "_").replace("-", "_")[:80]
                    misconception_check = check_explanation_for_misconceptions(topic_name, user_exp) if check_explanation_for_misconceptions else {}
                    if misconception_check.get("has_misconceptions"):
                        st.warning(f"⚠️ I detected a possible misconception (severity: {misconception_check.get('severity', 'moderate')})")
                        for correction in misconception_check.get("corrections", []):
                            st.info(correction)
                        if record_learning_event:
                            try:
                                record_learning_event(
                                    topic=_topic_key,
                                    level=msg_level,
                                    event_type="explain_back",
                                    content=user_exp[:200],
                                    outcome="confused",
                                    notes=f"Misconception: {misconception_check.get('detected', [{}])[0].get('misconception', '')}",
                                )
                            except Exception:
                                pass
                    else:
                        if record_learning_event:
                            try:
                                record_learning_event(
                                    topic=_topic_key,
                                    level=msg_level,
                                    event_type="explain_back",
                                    content=user_exp[:200],
                                    outcome="understood",
                                )
                            except Exception:
                                pass
                    ev = evaluate_explanation(topic_name, user_exp, msg_level, api_key=os.environ.get("GEMINI_API_KEY") or st.session_state.get("api_key"))
                    st.metric("Score", f"{ev.get('score', 0)}/100")
                    if ev.get("correct_points"):
                        st.success("You got right: " + "; ".join(ev["correct_points"][:3]))
                    if ev.get("missing_points"):
                        st.info("Also important: " + "; ".join(ev["missing_points"][:3]))
                    if ev.get("misconceptions"):
                        st.warning("Watch out: " + "; ".join(ev["misconceptions"][:2]))
                    st.caption(ev.get("feedback", ""))
                    if progress_tracker:
                        delta = (ev.get("score", 50) - 50) / 100.0
                        try:
                            from core import memory
                            memory.update_confidence(topic_name, delta)
                        except Exception:
                            pass
                        progress_tracker.update_topic_confidence(topic_name, ev.get("score", 50) / 100.0)
                    st.session_state["explain_back_last_score"] = ev.get("score", 50)
                    st.session_state.pop("explain_back_index", None)
                    st.session_state.pop("explain_back_topic", None)
                    st.rerun()

# Prompt from chat input or from voice (pending_voice_prompt)
prompt = None
if st.session_state.get("pending_voice_prompt"):
    _p = st.session_state.pop("pending_voice_prompt")
    if detect_voice_command:
        _cmd = detect_voice_command(_p)
        if _cmd == "quiz":
            st.session_state["quiz_mode"] = True
            st.session_state.pop("quiz_questions", None)
            st.session_state.pop("quiz_index", None)
            st.session_state.pop("quiz_answers", None)
            st.session_state.pop("quiz_topic", None)
            st.rerun()
        elif _cmd == "stop":
            st.session_state.pop("last_tts_audio", None)
            st.session_state.pop("last_tts_message_idx", None)
            st.rerun()
        elif _cmd in ("simpler", "deeper") and "messages" in st.session_state and st.session_state.messages:
            _msgs = st.session_state.messages
            _last_idx = len(_msgs) - 1
            if _last_idx >= 0 and _msgs[_last_idx].get("role") == "assistant":
                st.session_state["re_explain"] = {"direction": _cmd, "msg_index": _last_idx}
                st.rerun()
        elif _cmd is None:
            prompt = _p
    else:
        prompt = _p
if prompt is None:
    prompt = st.chat_input("What would you like to learn about?")

if prompt:
    st.session_state.pop("explain_back_last_score", None)
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)
    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            engine = _engine if _engine is not None else TutorEngine(api_key=api_key or None)
            response = engine.teach(prompt, level=level)
            st.markdown(response.to_markdown())
    _content = response.to_markdown()
    _topic_fp = (prompt or "").strip()[:80].replace("\n", " ").strip() or "general"
    if record_learning_event:
        try:
            record_learning_event(
                topic=_topic_fp,
                level=level,
                event_type="explanation",
                content=(getattr(response, "explanation", None) or _content)[:200],
                outcome="understood",
            )
        except Exception:
            pass
    if generate_preemptive_warning:
        try:
            _warn = generate_preemptive_warning(_topic_fp.replace(" ", "_").replace("-", "_"))
            if _warn:
                _warned = st.session_state.get("misconception_warned_topics") or set()
                if _topic_fp not in _warned:
                    st.session_state["pending_misconception_warning"] = _warn
                    st.session_state["misconception_warned_topics"] = _warned | {_topic_fp}
        except Exception:
            pass
    try:
        _bp = _ROOT.parent / "breakthrough_engine"
        if _bp.exists() and str(_bp) not in sys.path:
            sys.path.insert(0, str(_bp))
        from hypothesis_ledger import HypothesisLedger
        _ledger = HypothesisLedger()
        _hyps_about = _ledger.get_active_hypotheses_about(prompt)
        if _hyps_about:
            _content += "\n\n---\n*You have an active hypothesis about this: " + ", ".join([f"**{h['id']}**" for h in _hyps_about]) + "*"
    except Exception:
        pass
    st.session_state.messages.append({"role": "assistant", "content": _content, "level": level})
    # Voice output: generate TTS for this response
    if st.session_state.get("voice_output") and voice_engine and voice_engine.tts_available:
        try:
            _slow = level in ("little", "kid")
            _tts_bytes = voice_engine.speak(_content, voice=_voice_selection, slow_for_kids=_slow)
            if _tts_bytes:
                st.session_state["last_tts_audio"] = _tts_bytes
                st.session_state["last_tts_message_idx"] = len(st.session_state.messages) - 1
        except Exception:
            pass
    if progress_tracker:
        try:
            topic_name = prompt.strip()[:80].replace("\n", " ").strip() or "general"
            progress_tracker.record_topic(topic_name)
            progress_tracker.record_question(topic_name, True)
        except Exception:
            pass
    if curriculum_engine and st.session_state.get("curriculum_learn"):
        cl = st.session_state["curriculum_learn"]
        if cl.get("topic_name", "").lower() in prompt.lower() or prompt.lower().strip().startswith("teach me about"):
            curriculum_engine.mark_complete(cl["curriculum_id"], cl["topic_id"])
            st.session_state.pop("curriculum_learn", None)

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
