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
try:
    from core.quiz_generator import generate_quiz
    from core.explain_back import evaluate_explanation
except ImportError:
    generate_quiz = None
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

# ----- Voice Mode -----
_profile = load_profile()
_voice_age = _profile.get("age")
_default_voice_on = _voice_age is not None and int(_voice_age) <= 10 if isinstance(_voice_age, (int, float)) else False
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
    if st.sidebar.button("🧠 Test Yourself"):
        st.session_state["quiz_mode"] = True
        st.session_state.pop("quiz_questions", None)
        st.session_state.pop("quiz_index", None)
        st.session_state.pop("quiz_answers", None)
        st.session_state.pop("quiz_topic", None)
        st.rerun()
    if st.session_state.get("show_review"):
        st.sidebar.markdown("---")
        st.sidebar.caption("**Due for review**")
        try:
            sm2_due = progress_tracker.get_due_reviews()
            due = progress_tracker.get_due_for_review()
            if sm2_due:
                for d in sm2_due[:3]:
                    st.sidebar.markdown(f"- {d.get('name', '')} (SM-2 due)")
            if due:
                for d in due[:5]:
                    st.sidebar.markdown(f"- {d.get('name', '')} (confidence: {d.get('decayed_confidence', 0):.2f})")
            if not sm2_due and not due:
                st.sidebar.caption("Nothing due. Keep learning!")
            if st.sidebar.button("Close", key="close_review"):
                st.session_state["show_review"] = False
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

# ----- Quiz Mode -----
if st.session_state.get("quiz_mode") and progress_tracker and generate_quiz:
    st.subheader("🧠 Quiz Time!")
    if "quiz_questions" not in st.session_state or not st.session_state["quiz_questions"]:
        due = progress_tracker.get_due_reviews()
        recent = progress_tracker.get_recent_topics(5)
        topics = [d["name"] for d in due[:3]] if due else [r["topic"] for r in recent[:3]]
        if not topics:
            topics = ["general"]
        quiz_topic = topics[0]
        st.session_state["quiz_topic"] = quiz_topic
        try:
            st.session_state["quiz_questions"] = generate_quiz(quiz_topic, level, n_questions=3, api_key=os.environ.get("GEMINI_API_KEY") or st.session_state.get("api_key"))
        except Exception:
            st.session_state["quiz_questions"] = []
        st.session_state["quiz_index"] = 0
        st.session_state["quiz_answers"] = []
        if not st.session_state["quiz_questions"]:
            st.session_state["quiz_questions"] = []
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
            st.rerun()
        st.stop()
    idx = st.session_state.get("quiz_index", 0)
    quiz_topic = st.session_state.get("quiz_topic", "")
    if idx < len(qs):
        q = qs[idx]
        st.markdown(f"**Question {idx + 1} of {len(qs)}:**")
        st.markdown(q.get("question", ""))
        options = q.get("options", ["A", "B", "C", "D"])
        correct_letter = str(q.get("correct", "A")).upper()
        choice = st.radio("Choose one:", options, key="quiz_choice", format_func=lambda x: x)
        if st.button("Submit Answer", key="quiz_submit"):
            correct_idx = ord(correct_letter) - ord("A") if correct_letter in "ABCD" else 0
            correct = choice in options and options.index(choice) == correct_idx
            quality = 5 if correct else (2 if choice else 0)
            progress_tracker.schedule_review(quiz_topic, quality)
            progress_tracker.record_question(quiz_topic, correct)
            st.session_state["quiz_answers"] = st.session_state.get("quiz_answers", []) + [{"correct": correct, "explanation": q.get("explanation", ""), "topic": quiz_topic}]
            if correct:
                st.success("✅ Correct!\n\n" + (q.get("explanation") or ""))
            else:
                st.error("❌ Not quite. " + (q.get("explanation") or ""))
            st.session_state["quiz_index"] = idx + 1
            st.rerun()
    else:
        answers = st.session_state.get("quiz_answers", [])
        correct_count = sum(1 for a in answers if a.get("correct"))
        total = len(answers)
        st.markdown("**Quiz Complete!**")
        st.metric("Score", f"{correct_count}/{total} ({100 * correct_count // total if total else 0}%)")
        for a in answers:
            if a.get("correct"):
                st.success(f"✅ {a.get('topic', '')} - Got it!")
            else:
                st.warning(f"❌ {a.get('topic', '')} - Review needed")
        st.caption("These topics have been scheduled for review.")
        if st.button("Back to Learning", key="quiz_back"):
            st.session_state["quiz_mode"] = False
            st.session_state.pop("quiz_questions", None)
            st.session_state.pop("quiz_index", None)
            st.session_state.pop("quiz_answers", None)
            st.session_state.pop("quiz_topic", None)
            st.rerun()
    st.stop()

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
            st.success(f"✅ {t.get('name', t.get('id', ''))}")
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
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)
    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            engine = _engine if _engine is not None else TutorEngine(api_key=api_key or None)
            response = engine.teach(prompt, level=level)
            st.markdown(response.to_markdown())
    _content = response.to_markdown()
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
