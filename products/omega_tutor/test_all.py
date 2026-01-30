"""
OMEGA Tutor - Comprehensive Functionality Test
Run this to check all components work before using the app.
"""

import sys
import os
from pathlib import Path

# Setup path
sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))


def test_section(name):
    print(f"\n{'='*50}")
    print(f"TESTING: {name}")
    print("=" * 50)


def test_pass(msg):
    print(f"  ✅ {msg}")


def test_fail(msg, error=None):
    print(f"  ❌ {msg}")
    if error:
        print(f"     Error: {error}")


def test_warn(msg):
    print(f"  ⚠️  {msg}")


# ============================================================
# 1. IMPORTS
# ============================================================
test_section("Core Imports")

try:
    from core.user_profile import (
        load_profile,
        save_profile,
        has_profile,
        get_current_level,
        PERSONA_CONFIG,
    )
    test_pass("user_profile")
except Exception as e:
    test_fail("user_profile", e)

try:
    from core.level_prompts import get_level_prompt, LEVEL_PROMPTS
    test_pass("level_prompts")
except Exception as e:
    test_fail("level_prompts", e)

try:
    from core.level_adjust import adjust_level, LEVEL_ORDER
    test_pass("level_adjust")
except Exception as e:
    test_fail("level_adjust", e)

try:
    from core.tutor_engine import TutorEngine
    test_pass("tutor_engine")
except Exception as e:
    test_fail("tutor_engine", e)

try:
    from core.progress import ProgressTracker, progress_tracker
    test_pass("progress")
except Exception as e:
    test_fail("progress", e)

try:
    from core.memory import record_learning, get_related_learning, has_learned
    test_pass("memory")
except Exception as e:
    test_fail("memory", e)

try:
    from core.curriculum import CurriculumEngine, curriculum_engine
    test_pass("curriculum")
except Exception as e:
    test_fail("curriculum", e)

try:
    from core.curriculum_progress import (
        is_topic_complete,
        mark_topic_complete,
        get_curriculum_stats,
    )
    test_pass("curriculum_progress")
except Exception as e:
    test_fail("curriculum_progress", e)

try:
    from core.knowledge_decay import calculate_confidence, get_review_priority
    test_pass("knowledge_decay")
except Exception as e:
    test_fail("knowledge_decay", e)

try:
    from core.quiz_generator import generate_quiz, generate_open_question
    test_pass("quiz_generator")
except Exception as e:
    test_fail("quiz_generator", e)

try:
    from core.spaced_repetition import SM2
    test_pass("spaced_repetition (SM2)")
except Exception as e:
    test_fail("spaced_repetition", e)

try:
    from core.explain_back import evaluate_explanation
    test_pass("explain_back")
except Exception as e:
    test_fail("explain_back", e)

try:
    from core.session_summary import generate_summary
    test_pass("session_summary")
except Exception as e:
    test_fail("session_summary", e)

try:
    from core.cognitive_load import (
        get_cognitive_state,
        should_show_quiz,
        is_overwhelmed,
    )
    test_pass("cognitive_load")
except Exception as e:
    test_fail("cognitive_load", e)

try:
    from core.voice import VoiceEngine, voice_engine
    if voice_engine.whisper_available:
        test_pass("voice (Whisper available)")
    else:
        test_warn("voice (Whisper NOT available - voice input disabled)")
    if voice_engine.tts_available:
        test_pass("voice (TTS available)")
    else:
        test_warn("voice (TTS NOT available - voice output disabled)")
except Exception as e:
    test_warn(f"voice (optional): {e}")

# ============================================================
# 2. SUBSTRATE (Optional)
# ============================================================
test_section("Substrate (Optional)")

try:
    from shared import vector_store, knowledge_graph, lineage_graph
    if vector_store:
        test_pass("vector_store connected")
    else:
        test_warn("vector_store not available")
    if knowledge_graph:
        test_pass("knowledge_graph connected")
    else:
        test_warn("knowledge_graph not available")
    if lineage_graph:
        test_pass("lineage_graph connected")
    else:
        test_warn("lineage_graph not available")
except Exception as e:
    test_warn(f"substrate not available: {e}")

# ============================================================
# 3. DATABASE OPERATIONS
# ============================================================
test_section("Database Operations")

session_id = None
try:
    tracker = ProgressTracker()
    session_id = tracker.start_session()
    test_pass(f"start_session() returned {session_id}")
except Exception as e:
    test_fail("start_session()", e)

try:
    tracker = ProgressTracker()
    tracker.record_topic("test_topic")
    test_pass("record_topic()")
except Exception as e:
    test_fail("record_topic()", e)

try:
    tracker = ProgressTracker()
    tracker.record_question("test_topic", correct=True)
    test_pass("record_question()")
except Exception as e:
    test_fail("record_question()", e)

try:
    tracker = ProgressTracker()
    stats = tracker.get_weekly_stats()
    test_pass(f"get_weekly_stats(): {stats}")
except Exception as e:
    test_fail("get_weekly_stats()", e)

try:
    tracker = ProgressTracker()
    streak = tracker.get_streak()
    test_pass(f"get_streak(): {streak}")
except Exception as e:
    test_fail("get_streak()", e)

if session_id is not None:
    try:
        tracker = ProgressTracker()
        tracker.end_session(session_id)
        test_pass("end_session()")
    except Exception as e:
        test_fail("end_session()", e)

# ============================================================
# 4. CURRICULUM
# ============================================================
test_section("Curriculum")

curricula = []
try:
    curricula = curriculum_engine.list_curricula()
    test_pass(f"list_curricula(): {len(curricula)} found")
    for c in curricula[:3]:
        print(f"       - {c.get('id', c.get('name', 'unknown'))}")
except Exception as e:
    test_fail("list_curricula()", e)

try:
    if curricula:
        cid = curricula[0].get("id", curricula[0].get("name"))
        progress = curriculum_engine.get_progress(cid)
        test_pass(
            f"get_progress('{cid}'): {progress.get('percent_complete', 0)}% complete"
        )
    else:
        test_warn("get_progress() skipped (no curricula)")
except Exception as e:
    test_fail("get_progress()", e)

# ============================================================
# 5. LEVEL SYSTEM
# ============================================================
test_section("Level System")

try:
    for level in ["kid", "undergrad", "adult", "expert", "researcher"]:
        prompt = get_level_prompt(level)
        if prompt and len(prompt) > 20:
            test_pass(f"get_level_prompt('{level}'): {len(prompt)} chars")
        else:
            test_fail(f"get_level_prompt('{level}'): empty or too short")
except Exception as e:
    test_fail("get_level_prompt()", e)

try:
    result = adjust_level("adult", "simpler")
    test_pass(f"adjust_level('adult', 'simpler'): {result}")
    result = adjust_level("adult", "deeper")
    test_pass(f"adjust_level('adult', 'deeper'): {result}")
except Exception as e:
    test_fail("adjust_level()", e)

# ============================================================
# 6. SM-2 SPACED REPETITION
# ============================================================
test_section("Spaced Repetition (SM-2)")

try:
    result = SM2.update_card(
        correct=True, quality=4, repetitions=0, easiness=2.5, previous_interval=1
    )
    test_pass(
        f"SM2.update_card(correct): interval={result.get('interval')}, next={result.get('next_review_date')}"
    )
except Exception as e:
    test_fail("SM2.update_card()", e)

try:
    result = SM2.update_card(
        correct=False, quality=1, repetitions=3, easiness=2.5, previous_interval=10
    )
    test_pass(f"SM2.update_card(incorrect): interval={result.get('interval')} (should reset)")
except Exception as e:
    test_fail("SM2.update_card(incorrect)", e)

# ============================================================
# 7. KNOWLEDGE DECAY
# ============================================================
test_section("Knowledge Decay")

try:
    conf = calculate_confidence(1.0, days_since=7)
    test_pass(f"calculate_confidence(1.0, 7 days): {conf:.2f}")
except Exception as e:
    test_fail("calculate_confidence()", e)

# ============================================================
# 8. LLM CONNECTION
# ============================================================
test_section("LLM Connection")

# Check API keys
google_key = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
openai_key = os.environ.get("OPENAI_API_KEY")
anthropic_key = os.environ.get("ANTHROPIC_API_KEY")

if google_key:
    test_pass("GOOGLE_API_KEY / GEMINI_API_KEY is set")
else:
    test_warn("GOOGLE_API_KEY / GEMINI_API_KEY not set")

if openai_key:
    test_pass("OPENAI_API_KEY is set")
else:
    test_warn("OPENAI_API_KEY not set")

if anthropic_key:
    test_pass("ANTHROPIC_API_KEY is set")
else:
    test_warn("ANTHROPIC_API_KEY not set")

# Check LM Studio
try:
    import requests
    r = requests.get("http://localhost:1234/v1/models", timeout=2)
    if r.status_code == 200:
        test_pass("LM Studio running at localhost:1234")
    else:
        test_warn("LM Studio not responding")
except Exception:
    test_warn("LM Studio not running (localhost:1234)")

# Quick LLM test (with timeout)
print("\n  Testing LLM call (5 second timeout)...")
try:
    import threading
    result = [None]
    error = [None]

    def call_llm():
        try:
            engine = TutorEngine()
            result[0] = engine.teach("What is 2+2?", "adult")
        except Exception as e:
            error[0] = e

    thread = threading.Thread(target=call_llm)
    thread.start()
    thread.join(timeout=5)

    if thread.is_alive():
        test_warn("LLM call timed out (no LLM available or slow)")
    elif error[0]:
        test_fail("LLM call failed", error[0])
    elif result[0] is not None:
        # TeachingResponse has .explanation attribute
        resp = result[0]
        expl = (resp.explanation if hasattr(resp, "explanation") else str(resp)) or ""
        test_pass(f"LLM responded: {(expl[:50] + '...') if len(expl) > 50 else expl}")
    else:
        test_warn("LLM returned empty response")
except Exception as e:
    test_fail("LLM test", e)

# ============================================================
# 9. USER PROFILE
# ============================================================
test_section("User Profile")

try:
    if has_profile():
        profile = load_profile()
        test_pass(f"Profile exists: {profile}")
    else:
        test_pass("No profile yet (will be created on first use)")
except Exception as e:
    test_fail("load_profile()", e)

# ============================================================
# SUMMARY
# ============================================================
print("\n" + "=" * 50)
print("TEST COMPLETE")
print("=" * 50)
print(
    """
Key findings:
- ✅ = Working
- ⚠️  = Optional/Warning (may work without)
- ❌ = Broken (needs fix)

If LLM tests failed/timed out:
1. Start LM Studio and load a model, OR
2. Set GEMINI_API_KEY or GOOGLE_API_KEY environment variable

To run Tutor:
  cd products/omega_tutor
  python -m streamlit run app.py --server.port 8503
"""
)
