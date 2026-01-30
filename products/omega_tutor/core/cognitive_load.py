"""
OMEGA Tutor — Cognitive load governor.
Never introduce more than ONE new cognitive demand at a time.
"""

import re
from typing import Dict, Any, List

# State strings
CONFUSED = "confused"
TESTING = "testing"
DEEP = "deep"
EXPLORING = "exploring"
OVERWHELMED = "overwhelmed"

OVERWHELM_PHRASES = [
    "i'm confused", "i am confused", "confused", "this is too much",
    "too much", "overwhelming", "don't get it", "dont get it",
    "lost", "slow down", "simplify", "what?", "huh?",
]


def _messages(session_state: Dict[str, Any]) -> List[Dict[str, Any]]:
    return (session_state.get("messages") or []) if isinstance(session_state.get("messages"), list) else []


def _last_n_user_messages(session_state: Dict[str, Any], n: int = 5) -> List[str]:
    msgs = _messages(session_state)
    user = [m.get("content", "") for m in msgs if m.get("role") == "user"]
    return user[-n:] if user else []


def _count_simpler_clicks(session_state: Dict[str, Any]) -> int:
    """Heuristic: count recent assistant messages that might follow 'simpler' (we don't store button clicks)."""
    msgs = _messages(session_state)
    count = 0
    for i, m in enumerate(msgs):
        if m.get("role") != "assistant":
            continue
        content = (m.get("content") or "").lower()
        if "simpler" in content or "adjusted to" in content and "level" in content:
            count += 1
    return min(count, 5)


def _has_explicit_overwhelm(messages: List[Dict]) -> bool:
    for m in messages:
        if m.get("role") != "user":
            continue
        text = (m.get("content") or "").strip().lower()
        for phrase in OVERWHELM_PHRASES:
            if phrase in text:
                return True
    return False


def _last_response_had_misconceptions(session_state: Dict[str, Any]) -> bool:
    """True if last explain-back or similar indicated misconceptions."""
    if session_state.get("explain_back_last_score") is not None:
        score = session_state.get("explain_back_last_score")
        if isinstance(score, (int, float)) and score < 50:
            return True
    return False


def _is_quiz_mode_active(session_state: Dict[str, Any]) -> bool:
    return bool(session_state.get("quiz_mode") or session_state.get("quiz_questions"))


def _message_count(session_state: Dict[str, Any]) -> int:
    return len(_messages(session_state))


def get_cognitive_state(session_state: Dict[str, Any]) -> str:
    """
    Returns: "confused" | "testing" | "deep" | "exploring" | "overwhelmed".
    """
    msgs = _messages(session_state)
    if not msgs:
        return EXPLORING

    # Overwhelm: explicit phrases or multiple simpler signals
    if _has_explicit_overwhelm(msgs):
        return OVERWHELMED
    simpler_count = _count_simpler_clicks(session_state)
    if simpler_count >= 2:
        return OVERWHELMED

    # Testing: in quiz or just answered
    if _is_quiz_mode_active(session_state):
        return TESTING
    if session_state.get("quiz_index") is not None and session_state.get("quiz_answers"):
        return TESTING

    # Confused: last response had misconceptions
    if _last_response_had_misconceptions(session_state):
        return CONFUSED

    # Deep: many back-and-forth (e.g. 6+ messages)
    if _message_count(session_state) >= 6:
        return DEEP

    return EXPLORING


def should_show_quiz(session_state: Dict[str, Any]) -> bool:
    """False if user is confused or just got something wrong."""
    state = get_cognitive_state(session_state)
    if state == CONFUSED:
        return False
    if state == OVERWHELMED:
        return False
    if _last_response_had_misconceptions(session_state):
        return False
    return True


def should_show_dashboard(session_state: Dict[str, Any]) -> bool:
    """False if user is in deep conversation (many back-and-forth)."""
    state = get_cognitive_state(session_state)
    if state == DEEP:
        return False
    if state == OVERWHELMED:
        return False
    if _message_count(session_state) >= 8:
        return False
    return True


def should_introduce_new_concept(session_state: Dict[str, Any]) -> bool:
    """False if user is currently testing or last response had misconceptions."""
    state = get_cognitive_state(session_state)
    if state == TESTING:
        return False
    if _last_response_had_misconceptions(session_state):
        return False
    if state == OVERWHELMED or state == CONFUSED:
        return False
    return True


def get_allowed_actions(state: str) -> List[str]:
    """
    Allowed actions per cognitive state.
    "confused" | "testing" | "deep" | "exploring" | "overwhelmed".
    """
    m = {
        CONFUSED: ["simpler", "example", "exit"],
        TESTING: ["answer", "hint", "skip", "exit"],
        DEEP: ["deeper", "related", "exit"],
        EXPLORING: ["quiz", "deeper", "new_topic", "exit"],
        OVERWHELMED: ["simpler", "break", "exit"],
    }
    return m.get(state, ["exit"])


def is_overwhelmed(session_state: Dict[str, Any]) -> bool:
    """True if current state is overwhelmed."""
    return get_cognitive_state(session_state) == OVERWHELMED
