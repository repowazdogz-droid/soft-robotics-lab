"""
OMEGA Tutor — End-of-session summaries.
Topics, confidence labels, time spent, suggested next, encouragement.
"""

from datetime import datetime
from typing import Dict, Any, List, Optional

try:
    from core.progress import progress_tracker
except ImportError:
    progress_tracker = None

try:
    from core.curriculum import curriculum_engine
except ImportError:
    curriculum_engine = None


def _confidence_label(confidence: float) -> str:
    if confidence >= 0.8:
        return "solid understanding"
    if confidence >= 0.5:
        return "good progress"
    return "might need review"


def generate_summary(session_id: Any, selected_curriculum_id: Optional[str] = None) -> Dict[str, Any]:
    """
    Generate end-of-session summary.
    session_id: int (progress session id).
    selected_curriculum_id: optional curriculum id for suggested_next.
    Returns: {topics: [{name, confidence_label}], time_minutes, suggested_next, encouragement}.
    """
    sid = int(session_id) if session_id is not None else None
    topics: List[Dict[str, str]] = []
    time_minutes = 0
    suggested_next: List[str] = []
    encouragement = "Great session! See you next time. 🌟"

    if not progress_tracker or sid is None:
        return {
            "topics": topics,
            "time_minutes": time_minutes,
            "suggested_next": suggested_next,
            "encouragement": encouragement,
        }

    started_at_iso = progress_tracker.get_session_start(sid)
    if not started_at_iso:
        return {
            "topics": topics,
            "time_minutes": time_minutes,
            "suggested_next": suggested_next,
            "encouragement": encouragement,
        }

    try:
        started = datetime.fromisoformat(started_at_iso.replace("Z", "+00:00"))
    except Exception:
        started = datetime.now()
    end = datetime.now()
    time_minutes = max(0, int((end - started).total_seconds() / 60))

    raw_topics = progress_tracker.get_topics_reviewed_since(started_at_iso)
    for t in raw_topics:
        name = (t.get("name") or "").strip() or "general"
        conf = float(t.get("confidence", 1.0))
        topics.append({"name": name, "confidence_label": _confidence_label(conf)})

    # Suggested next: from selected curriculum (get_next_topics) or weakest topics
    learned_names = {t["name"] for t in topics}
    if curriculum_engine and selected_curriculum_id:
        try:
            next_topics = curriculum_engine.get_next_topics(selected_curriculum_id, n=3)
            suggested_next = [t.get("name", t.get("id", "")) for t in next_topics if t][:3]
        except Exception:
            pass
    if not suggested_next and progress_tracker:
        weak = progress_tracker.get_weakest_topics(5)
        suggested_next = [w for w in weak if w not in learned_names][:3]
    if not suggested_next and curriculum_engine:
        try:
            cid = selected_curriculum_id
            if not cid:
                for c in curriculum_engine.list_curricula()[:1]:
                    cid = c["id"]
                    break
            if cid:
                curr = curriculum_engine.load_curriculum(cid)
                if curr:
                    all_names = [t.get("name", "") for t in curr.get("topics", []) if t.get("name")]
                    suggested_next = [n for n in all_names if n and n not in learned_names][:3]
        except Exception:
            pass

    # Encouragement varies by performance
    solid = sum(1 for t in topics if t["confidence_label"] == "solid understanding")
    need_review = sum(1 for t in topics if t["confidence_label"] == "might need review")
    if topics:
        if solid == len(topics):
            encouragement = "Excellent focus today. You really got it. 🌟"
        elif need_review == 0:
            encouragement = "Great session! Learning takes time, and you're making progress. 🌟"
        elif need_review < len(topics):
            encouragement = "Good session. A bit of review next time will lock it in. 🌟"
        else:
            encouragement = "You showed up and tried—that's what matters. Next time we can go slower. 🌟"
    else:
        encouragement = "Thanks for being here. Whenever you're ready, we can dive in. 🌟"

    return {
        "topics": topics,
        "time_minutes": time_minutes,
        "suggested_next": suggested_next,
        "encouragement": encouragement,
    }
