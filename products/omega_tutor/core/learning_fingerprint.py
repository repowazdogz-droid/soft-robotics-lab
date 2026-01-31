"""
Learning Fingerprint - Track how each person learns

What works for them:
- Which analogies landed
- Which explanations confused
- Time-to-understand per concept
- What "clicked" after what

Builds a unique learning profile over time.
"""
from dataclasses import dataclass, field, fields
from typing import List, Dict, Optional, Any
from datetime import datetime
from pathlib import Path
import json

# Data storage
_DATA_DIR = Path(__file__).resolve().parent.parent / "data"
_FINGERPRINT_FILE = _DATA_DIR / "learning_fingerprint.json"


@dataclass
class LearningEvent:
    timestamp: str
    topic: str
    level: str
    event_type: str  # "explanation", "analogy", "quiz", "explain_back"
    content: str  # The explanation/analogy used
    outcome: str  # "understood", "confused", "partial", "correct", "incorrect"
    time_spent_seconds: Optional[float] = None
    follow_up_needed: bool = False
    notes: str = ""


@dataclass
class TopicProfile:
    topic: str
    times_studied: int = 0
    best_level: str = "beginner"
    successful_analogies: List[str] = field(default_factory=list)
    failed_analogies: List[str] = field(default_factory=list)
    avg_time_to_understand: float = 0.0
    common_confusions: List[str] = field(default_factory=list)
    mastery_score: float = 0.0
    last_studied: str = ""


@dataclass
class LearningFingerprint:
    user_id: str = "default"
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = field(default_factory=lambda: datetime.now().isoformat())

    # Learning style indicators
    prefers_analogies: bool = True
    prefers_examples: bool = True
    prefers_formal_definitions: bool = False
    prefers_visual: bool = False
    optimal_explanation_length: str = "medium"  # short, medium, long

    # Pace
    avg_time_per_topic_seconds: float = 300.0
    needs_repetition: bool = False

    # History
    events: List[Dict] = field(default_factory=list)
    topic_profiles: Dict[str, Dict] = field(default_factory=dict)

    # Patterns
    best_performing_topics: List[str] = field(default_factory=list)
    struggling_topics: List[str] = field(default_factory=list)
    analogy_effectiveness: Dict[str, float] = field(default_factory=dict)  # analogy -> success rate


def load_fingerprint(user_id: str = "default") -> LearningFingerprint:
    """Load or create learning fingerprint."""
    _DATA_DIR.mkdir(parents=True, exist_ok=True)

    if _FINGERPRINT_FILE.exists():
        try:
            data = json.loads(_FINGERPRINT_FILE.read_text(encoding="utf-8"))
            if data.get("user_id") == user_id:
                valid_keys = {f.name for f in fields(LearningFingerprint)}
                fp = LearningFingerprint(**{k: data[k] for k in valid_keys if k in data})
                return fp
        except Exception:
            pass

    return LearningFingerprint(user_id=user_id)


def save_fingerprint(fp: LearningFingerprint) -> None:
    """Save learning fingerprint."""
    _DATA_DIR.mkdir(parents=True, exist_ok=True)
    fp.updated_at = datetime.now().isoformat()

    data = {
        "user_id": fp.user_id,
        "created_at": fp.created_at,
        "updated_at": fp.updated_at,
        "prefers_analogies": fp.prefers_analogies,
        "prefers_examples": fp.prefers_examples,
        "prefers_formal_definitions": fp.prefers_formal_definitions,
        "prefers_visual": fp.prefers_visual,
        "optimal_explanation_length": fp.optimal_explanation_length,
        "avg_time_per_topic_seconds": fp.avg_time_per_topic_seconds,
        "needs_repetition": fp.needs_repetition,
        "events": fp.events[-1000:],  # Keep last 1000 events
        "topic_profiles": fp.topic_profiles,
        "best_performing_topics": fp.best_performing_topics,
        "struggling_topics": fp.struggling_topics,
        "analogy_effectiveness": fp.analogy_effectiveness,
    }

    _FINGERPRINT_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")


def update_topic_profile(
    fp: LearningFingerprint,
    topic: str,
    level: str,
    event_type: str,
    outcome: str,
    content: str,
    time_spent: Optional[float],
) -> None:
    """Update the profile for a specific topic."""
    if topic not in fp.topic_profiles:
        fp.topic_profiles[topic] = {
            "topic": topic,
            "times_studied": 0,
            "best_level": level,
            "successful_analogies": [],
            "failed_analogies": [],
            "avg_time_to_understand": 0.0,
            "common_confusions": [],
            "mastery_score": 0.0,
            "last_studied": "",
        }

    profile = fp.topic_profiles[topic]
    profile["times_studied"] += 1
    profile["last_studied"] = datetime.now().isoformat()

    # Update level if progressed
    level_order = ["kid", "beginner", "student", "adult", "expert", "researcher"]
    if level in level_order:
        current_idx = level_order.index(profile["best_level"]) if profile["best_level"] in level_order else 0
        new_idx = level_order.index(level)
        if new_idx > current_idx and outcome in ["understood", "correct"]:
            profile["best_level"] = level

    # Track analogies
    if event_type == "analogy" or "like" in (content or "").lower() or "similar to" in (content or "").lower():
        analogy_key = (content or "")[:100]
        if outcome in ["understood", "correct"]:
            if analogy_key not in profile["successful_analogies"]:
                profile["successful_analogies"].append(analogy_key)
                profile["successful_analogies"] = profile["successful_analogies"][-10:]
        elif outcome in ["confused", "incorrect"]:
            if analogy_key not in profile["failed_analogies"]:
                profile["failed_analogies"].append(analogy_key)
                profile["failed_analogies"] = profile["failed_analogies"][-10:]

    # Track confusions
    if outcome == "confused" and content:
        confusion = content[:100]
        if confusion not in profile["common_confusions"]:
            profile["common_confusions"].append(confusion)
            profile["common_confusions"] = profile["common_confusions"][-5:]

    # Update time tracking
    if time_spent is not None:
        old_avg = profile["avg_time_to_understand"]
        n = profile["times_studied"]
        profile["avg_time_to_understand"] = ((old_avg * (n - 1)) + time_spent) / n

    # Update mastery score
    update_mastery_score(profile)


def update_mastery_score(profile: Dict) -> None:
    """Calculate mastery score for a topic."""
    score = 0.0

    # Base score from times studied
    score += min(profile["times_studied"] * 0.1, 0.3)

    # Level bonus
    level_bonus = {
        "kid": 0.0,
        "beginner": 0.1,
        "student": 0.2,
        "adult": 0.3,
        "expert": 0.4,
        "researcher": 0.5,
    }
    score += level_bonus.get(profile["best_level"], 0.1)

    # Successful analogies bonus
    score += min(len(profile.get("successful_analogies", [])) * 0.05, 0.2)

    # Confusion penalty
    score -= min(len(profile.get("common_confusions", [])) * 0.05, 0.2)

    profile["mastery_score"] = max(0.0, min(1.0, score))


def update_preferences(fp: LearningFingerprint) -> None:
    """Update learning style preferences based on history."""
    if len(fp.events) < 5:
        return

    recent = fp.events[-50:]

    analogy_success = 0
    analogy_total = 0
    example_success = 0
    example_total = 0
    formal_success = 0
    formal_total = 0

    for event in recent:
        content_lower = (event.get("content") or "").lower()
        outcome = event.get("outcome", "")

        is_success = outcome in ["understood", "correct"]

        if "like" in content_lower or "similar to" in content_lower or "imagine" in content_lower:
            analogy_total += 1
            if is_success:
                analogy_success += 1

        if "for example" in content_lower or "such as" in content_lower or "e.g." in content_lower:
            example_total += 1
            if is_success:
                example_success += 1

        if "definition" in content_lower or "formally" in content_lower or "mathematically" in content_lower:
            formal_total += 1
            if is_success:
                formal_success += 1

    if analogy_total >= 3:
        fp.prefers_analogies = (analogy_success / analogy_total) > 0.6
        fp.analogy_effectiveness["overall"] = analogy_success / analogy_total

    if example_total >= 3:
        fp.prefers_examples = (example_success / example_total) > 0.6

    if formal_total >= 3:
        fp.prefers_formal_definitions = (formal_success / formal_total) > 0.6

    topic_scores = [(t, p.get("mastery_score", 0)) for t, p in fp.topic_profiles.items()]
    topic_scores.sort(key=lambda x: x[1], reverse=True)

    fp.best_performing_topics = [t for t, s in topic_scores[:5] if s > 0.5]
    fp.struggling_topics = [t for t, s in topic_scores[-5:] if s < 0.4]


def record_learning_event(
    topic: str,
    level: str,
    event_type: str,
    content: str,
    outcome: str,
    time_spent_seconds: Optional[float] = None,
    follow_up_needed: bool = False,
    notes: str = "",
    user_id: str = "default",
) -> None:
    """Record a learning event and update fingerprint."""
    fp = load_fingerprint(user_id=user_id)

    event = {
        "timestamp": datetime.now().isoformat(),
        "topic": topic,
        "level": level,
        "event_type": event_type,
        "content": (content or "")[:500],
        "outcome": outcome,
        "time_spent_seconds": time_spent_seconds,
        "follow_up_needed": follow_up_needed,
        "notes": notes,
    }

    fp.events.append(event)

    update_topic_profile(fp, topic, level, event_type, outcome, content or "", time_spent_seconds)

    update_preferences(fp)

    save_fingerprint(fp)


def get_learning_recommendations(topic: str, user_id: str = "default") -> Dict[str, Any]:
    """Get recommendations for teaching a topic based on fingerprint."""
    fp = load_fingerprint(user_id=user_id)

    recommendations = {
        "use_analogies": fp.prefers_analogies,
        "use_examples": fp.prefers_examples,
        "use_formal_definitions": fp.prefers_formal_definitions,
        "explanation_length": fp.optimal_explanation_length,
        "needs_repetition": fp.needs_repetition,
        "related_strengths": [],
        "avoid_confusions": [],
        "successful_approaches": [],
    }

    if topic in fp.topic_profiles:
        profile = fp.topic_profiles[topic]
        recommendations["successful_approaches"] = profile.get("successful_analogies", [])[:3]
        recommendations["avoid_confusions"] = profile.get("common_confusions", [])[:3]
        recommendations["mastery_score"] = profile.get("mastery_score", 0.0)

    for t, p in fp.topic_profiles.items():
        if t != topic and p.get("mastery_score", 0) > 0.6:
            recommendations["related_strengths"].append(t)

    recommendations["related_strengths"] = recommendations["related_strengths"][:3]

    return recommendations


def get_fingerprint_summary(user_id: str = "default") -> Dict[str, Any]:
    """Get a summary of the learning fingerprint."""
    fp = load_fingerprint(user_id=user_id)

    return {
        "total_events": len(fp.events),
        "topics_studied": len(fp.topic_profiles),
        "learning_style": {
            "prefers_analogies": fp.prefers_analogies,
            "prefers_examples": fp.prefers_examples,
            "prefers_formal": fp.prefers_formal_definitions,
            "optimal_length": fp.optimal_explanation_length,
        },
        "best_topics": fp.best_performing_topics[:5],
        "struggling_topics": fp.struggling_topics[:5],
        "avg_time_per_topic": fp.avg_time_per_topic_seconds,
        "needs_repetition": fp.needs_repetition,
    }
