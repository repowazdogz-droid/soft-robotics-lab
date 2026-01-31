"""
OMEGA-MAX Omega Tutor — low-friction learning.
Zero friction: ask, get taught. Levels: child, teen, adult, expert, researcher.
"""

from .tutor_engine import TutorEngine, TeachingResponse
from .level_adapter import adapt_prompt, LEVEL_PROMPTS
from .exercise_generator import ExerciseGenerator, Exercise
from .knowledge_base import KnowledgeBase

__all__ = [
    "TutorEngine",
    "TeachingResponse",
    "adapt_prompt",
    "LEVEL_PROMPTS",
    "ExerciseGenerator",
    "Exercise",
    "KnowledgeBase",
]

try:
    from .learning_fingerprint import (
        LearningFingerprint,
        LearningEvent,
        TopicProfile,
        load_fingerprint,
        save_fingerprint,
        record_learning_event,
        get_learning_recommendations,
        get_fingerprint_summary,
    )
    __all__ += [
        "LearningFingerprint",
        "LearningEvent",
        "TopicProfile",
        "load_fingerprint",
        "save_fingerprint",
        "record_learning_event",
        "get_learning_recommendations",
        "get_fingerprint_summary",
    ]
except ImportError:
    pass

try:
    from .misconception_detector import (
        Misconception,
        MISCONCEPTIONS,
        detect_misconceptions,
        check_explanation_for_misconceptions,
        generate_preemptive_warning,
        get_topic_misconceptions,
        generate_correction,
        add_misconception,
    )
    __all__ += [
        "Misconception",
        "MISCONCEPTIONS",
        "detect_misconceptions",
        "check_explanation_for_misconceptions",
        "generate_preemptive_warning",
        "get_topic_misconceptions",
        "generate_correction",
        "add_misconception",
    ]
except ImportError:
    pass
