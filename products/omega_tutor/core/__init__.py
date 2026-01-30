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
