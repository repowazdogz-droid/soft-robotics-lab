"""
OMEGA-MAX level adapter.
Adapts content delivery to age/expertise. Uses level_prompts for all level strings.
"""

from core.level_prompts import LEVEL_PROMPTS, get_level_prompt


def adapt_prompt(question: str, level: str) -> str:
    """Build the full prompt for the LLM: level instructions + question."""
    instructions = get_level_prompt(level or "adult")
    return f"{instructions.strip()}\n\n---\n\nLearner asks: {question}"
