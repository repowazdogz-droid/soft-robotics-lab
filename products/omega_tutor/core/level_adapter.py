"""
OMEGA-MAX level adapter.
Adapts content delivery to age/expertise: child, teen, adult, expert, researcher.
"""

from typing import Dict

LEVEL_PROMPTS: Dict[str, str] = {
    "child": """
Explain like talking to a curious 8-year-old.
- Use simple words (no jargon)
- Short sentences
- Concrete examples they can see/touch
- Make it fun and wonder-filled
- Use analogies to toys, animals, food, games
""",
    "teen": """
Explain for a smart 15-year-old.
- Introduce technical terms but define them
- Connect to things they care about (games, social media, sports, music)
- Be encouraging, not condescending
- Show why this matters in the real world
""",
    "adult": """
Explain for a curious adult learner.
- Full vocabulary, define specialized terms once
- Practical, applicable knowledge
- Respect their time — be clear and efficient
- Include "how to actually use this"
""",
    "expert": """
Explain for someone with domain background.
- Assume foundational knowledge
- Focus on nuance, edge cases, common misconceptions
- Reference key papers/frameworks
- Include current debates and open questions
""",
    "researcher": """
OMEGA-MAX depth. Explain for active researcher.
- Full technical depth
- Connect to substrates: materials, compute, bio, manufacturing, coordination, environment
- Frame in T1-T4 temporal horizons
- Identify open problems and research frontiers
- Surface assumptions and uncertainties
- Multiple competing frameworks/theories
""",
}


def adapt_prompt(question: str, level: str) -> str:
    """Build the full prompt for the LLM: level instructions + question."""
    level = (level or "adult").lower().strip()
    instructions = LEVEL_PROMPTS.get(level, LEVEL_PROMPTS["adult"])
    return f"{instructions.strip()}\n\n---\n\nLearner asks: {question}"
