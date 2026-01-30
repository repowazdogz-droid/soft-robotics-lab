"""
OMEGA-MAX exercise generator.
Generates level-appropriate practice: child (draw/explore) → researcher (design experiment).
"""

import os
import sys
from pathlib import Path
from typing import Optional, Any
from dataclasses import dataclass

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


@dataclass
class Exercise:
    """A practice exercise for the learner."""
    prompt: str
    level: str
    hint: Optional[str] = None
    kind: str = "open"  # open, multiple_choice, design, draw


class ExerciseGenerator:
    """Generates appropriate exercises by level."""

    def __init__(self, api_key: Optional[str] = None):
        self.api_key = api_key or os.environ.get("GEMINI_API_KEY") or os.environ.get("ANTHROPIC_API_KEY")

    def generate(self, topic: str, level: str) -> Exercise:
        """
        Generates appropriate exercise:
        - child: "Can you draw..." / "What would happen if..."
        - teen: Multiple choice or short scenario
        - adult: Apply-it-yourself challenge
        - expert: Edge case problem
        - researcher: "Design an experiment to test..."
        """
        level = (level or "adult").lower().strip()
        if not self.api_key:
            return Exercise(
                prompt=f"Think about: {topic}. Try explaining it in your own words or with a simple example.",
                level=level,
                kind="open",
            )
        try:
            return self._call_llm(topic, level)
        except Exception:
            return Exercise(
                prompt=f"Reflect on: {topic}. What's one thing you could try or explain to someone else?",
                level=level,
                kind="open",
            )

    def _call_llm(self, topic: str, level: str) -> Exercise:
        """Use LLM to generate level-appropriate exercise."""
        prompt = f"""Generate ONE short practice exercise for someone learning about: {topic}.
Level: {level}.

Format your response as exactly two lines:
Line 1: EXERCISE: [the exercise prompt - one sentence or short paragraph]
Line 2: HINT: [optional hint, or "None"]

Level guidelines:
- child: "Can you draw..." or "What would happen if..." — fun, concrete
- teen: A short scenario or "Which of these..." multiple choice style
- adult: "Try this: ..." apply-it-yourself challenge
- expert: Edge case or "What happens when..."
- researcher: "Design an experiment to test..." or "Formulate a hypothesis..."
"""
        # Try Gemini first
        try:
            api_key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
            if api_key:
                import google.generativeai as genai
                genai.configure(api_key=api_key)
                model = genai.GenerativeModel("gemini-1.5-flash")
                r = model.generate_content(prompt)
                text = (r.text or "").strip()
                return self._parse_exercise_response(text, topic, level)
        except Exception:
            pass
        # Try Anthropic
        try:
            api_key = os.environ.get("ANTHROPIC_API_KEY")
            if api_key:
                import anthropic
                client = anthropic.Anthropic(api_key=api_key)
                msg = client.messages.create(
                    model="claude-3-5-haiku-20241022",
                    max_tokens=256,
                    messages=[{"role": "user", "content": prompt}],
                )
                text = (msg.content[0].text if msg.content else "").strip()
                return self._parse_exercise_response(text, topic, level)
        except Exception:
            pass
        return Exercise(
            prompt=f"Reflect on: {topic}. Try explaining it in your own words.",
            level=level,
            kind="open",
        )

    def _parse_exercise_response(self, text: str, topic: str, level: str) -> Exercise:
        """Parse EXERCISE: and HINT: from LLM response."""
        prompt_line = topic
        hint = None
        for line in text.split("\n"):
            line = line.strip()
            if line.upper().startswith("EXERCISE:"):
                prompt_line = line.split(":", 1)[-1].strip()
            elif line.upper().startswith("HINT:"):
                h = line.split(":", 1)[-1].strip()
                if h and h.lower() != "none":
                    hint = h
        return Exercise(prompt=prompt_line, level=level, hint=hint, kind="open")
