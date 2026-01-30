"""
OMEGA-MAX tutor engine.
LLM teaching: teach, follow_up, deeper. Returns TeachingResponse.
LM Studio (local) primary; Gemini fallback.
"""

import os
import sys
from pathlib import Path
from typing import Optional, List
from dataclasses import dataclass, field

from openai import OpenAI

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.level_adapter import adapt_prompt
from core.level_prompts import get_level_prompt
from core.exercise_generator import ExerciseGenerator, Exercise

try:
    from prompts.tutor_system import TUTOR_SYSTEM_PROMPT
except ImportError:
    TUTOR_SYSTEM_PROMPT = "You are a helpful tutor. Explain clearly. Use examples."


def _exercise_level(level: str) -> Optional[str]:
    """Map profile level to exercise-generator level (child, teen, adult, expert, researcher)."""
    key = (level or "adult").lower().strip()
    if key in ("little", "kid"):
        return "child"
    if key in ("teen", "sixth_form"):
        return "teen"
    if key in ("undergrad", "adult", "senior"):
        return "adult"
    if key in ("expert", "researcher"):
        return key
    return "adult"


@dataclass
class TeachingResponse:
    """Structured teaching response."""
    explanation: str
    example: str
    exercise: Optional[str] = None
    follow_ups: List[str] = field(default_factory=list)
    sources: List[str] = field(default_factory=list)

    def to_markdown(self) -> str:
        md = f"## Explanation\n\n{self.explanation}\n\n## Example\n\n{self.example}\n\n"
        if self.exercise:
            md += f"## Try This\n\n{self.exercise}\n\n"
        if self.follow_ups:
            md += "## Go Deeper\n\n"
            for q in self.follow_ups:
                md += f"- {q}\n"
            md += "\n"
        if self.sources:
            md += "## Sources\n\n"
            for s in self.sources:
                md += f"- {s}\n"
        return md


class TutorEngine:
    """LLM teaching engine. LM Studio (local) primary; Gemini fallback."""

    def __init__(self, api_key: str = None):
        key = api_key or os.environ.get("GEMINI_API_KEY")
        self.use_local = False
        self._local_model_id = "local-model"
        try:
            self.local_client = OpenAI(
                base_url="http://localhost:1234/v1",
                api_key="lm-studio",
            )
            models = self.local_client.models.list()
            if models.data:
                self._local_model_id = models.data[0].id
            self.use_local = True
        except Exception:
            self.use_local = False

        if not self.use_local:
            if not key:
                raise ValueError("No LM Studio and no GEMINI_API_KEY")
            import google.generativeai as genai
            genai.configure(api_key=key)
            self.gemini_model = genai.GenerativeModel("gemini-2.0-flash")

        self.exercise_gen = ExerciseGenerator(api_key=key)

    def _generate(self, prompt: str) -> str:
        """Single entry point: local (LM Studio) or Gemini."""
        if self.use_local:
            response = self.local_client.chat.completions.create(
                model=self._local_model_id,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.7,
            )
            return (response.choices[0].message.content or "").strip()
        else:
            response = self.gemini_model.generate_content(prompt)
            return (response.text or "").strip()

    def teach(self, question: str, level: str = "adult") -> TeachingResponse:
        """
        Returns structured response:
        - explanation, example, exercise (optional), follow_ups, sources (expert/researcher)
        """
        user_prompt = adapt_prompt(question, level)
        full_system = f"{TUTOR_SYSTEM_PROMPT}\n\nCurrent level instruction: {get_level_prompt(level)}"
        raw = self._call_llm(full_system, user_prompt)
        parsed = self._parse_response(raw, question, level)
        _ex_level = _exercise_level(level)
        if _ex_level and len(question.split()) > 3:
            ex = self.exercise_gen.generate(question[:200], _ex_level)
            if ex.prompt and "Configure" not in ex.prompt:
                parsed.exercise = ex.prompt
                if ex.hint:
                    parsed.exercise += f"\n\n*Hint: {ex.hint}*"
        return parsed

    def follow_up(self, original_question: str, follow_up: str, level: str) -> TeachingResponse:
        """Handles 'wait, what's X?' type questions in context of original."""
        context = f"Previously the learner asked: {original_question}\n\nNow they ask a follow-up: {follow_up}\n\nAnswer the follow-up in context. Keep it concise but complete."
        user_prompt = adapt_prompt(context, level)
        full_system = f"{TUTOR_SYSTEM_PROMPT}\n\nCurrent level instruction: {get_level_prompt(level)}"
        raw = self._call_llm(full_system, user_prompt)
        return self._parse_response(raw, follow_up, level)

    def deeper(self, topic: str, current_level: str) -> TeachingResponse:
        """Goes one level deeper on the same topic."""
        question = f"Go deeper on: {topic}. Assume I already understand the basics. Add nuance, edge cases, or next-level connections."
        return self.teach(question, level=current_level)

    def _call_llm(self, system: str, user: str) -> str:
        """Build prompt and call _generate(). Return raw text or error sentinel."""
        prompt = f"{system}\n\n---\n\n{user}"
        try:
            return self._generate(prompt)
        except Exception as e:
            err = str(e).strip()[:200]
            return f"[OMEGA_ERROR]{err}"

    def _parse_response(self, raw: str, question: str, level: str) -> TeachingResponse:
        """Parse raw LLM output into TeachingResponse."""
        if not raw:
            return TeachingResponse(
                explanation="I couldn't generate a response. Check: (1) LM Studio running at http://localhost:1234 with a model loaded, or (2) GEMINI_API_KEY set for cloud fallback.",
                example="",
            )
        if raw.startswith("[OMEGA_ERROR]"):
            err = raw[13:].strip()
            return TeachingResponse(
                explanation=f"I couldn't generate a response. Error: {err}. Check: (1) LM Studio running at http://localhost:1234 with a model loaded, or (2) GEMINI_API_KEY set for cloud fallback.",
                example="",
            )
        explanation = raw
        example = ""
        exercise = None
        follow_ups = []
        sources = []
        parts = raw.split("## ")
        for p in parts:
            p = p.strip()
            if not p:
                continue
            lines = p.split("\n")
            title = lines[0].strip().lower()
            body = "\n".join(lines[1:]).strip() if len(lines) > 1 else ""
            if "explanation" in title:
                explanation = body or p
            elif "example" in title:
                example = body or p
            elif "try this" in title or "exercise" in title:
                exercise = body or p
            elif "go deeper" in title or "follow" in title:
                for line in body.split("\n"):
                    line = line.strip().lstrip("-* ")
                    if line:
                        follow_ups.append(line)
            elif "source" in title:
                for line in body.split("\n"):
                    line = line.strip().lstrip("-* ")
                    if line:
                        sources.append(line)
        if not example and "\n\n" in explanation:
            first, rest = explanation.split("\n\n", 1)
            explanation = first
            example = rest.split("## ")[0].strip() if "## " in rest else rest.strip()
        return TeachingResponse(
            explanation=explanation or raw[:2000],
            example=example,
            exercise=exercise,
            follow_ups=follow_ups[:5],
            sources=sources[:5],
        )
