"""
Minimal TutorEngine - no memory, no substrate, just works.
No blocking models.list() in __init__; LM Studio tested on first teach().
"""
from openai import OpenAI
from typing import Optional, List
from dataclasses import dataclass, field
import os

@dataclass
class TeachingResponse:
    explanation: str
    example: str = ""
    topic: str = ""
    exercise: str = ""
    follow_ups: List[str] = field(default_factory=list)
    sources: List[str] = field(default_factory=list)

    def get(self, key, default=None):
        return getattr(self, key, default)

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
    def __init__(self, api_key: Optional[str] = None):
        self._local_model_id = "phi-3-mini-4k-instruct"
        self._api_key = api_key or os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")

        # Create LM Studio client (don't test it here, test on first use)
        self._local_client = OpenAI(
            base_url="http://localhost:1234/v1",
            api_key="not-needed",
            timeout=30,
        )
        self.use_local = True

    def teach(self, question: str, level: str = "adult") -> TeachingResponse:
        level_prompts = {
            "little": "Explain very simply for a young child. Use fun, playful examples.",
            "kid": "Explain simply for a child. Use fun examples and comparisons.",
            "teen": "Explain clearly for a teenager. Be relatable.",
            "student": "Explain clearly for a student. Include key concepts.",
            "undergrad": "Explain for a university student. Be thorough.",
            "adult": "Explain clearly and directly. Respect my time.",
            "senior": "Explain clearly and patiently. No jargon.",
            "expert": "Give technical detail. Assume domain knowledge.",
            "researcher": "Full depth, cutting edge. OMEGA-MAX level.",
        }

        system = level_prompts.get(level, level_prompts["adult"])
        prompt = f"You are OMEGA Tutor, an expert teacher. {system}\n\nUser question: {question}"

        # Try LM Studio first
        try:
            response = self._local_client.chat.completions.create(
                model=self._local_model_id,
                messages=[
                    {"role": "system", "content": f"You are OMEGA Tutor, an expert teacher. {system}"},
                    {"role": "user", "content": question},
                ],
                max_tokens=1000,
                temperature=0.7,
            )
            return TeachingResponse(
                explanation=response.choices[0].message.content,
                example="",
                topic=question[:50],
                exercise="",
            )
        except Exception:
            pass

        # Try Gemini
        if self._api_key:
            try:
                import google.generativeai as genai
                genai.configure(api_key=self._api_key)
                model = genai.GenerativeModel("gemini-pro")
                response = model.generate_content(prompt)
                return TeachingResponse(
                    explanation=response.text,
                    example="",
                    topic=question[:50],
                    exercise="",
                )
            except Exception as gemini_err:
                return TeachingResponse(
                    explanation=f"Gemini error: {gemini_err}",
                    example="",
                    topic=question[:50],
                    exercise="",
                )

        return TeachingResponse(
            explanation="No backend available. Start LM Studio with a model, or set GEMINI_API_KEY.",
            example="",
            topic=question[:50],
            exercise="",
        )

    def follow_up(self, question: str, level: str = "adult", context: str = "") -> TeachingResponse:
        full_question = f"Context: {context}\n\nQuestion: {question}" if context else question
        return self.teach(full_question, level)

    def deeper(self, topic: str, current_level: str) -> TeachingResponse:
        """Go one level deeper on the same topic."""
        return self.teach(
            f"Go deeper on: {topic}. Assume I already understand the basics. Add nuance, edge cases, or next-level connections.",
            level=current_level,
        )
