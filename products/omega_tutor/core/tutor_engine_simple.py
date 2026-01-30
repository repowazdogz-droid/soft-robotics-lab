"""
Minimal TutorEngine - no memory, no substrate, just works.
"""
from openai import OpenAI
from typing import Dict, Any

class TutorEngine:
    def __init__(self):
        self._client = OpenAI(
            base_url="http://localhost:1234/v1",
            api_key="not-needed",
            timeout=30
        )
        self._local_model_id = "phi-3-mini-4k-instruct"

    def teach(self, question: str, level: str = "adult") -> Dict[str, Any]:
        level_prompts = {
            "kid": "Explain simply for a child. Use fun examples.",
            "student": "Explain clearly for a student.",
            "adult": "Explain clearly and directly.",
            "expert": "Give technical detail.",
            "researcher": "Full depth, cutting edge."
        }

        system = level_prompts.get(level, level_prompts["adult"])

        try:
            response = self._client.chat.completions.create(
                model=self._local_model_id,
                messages=[
                    {"role": "system", "content": f"You are OMEGA Tutor. {system}"},
                    {"role": "user", "content": question}
                ],
                max_tokens=1000,
                temperature=0.7
            )
            return {
                "explanation": response.choices[0].message.content,
                "example": "",
                "topic": question[:50]
            }
        except Exception as e:
            return {
                "explanation": f"Error: {e}",
                "example": "",
                "topic": question[:50]
            }
