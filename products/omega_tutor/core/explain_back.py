"""
OMEGA Tutor — "Explain it back" mastery evaluation.
LLM evaluates user explanation; feedback is encouraging, not harsh.
"""

import os
import sys
import json
import re
from pathlib import Path
from typing import Dict, Any, Optional, List

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def _call_llm(prompt: str, api_key: Optional[str] = None) -> str:
    """LM Studio first, then Gemini."""
    key = api_key or os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    try:
        from openai import OpenAI
        client = OpenAI(base_url="http://localhost:1234/v1", api_key="lm-studio")
        models = client.models.list()
        model_id = models.data[0].id if models.data else "local-model"
        r = client.chat.completions.create(model=model_id, messages=[{"role": "user", "content": prompt}], temperature=0.4)
        return (r.choices[0].message.content or "").strip()
    except Exception:
        pass
    if key:
        try:
            import google.generativeai as genai
            genai.configure(api_key=key)
            model = genai.GenerativeModel("gemini-2.0-flash")
            r = model.generate_content(prompt)
            return (r.text or "").strip()
        except Exception:
            pass
    return ""


def evaluate_explanation(
    topic: str,
    user_explanation: str,
    level: str,
    api_key: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Evaluate user's explanation. Never harsh — focus on what's right first.
    Returns: {score: 0-100, correct_points: [], missing_points: [], misconceptions: [], feedback: str}
    """
    topic = (topic or "").strip() or "general"
    level = (level or "adult").strip()
    user_text = (user_explanation or "").strip()[:2000]
    prompt = f"""You are a kind tutor. The learner just explained "{topic}" in their own words (they learn at {level} level).
Evaluate gently. Focus on what they got RIGHT first. Never be harsh.

Learner's explanation:
{user_text}

Respond with JSON only (no markdown):
{{
  "score": 0-100,
  "correct_points": ["thing they got right", ...],
  "missing_points": ["important point they didn't mention", ...],
  "misconceptions": ["only if they said something wrong", otherwise empty array],
  "feedback": "One short, encouraging sentence. Start with what they did well. If something is missing, suggest it kindly."
}}
"""
    raw = _call_llm(prompt, api_key)
    if not raw:
        return {
            "score": 50,
            "correct_points": [],
            "missing_points": [],
            "misconceptions": [],
            "feedback": "We couldn't evaluate this time. Your effort counts — try explaining again or ask for a hint!",
        }
    try:
        m = re.search(r"\{[\s\S]*\}", raw)
        if m:
            data = json.loads(m.group(0))
            score = max(0, min(100, int(data.get("score", 50))))
            return {
                "score": score,
                "correct_points": _ensure_list(data.get("correct_points", [])),
                "missing_points": _ensure_list(data.get("missing_points", []))[:5],
                "misconceptions": _ensure_list(data.get("misconceptions", []))[:3],
                "feedback": (data.get("feedback") or "Good effort!")[:400],
            }
    except Exception:
        pass
    return {
        "score": 50,
        "correct_points": [],
        "missing_points": [],
        "misconceptions": [],
        "feedback": "Good effort explaining! Keep practicing.",
    }


def _ensure_list(x: Any) -> List[str]:
    if isinstance(x, list):
        return [str(i) for i in x]
    return []
