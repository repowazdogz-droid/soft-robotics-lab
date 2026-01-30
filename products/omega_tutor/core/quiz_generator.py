"""
OMEGA Tutor — Generate quiz questions from learned topics.
Uses LLM to generate multiple choice and open questions.
"""

import os
import sys
import json
import re
from pathlib import Path
from typing import List, Dict, Any, Optional

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
        r = client.chat.completions.create(model=model_id, messages=[{"role": "user", "content": prompt}], temperature=0.5)
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


def generate_quiz(topic: str, level: str, n_questions: int = 3, api_key: Optional[str] = None) -> List[Dict[str, Any]]:
    """
    Generate n multiple choice questions about topic for level.
    Returns list of {question, options: [A, B, C, D], correct: "B", explanation}.
    """
    topic = (topic or "").strip() or "general"
    level = (level or "adult").strip()
    n_questions = max(1, min(10, n_questions))
    prompt = f"""Generate {n_questions} multiple choice questions about "{topic}" for a {level} learner.
Each question should test understanding, not just recall. Use 4 options A-D. One correct answer per question.
Format your response as a JSON array only, no other text. Each object must have:
- "question": string
- "options": array of 4 strings (labels A, B, C, D)
- "correct": "A" or "B" or "C" or "D"
- "explanation": string (brief, encouraging)

Example: [{{"question": "...", "options": ["...", "...", "...", "..."], "correct": "B", "explanation": "..."}}]
"""
    raw = _call_llm(prompt, api_key)
    if not raw:
        return _fallback_quiz(topic, n_questions)
    try:
        # Extract JSON array from response (handle markdown code blocks)
        json_str = raw
        m = re.search(r"\[[\s\S]*\]", raw)
        if m:
            json_str = m.group(0)
        data = json.loads(json_str)
        if not isinstance(data, list):
            data = [data]
        result = []
        for item in data:
            q = item.get("question", "")
            opts = item.get("options", [])
            if len(opts) < 4:
                opts = opts + [""] * (4 - len(opts))
            correct = str(item.get("correct", "A")).upper()
            if correct not in "ABCD":
                correct = "A"
            result.append({
                "question": q[:500],
                "options": opts[:4],
                "correct": correct,
                "explanation": (item.get("explanation") or "")[:500],
            })
        return result[:n_questions]
    except Exception:
        return _fallback_quiz(topic, n_questions)


def _fallback_quiz(topic: str, n: int) -> List[Dict[str, Any]]:
    """Fallback when LLM fails."""
    return [
        {
            "question": f"What is one key idea about {topic}?",
            "options": ["A concept you learned", "Something unrelated", "A detail to recall", "An application"],
            "correct": "A",
            "explanation": "Review the topic to strengthen your understanding.",
        }
    ] * min(n, 3)


def generate_open_question(topic: str, level: str, api_key: Optional[str] = None) -> Dict[str, Any]:
    """Generate one 'Explain it back' style question. Returns {question, key_points, example_answer}."""
    topic = (topic or "").strip() or "general"
    level = (level or "adult").strip()
    prompt = f"""Create ONE "Explain it back" question for someone who learned about "{topic}" at {level} level.
Return JSON only:
{{"question": "Explain [topic] in your own words...", "key_points": ["point1", "point2", "point3"], "example_answer": "A short model answer"}}
"""
    raw = _call_llm(prompt, api_key)
    if not raw:
        return {
            "question": f"Explain {topic} in your own words. What are the main ideas?",
            "key_points": ["Main idea 1", "Main idea 2"],
            "example_answer": "Review the material and try again.",
        }
    try:
        m = re.search(r"\{[\s\S]*\}", raw)
        if m:
            data = json.loads(m.group(0))
            return {
                "question": (data.get("question") or f"Explain {topic} in your own words.")[:500],
                "key_points": data.get("key_points", [])[:5],
                "example_answer": (data.get("example_answer") or "")[:500],
            }
    except Exception:
        pass
    return {
        "question": f"Explain {topic} in your own words. What are the main ideas?",
        "key_points": ["Main idea 1", "Main idea 2"],
        "example_answer": "Review the material and try again.",
    }
