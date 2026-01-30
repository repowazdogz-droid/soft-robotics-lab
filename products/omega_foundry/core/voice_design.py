"""
OMEGA Foundry — Voice-to-design: transcribe voice, parse intent with LLM, generate design (MJCF).
Uses same voice infrastructure as Tutor (Whisper). LM Studio / Gemini for intent parsing.
"""

import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, Optional

_FOUNDRY_ROOT = Path(__file__).resolve().parent.parent
_PRODUCTS = _FOUNDRY_ROOT.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))

# Optional: Tutor voice engine for transcribe
_voice_engine = None
try:
    from omega_tutor.core.voice import VoiceEngine
    _voice_engine = VoiceEngine()
except Exception:
    pass


def _transcribe(audio_path: str) -> str:
    """Transcribe audio to text using Tutor VoiceEngine (Whisper). Returns empty string if unavailable."""
    if not _voice_engine or not getattr(_voice_engine, "whisper_available", False):
        return ""
    path = Path(audio_path)
    if not path.exists():
        return ""
    return _voice_engine.transcribe(str(path)) or ""


_llm_cache: Dict[str, Dict] = {}


def _call_llm_for_intent(transcript: str) -> Optional[Dict[str, Any]]:
    """
    Parse design intent from transcript. Try LM Studio first, then Gemini, then rule-based.
    Returns dict with domain, scale, material, target_object, params (and raw_intent).
    """
    if not (transcript or transcript.strip()):
        return None
    transcript = transcript.strip()[:2000]
    cache_key = transcript
    if cache_key in _llm_cache:
        return _llm_cache[cache_key]

    prompt = f"""The user described a robot/mechanism design in voice. Transcript: "{transcript}"

Extract design intent. Respond with ONLY a valid JSON object (no markdown, no code fence). Keys:
- "domain": one of "gripper", "mechanism", "enclosure"
- "scale": "small" | "medium" | "large"
- "material": optional string, e.g. "silicone", "tpu", "rigid", or null
- "target_object": optional string, e.g. "eggs", "electronics", or null
- "params": object with optional keys: num_fingers (int), gesture (string: pinch, power, tripod, etc.), type (for mechanism: hinge, four_bar_linkage, slider, cam_follower, gear_pair; for enclosure: box, housing, bracket, mount), waterproof (bool), ventilated (bool), force_requirement (gentle/medium/strong)
- "raw_intent": the original transcript or short summary

Examples:
- "Make me a two-finger gripper for eggs" -> {{"domain":"gripper","scale":"medium","material":"silicone","target_object":"eggs","params":{{"num_fingers":2,"gesture":"pinch","force_requirement":"gentle"}},"raw_intent":"two-finger gripper for eggs"}}
- "Create a waterproof enclosure, 10 by 15 centimeters" -> {{"domain":"enclosure","scale":"medium","params":{{"type":"box","waterproof":true}},"raw_intent":"waterproof enclosure 10x15 cm"}}
"""

    def _parse_json(text: str) -> Optional[Dict]:
        if not text:
            return None
        text = text.strip()
        if "```" in text:
            parts = text.split("```")
            if len(parts) > 1:
                text = parts[1]
                if text.startswith("json"):
                    text = text[4:]
        try:
            return json.loads(text)
        except Exception:
            return None

    # 1. LM Studio
    try:
        from openai import OpenAI
        client = OpenAI(base_url="http://localhost:1234/v1", api_key="lm-studio")
        r = client.chat.completions.create(
            model="local-model",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=1024,
        )
        text = (r.choices[0].message.content or "").strip()
        out = _parse_json(text)
        if out and out.get("domain") in ("gripper", "mechanism", "enclosure"):
            _llm_cache[cache_key] = out
            return out
    except Exception:
        pass

    # 2. Gemini
    try:
        api_key = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
        if api_key:
            import google.generativeai as genai
            genai.configure(api_key=api_key)
            model = genai.GenerativeModel("gemini-1.5-flash")
            response = model.generate_content(prompt)
            if response and response.text:
                out = _parse_json(response.text.strip())
                if out and out.get("domain") in ("gripper", "mechanism", "enclosure"):
                    _llm_cache[cache_key] = out
                    return out
    except Exception:
        pass

    # 3. Rule-based fallback from transcript
    from .intent_parser import IntentParser
    parser = IntentParser()
    spec = parser.parse(transcript)
    out = {
        "domain": spec.domain,
        "scale": spec.scale,
        "material": spec.material,
        "target_object": spec.target_object,
        "params": spec.params,
        "raw_intent": spec.raw_intent or transcript,
    }
    _llm_cache[cache_key] = out
    return out


def voice_to_intent(audio_path: str) -> Dict[str, Any]:
    """
    Transcribe audio and parse design intent (Whisper + LLM or rule-based).
    Returns dict: domain, scale, material, target_object, params, raw_intent. Empty dict on failure.
    """
    text = _transcribe(audio_path)
    if not text:
        return {"error": "Transcription failed or Whisper unavailable", "raw_intent": ""}
    intent = _call_llm_for_intent(text)
    if not intent:
        return {"error": "Could not parse intent", "raw_intent": text}
    return intent


def intent_to_design(intent: Dict[str, Any]) -> str:
    """
    Generate MJCF (or URDF for enclosure) from intent dict.
    Intent must have: domain, scale, params; optional material, target_object, raw_intent.
    """
    from .intent_parser import DesignSpec
    from .design_engine import DesignEngine

    domain = intent.get("domain", "gripper")
    scale = intent.get("scale", "medium")
    material = intent.get("material")
    target_object = intent.get("target_object")
    params = dict(intent.get("params") or {})
    raw_intent = intent.get("raw_intent", "")

    spec = DesignSpec(
        domain=domain,
        scale=scale,
        material=material,
        params=params,
        target_object=target_object,
        raw_intent=raw_intent,
    )
    engine = DesignEngine()
    design = engine.generate(spec)
    return design.mjcf_xml or design.urdf_xml or ""
