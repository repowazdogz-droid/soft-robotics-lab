"""
OMEGA Tutor — Voice input (Whisper) and output (TTS).
Optional: graceful fallback if dependencies not installed.
"""

import re
import tempfile
from pathlib import Path
from typing import List, Optional, Tuple

# Flags set after optional imports
_whisper_available = False
_tts_available = False
_whisper_backend: Optional[str] = None  # "faster_whisper" | "openai_whisper"
_tts_backend: Optional[str] = None  # "edge_tts" | "pyttsx3"

_whisper_model = None
_edge_voices: List[dict] = []
_pyttsx_engine = None

try:
    from faster_whisper import WhisperModel
    _whisper_model = WhisperModel("base", device="cpu", compute_type="int8")
    _whisper_available = True
    _whisper_backend = "faster_whisper"
except Exception:
    try:
        import whisper
        _whisper_model = whisper.load_model("base")
        _whisper_available = True
        _whisper_backend = "openai_whisper"
    except Exception:
        pass

try:
    import edge_tts
    import asyncio
    _edge_voices = []  # populated lazily
    _tts_available = True
    _tts_backend = "edge_tts"
except Exception:
    try:
        import pyttsx3
        _pyttsx_engine = pyttsx3.init()
        _tts_available = True
        _tts_backend = "pyttsx3"
    except Exception:
        pass


def _strip_markdown(text: str) -> str:
    """Remove markdown formatting for TTS (code blocks, bold, links, etc.)."""
    if not text or not isinstance(text, str):
        return ""
    t = text.strip()
    # Remove code blocks (```...``` and `...`)
    t = re.sub(r"```[\s\S]*?```", " ", t)
    t = re.sub(r"`[^`]+`", " ", t)
    # Remove bold/italic
    t = re.sub(r"\*\*([^*]+)\*\*", r"\1", t)
    t = re.sub(r"\*([^*]+)\*", r"\1", t)
    t = re.sub(r"__([^_]+)__", r"\1", t)
    t = re.sub(r"_([^_]+)_", r"\1", t)
    # Remove links [text](url)
    t = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", t)
    # Collapse whitespace and newlines
    t = re.sub(r"\s+", " ", t).strip()
    return t


def _chunk_sentences(text: str, max_chars: int = 400) -> List[str]:
    """Split text into sentence-sized chunks for TTS (avoids timeouts)."""
    text = _strip_markdown(text)
    if not text or len(text) <= max_chars:
        return [text] if text else []
    chunks = []
    current = []
    length = 0
    for part in re.split(r"(?<=[.!?])\s+", text):
        if length + len(part) + 1 <= max_chars:
            current.append(part)
            length += len(part) + 1
        else:
            if current:
                chunks.append(" ".join(current))
            current = [part] if len(part) <= max_chars else [part[:max_chars]]
            length = len(part)
    if current:
        chunks.append(" ".join(current))
    return chunks


class VoiceEngine:
    """Voice input (Whisper) and output (TTS). Optional deps; use .whisper_available / .tts_available."""

    def __init__(self):
        self._whisper = _whisper_model
        self._whisper_backend = _whisper_backend
        self._tts_backend = _tts_backend
        self._edge_voices = _edge_voices
        self._pyttsx = _pyttsx_engine

    @property
    def whisper_available(self) -> bool:
        return _whisper_available

    @property
    def tts_available(self) -> bool:
        return _tts_available

    def transcribe(self, audio_path: str) -> str:
        """Transcribe audio file to text. Returns empty string if Whisper not available or on error."""
        if not _whisper_available or not self._whisper:
            return ""
        path = Path(audio_path)
        if not path.exists():
            return ""
        try:
            if self._whisper_backend == "faster_whisper":
                segments, _ = self._whisper.transcribe(str(path), language=None)
                return " ".join(s.text for s in segments).strip()
            if self._whisper_backend == "openai_whisper":
                result = self._whisper.transcribe(str(path))
                return (result.get("text") or "").strip()
        except Exception:
            pass
        return ""

    def speak(
        self,
        text: str,
        voice: str = "default",
        rate: Optional[str] = None,
        slow_for_kids: bool = False,
    ) -> bytes:
        """
        Convert text to speech. Strips markdown. Chunks long text for pyttsx3.
        Returns audio bytes (WAV for pyttsx3, MP3 for edge-tts). Empty bytes on error.
        """
        if not _tts_available or not text:
            return b""
        plain = _strip_markdown(text)
        if not plain:
            return b""

        if self._tts_backend == "edge_tts":
            return self._speak_edge_tts(plain, voice, rate, slow_for_kids)
        if self._tts_backend == "pyttsx3":
            chunks = _chunk_sentences(plain, max_chars=400)
            return self._speak_pyttsx3(chunks or [plain], voice, rate, slow_for_kids)
        return b""

    def _speak_edge_tts(
        self,
        plain: str,
        voice: str,
        rate: Optional[str],
        slow_for_kids: bool,
    ) -> bytes:
        import edge_tts
        import asyncio

        rate = rate or ("-20%" if slow_for_kids else "+0%")
        voice_id = voice if voice and voice != "default" else "en-GB-SoniaNeural"  # British Female
        # Single call; edge_tts supports long text. Limit length for stability.
        to_speak = plain[:3000] if len(plain) > 3000 else plain

        async def _run():
            comm = edge_tts.Communicate(to_speak, voice_id, rate=rate)
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
                await comm.save(f.name)
            return f.name

        try:
            path = asyncio.run(_run())
            data = Path(path).read_bytes()
            Path(path).unlink(missing_ok=True)
            return data
        except Exception:
            return b""

    def _speak_pyttsx3(
        self,
        chunks: List[str],
        voice: str,
        rate: Optional[str],
        slow_for_kids: bool,
    ) -> bytes:
        if not self._pyttsx:
            return b""
        try:
            if rate is None and slow_for_kids:
                self._pyttsx.setProperty("rate", 120)
            elif rate:
                self._pyttsx.setProperty("rate", int(rate))
            voices = self._pyttsx.getProperty("voices")
            if voice and voice != "default" and voices:
                for v in voices:
                    if voice in v.id or voice in (v.name or ""):
                        self._pyttsx.setProperty("voice", v.id)
                        break
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                path = f.name
            self._pyttsx.save_to_file(" ".join(chunks), path)
            self._pyttsx.runAndWait()
            data = Path(path).read_bytes()
            Path(path).unlink(missing_ok=True)
            return data
        except Exception:
            return b""

    def get_available_voices(self) -> List[Tuple[str, str]]:
        """Return list of (voice_id, display_name) for TTS. Prefer British Female for kids."""
        if self._tts_backend == "edge_tts":
            return self._get_edge_voices()
        if self._tts_backend == "pyttsx3" and self._pyttsx:
            out = []
            for v in self._pyttsx.getProperty("voices"):
                name = getattr(v, "name", None) or v.id
                out.append((v.id, name))
            return out
        return []

    def _get_edge_voices(self) -> List[Tuple[str, str]]:
        import asyncio
        import edge_tts

        global _edge_voices

        if _edge_voices:
            return [(v.get("ShortName", ""), v.get("ShortName", "") + " – " + (v.get("Gender", ""))) for v in _edge_voices]

        async def _list():
            all_voices = await edge_tts.list_voices()
            return [{"ShortName": v["ShortName"], "Gender": v.get("Gender", ""), "Locale": v.get("Locale", "")} for v in all_voices if "en-" in (v.get("Locale") or "")]

        try:
            voices = asyncio.run(_list())
            _edge_voices = voices
            preferred = [v for v in voices if "en-GB" in v["ShortName"] and v.get("Gender") == "Female"]
            rest = [v for v in voices if v not in preferred]
            ordered = preferred + rest
            return [(v["ShortName"], v["ShortName"] + " – " + (v.get("Gender") or "")) for v in ordered]
        except Exception:
            return [("en-GB-SoniaNeural", "British Female")]


def detect_voice_command(text: str) -> Optional[str]:
    """
    Detect spoken commands in transcribed text. Returns:
    'simpler' | 'deeper' | 'quiz' | 'stop' | None
    """
    if not text or not isinstance(text, str):
        return None
    t = text.strip().lower()
    if not t:
        return None
    if any(x in t for x in ("simpler please", "simpler", "easier", "explain simpler")):
        return "simpler"
    if any(x in t for x in ("go deeper", "deeper", "more detail", "explain deeper")):
        return "deeper"
    if any(x in t for x in ("quiz me", "quiz", "test me", "give me a quiz")):
        return "quiz"
    if any(x in t for x in ("stop", "stop speaking", "quiet")):
        return "stop"
    return None


# Singleton
voice_engine = VoiceEngine()
