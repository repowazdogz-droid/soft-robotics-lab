"""
Reality Bridge — Webhooks for external notifications.
Register, remove, list; trigger on validation_completed, validation_failed, batch_completed.
Config: data/webhooks.json. Async trigger with retry.
"""

import asyncio
import hashlib
import hmac
import json
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

import httpx

_DATA_DIR = Path(__file__).resolve().parent.parent / "data"
_WEBHOOKS_PATH = _DATA_DIR / "webhooks.json"


def _load_webhooks() -> Dict[str, Any]:
    if not _WEBHOOKS_PATH.exists():
        return {"webhooks": []}
    try:
        return json.loads(_WEBHOOKS_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {"webhooks": []}


def _save_webhooks(data: Dict[str, Any]) -> None:
    _DATA_DIR.mkdir(parents=True, exist_ok=True)
    _WEBHOOKS_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")


def register_webhook(url: str, events: List[str], secret: Optional[str] = None) -> str:
    """Register a webhook. Returns webhook_id."""
    data = _load_webhooks()
    webhook_id = f"wh-{uuid.uuid4().hex[:8]}"
    entry = {
        "id": webhook_id,
        "url": url.strip(),
        "events": list(events) if events else ["validation_completed", "validation_failed"],
        "secret": secret or "",
        "active": True,
    }
    data["webhooks"] = data.get("webhooks", []) + [entry]
    _save_webhooks(data)
    return webhook_id


def remove_webhook(webhook_id: str) -> bool:
    """Remove webhook by id. Returns True if removed."""
    data = _load_webhooks()
    before = len(data.get("webhooks", []))
    data["webhooks"] = [w for w in data.get("webhooks", []) if w.get("id") != webhook_id]
    if len(data["webhooks"]) < before:
        _save_webhooks(data)
        return True
    return False


def list_webhooks() -> List[Dict[str, Any]]:
    """List all webhooks (id, url, events, active; no secret)."""
    data = _load_webhooks()
    out = []
    for w in data.get("webhooks", []):
        out.append({
            "id": w.get("id"),
            "url": w.get("url"),
            "events": w.get("events", []),
            "active": w.get("active", True),
        })
    return out


def _sign_payload(payload: str, secret: str) -> str:
    if not secret:
        return ""
    return hmac.new(secret.encode("utf-8"), payload.encode("utf-8"), hashlib.sha256).hexdigest()


async def _send_one(url: str, payload: Dict[str, Any], secret: str, timeout: float = 10.0) -> bool:
    """POST payload to url with optional X-Webhook-Signature. Returns True if 2xx."""
    body = json.dumps(payload, default=str)
    headers = {"Content-Type": "application/json"}
    if secret:
        headers["X-Webhook-Signature"] = _sign_payload(body, secret)
    try:
        async with httpx.AsyncClient(timeout=timeout) as client:
            r = await client.post(url, content=body, headers=headers)
            return 200 <= r.status_code < 300
    except Exception:
        return False


async def trigger_webhook(event: str, payload: Dict[str, Any]) -> None:
    """
    Fire webhooks subscribed to event. Payload should include event, timestamp, and event-specific data.
    Retries once after 2s on failure. Non-blocking.
    """
    data = _load_webhooks()
    payload_base = {"event": event, "timestamp": __import__("datetime").datetime.utcnow().isoformat() + "Z", **payload}
    for w in data.get("webhooks", []):
        if not w.get("active", True):
            continue
        if event not in w.get("events", []):
            continue
        url = w.get("url")
        if not url:
            continue
        secret = w.get("secret", "") or ""
        ok = await _send_one(url, payload_base, secret)
        if not ok:
            await asyncio.sleep(2)
            await _send_one(url, payload_base, secret)


def trigger_webhook_sync(event: str, payload: Dict[str, Any]) -> None:
    """Synchronous wrapper: run trigger_webhook in event loop."""
    try:
        loop = asyncio.get_event_loop()
        if loop.is_running():
            asyncio.create_task(trigger_webhook(event, payload))
        else:
            loop.run_until_complete(trigger_webhook(event, payload))
    except Exception:
        try:
            asyncio.run(trigger_webhook(event, payload))
        except Exception:
            pass
