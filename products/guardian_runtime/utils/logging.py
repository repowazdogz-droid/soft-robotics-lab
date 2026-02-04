"""
Structured logging for Guardian Runtime.
"""
import json
import logging
import sys
from datetime import datetime, timezone
from typing import Any, Dict, Optional


def structured_log(
    level: str,
    message: str,
    agent_id: Optional[str] = None,
    session_id: Optional[str] = None,
    action: Optional[str] = None,
    decision: Optional[str] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> None:
    """Emit a structured log line (JSON-friendly)."""
    record = {
        "timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "level": level,
        "message": message,
    }
    if agent_id:
        record["agent_id"] = agent_id
    if session_id:
        record["session_id"] = session_id
    if action:
        record["action"] = action
    if decision:
        record["decision"] = decision
    if extra:
        record["extra"] = extra
    out = json.dumps(record)
    if level == "ERROR":
        sys.stderr.write(out + "\n")
    else:
        sys.stdout.write(out + "\n")


def get_logger(name: str) -> logging.Logger:
    """Return a standard logger for the given name."""
    return logging.getLogger(name)
