"""Store run artifacts on disk."""

import json
from pathlib import Path

from app.config import ARTIFACTS_DIR


def store_artifacts(
    run_id: str,
    artifact_manifest: list[str],
    run_payload: dict,
    uploaded_files: dict | None = None,
) -> Path:
    """
    Create artifacts/{run_id}/, write run.json, save any uploaded files.
    Returns the path to the run artifact directory.
    """
    ARTIFACTS_DIR.mkdir(parents=True, exist_ok=True)
    run_dir = ARTIFACTS_DIR / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    run_json = run_dir / "run.json"
    with open(run_json, "w", encoding="utf-8") as f:
        json.dump(run_payload, f, indent=2, default=str)

    if uploaded_files:
        for filename, content in uploaded_files.items():
            (run_dir / filename).write_bytes(
                content if isinstance(content, bytes) else content.encode("utf-8")
            )

    return run_dir


def get_artifacts_path(run_id: str) -> Path:
    """Return path to artifacts directory for a run."""
    return ARTIFACTS_DIR / run_id
