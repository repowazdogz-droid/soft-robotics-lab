"""POST a run bundle JSON file to the OMEGA Lab Cognition Server."""

import json
import sys
from pathlib import Path

import requests

SERVER = "http://localhost:8000"  # Change to mac-mini IP later


def post_run(bundle_path: str):
    """Send bundle JSON to POST /runs."""
    path = Path(bundle_path)
    if not path.exists():
        print(f"File not found: {bundle_path}")
        sys.exit(1)
    with open(path, encoding="utf-8") as f:
        bundle = json.load(f)

    response = requests.post(f"{SERVER}/runs", json=bundle)
    print(f"Status: {response.status_code}")
    print(f"Response: {response.json()}")
    return response


if __name__ == "__main__":
    if len(sys.argv) > 1:
        post_run(sys.argv[1])
    else:
        print("Usage: python post_run.py <bundle.json>")
