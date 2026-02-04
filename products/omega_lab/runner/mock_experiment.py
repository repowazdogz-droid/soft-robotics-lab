"""
Mock experiment runner for testing Lab OS integration
without requiring Isaac Sim
"""

import json
import random
from datetime import datetime
import requests

SERVER = "http://localhost:8000"


def mock_run(design_id: str, hypothesis_id: str, environment: str = "dry"):
    """Generate a mock run and POST to Lab OS"""

    run_id = f"R-{datetime.now().strftime('%Y%m%d-%H%M%S')}"

    # Mock metrics with some randomness
    slip_rate = round(random.uniform(0.05, 0.25), 3)
    contact_area = round(random.uniform(0.02, 0.04), 3)

    # Determine outcome
    if slip_rate < 0.15 and contact_area > 0.025:
        direction = "supports"
        strength = round(0.6 + random.uniform(0, 0.3), 2)
    elif slip_rate > 0.2 or contact_area < 0.02:
        direction = "refutes"
        strength = round(0.5 + random.uniform(0, 0.3), 2)
    else:
        direction = "ambiguous"
        strength = round(0.3 + random.uniform(0, 0.2), 2)

    bundle = {
        "run_id": run_id,
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "engine": "mock",
        "design_id": design_id,
        "hypothesis_id": hypothesis_id,
        "experiment_id": f"E-{design_id}-{environment}",
        "environment": environment,
        "metrics": {
            "slip_rate": slip_rate,
            "contact_area": contact_area,
            "grasp_force": round(random.uniform(1, 5), 2),
            "grasp_success": slip_rate < 0.2
        },
        "outcome": {
            "direction": direction,
            "strength": strength,
            "rationale": f"Slip: {slip_rate}, Contact: {contact_area}"
        },
        "artifact_manifest": [],
        "notes": "Mock experiment for testing"
    }

    print(f"Run: {run_id}")
    print(f"Metrics: slip={slip_rate}, contact={contact_area}")
    print(f"Outcome: {direction} (strength={strength})")

    # POST to Lab OS
    try:
        r = requests.post(f"{SERVER}/runs", json=bundle)
        if r.status_code == 200:
            result = r.json()
            print(f"Posted to Lab OS")
            print(f"New confidence: {result.get('new_confidence')}")
        else:
            print(f"Failed: {r.status_code}")
    except Exception as e:
        print(f"Error: {e}")

    return bundle


if __name__ == "__main__":
    import sys

    design_id = sys.argv[1] if len(sys.argv) > 1 else "GD-TEST"
    hypothesis_id = sys.argv[2] if len(sys.argv) > 2 else "H-TEST"
    environment = sys.argv[3] if len(sys.argv) > 3 else "dry"

    mock_run(design_id, hypothesis_id, environment)
