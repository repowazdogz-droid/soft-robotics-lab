"""
Runnable script: validate the "bad" demo gripper and print Passed, Score, Tests, Errors.
Run from reality_bridge: python test_validator.py
Or: python -c "import sys; sys.path.insert(0,'..'); from core.validator import validate_design; from shared.demo_pack import get_demo; r = validate_design(get_demo('bad')); print('Passed:', r.get('passed'), 'Score:', r.get('score'), 'Tests:', r.get('tests'), 'Errors:', r.get('errors'))"
"""

import sys
from pathlib import Path

# Add parent so "core" and "shared" are importable (run from reality_bridge or products)
ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(PARENT) not in sys.path:
    sys.path.insert(0, str(PARENT))

from core.validator import validate_design
from shared.demo_pack import get_demo

# Test bad gripper (should fail)
bad_mjcf = get_demo("bad")
result = validate_design(bad_mjcf)

print("=== Bad Gripper Test ===")
print("Passed:", result.get("passed"))
print("Score:", result.get("score"))
print("Tests:", result.get("tests"))
print("Errors:", result.get("errors"))
