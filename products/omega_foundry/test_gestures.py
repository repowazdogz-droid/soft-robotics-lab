#!/usr/bin/env python3
"""Quick test: intent parser + design engine gesture detection."""

import sys
from pathlib import Path

_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from core.intent_parser import IntentParser
from core.design_engine import DesignEngine

p = IntentParser()
e = DesignEngine()

tests = [
    "hook grip for carrying bags",
    "lateral grip for holding cards",
    "spherical grip for apples",
    "tripod grip for pen",
]

for t in tests:
    spec = p.parse(t)
    design = e.generate(spec)
    d = design.design_dict
    print(f"{t!r}")
    print(f"  -> gesture={d.get('source_gesture')}, num_fingers={d.get('num_fingers')}")
    print()
