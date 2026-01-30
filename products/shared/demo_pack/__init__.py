"""OMEGA Demo Pack - known-good, known-bad, known-edge grippers for end-to-end demos."""
from pathlib import Path

DEMO_PACK_DIR = Path(__file__).parent


def get_known_good() -> str:
    return (DEMO_PACK_DIR / "known_good.xml").read_text(encoding="utf-8")


def get_known_bad() -> str:
    return (DEMO_PACK_DIR / "known_bad.xml").read_text(encoding="utf-8")


def get_known_edge() -> str:
    return (DEMO_PACK_DIR / "known_edge.xml").read_text(encoding="utf-8")


def get_demo(name: str) -> str:
    if name == "good":
        return get_known_good()
    elif name == "bad":
        return get_known_bad()
    elif name == "edge":
        return get_known_edge()
    else:
        raise ValueError(f"Unknown demo: {name}")


DEMO_DESCRIPTIONS = {
    "good": {
        "name": "Egg Gripper",
        "description": "Simple 2-finger gripper for delicate objects. Passes all validations.",
        "expected": "All tests pass, score 6/6",
    },
    "bad": {
        "name": "Unstable Mass Gripper",
        "description": "Gripper with unrealistic mass properties. Fails physics validation.",
        "expected": "Fails STABILITY_TEST (PHYSICS_INSTABILITY)",
    },
    "edge": {
        "name": "Collision-Risk Gripper",
        "description": "Gripper where fingers can collide. Passes with warnings.",
        "expected": "Warns on SELF_COLLISION_TEST (GEOMETRY_SELF_INTERSECTION)",
    },
}
