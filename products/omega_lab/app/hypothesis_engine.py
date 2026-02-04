"""Simple Bayesian-style confidence update from run outcomes."""


def update_confidence(current: float, direction: str, strength: float) -> float:
    """
    Simple Bayesian-style heuristic:
    - supports → +0.05 * strength
    - refutes → -0.05 * strength
    - ambiguous → +0.01

    Clamp to [0.05, 0.95]
    Never allow absolute certainty.
    """
    if direction == "supports":
        delta = 0.05 * strength
    elif direction == "refutes":
        delta = -0.05 * strength
    else:
        delta = 0.01

    new_confidence = current + delta
    return max(0.05, min(0.95, new_confidence))
