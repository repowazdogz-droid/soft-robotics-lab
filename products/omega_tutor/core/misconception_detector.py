"""
Misconception Detector - Catch common errors before they stick

- Common misconceptions per topic
- Detect in student explanations
- Proactive correction before quiz
"""
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple, Any


@dataclass
class Misconception:
    id: str
    topic: str
    misconception: str  # The wrong belief
    correct_understanding: str
    common_triggers: List[str]  # Phrases that indicate this misconception
    severity: str  # "minor", "moderate", "critical"
    correction_approach: str


# Misconception database
MISCONCEPTIONS: Dict[str, List[Misconception]] = {
    "soft_robotics": [
        Misconception(
            id="SR001",
            topic="soft_robotics",
            misconception="Soft robots are weak and can't apply force",
            correct_understanding="Soft robots can apply significant force through pneumatic actuation, tendon drives, or granular jamming. Some can lift many times their weight.",
            common_triggers=["soft means weak", "can't grip hard", "no strength", "too soft to"],
            severity="moderate",
            correction_approach="Show examples of high-force soft actuators, explain pneumatic amplification",
        ),
        Misconception(
            id="SR002",
            topic="soft_robotics",
            misconception="Soft robots can't be precise",
            correct_understanding="With proper sensing and control, soft robots can achieve high precision. Their compliance actually helps in delicate tasks.",
            common_triggers=["not precise", "can't be accurate", "imprecise", "no precision"],
            severity="moderate",
            correction_approach="Discuss proprioception in soft robots, show precision manipulation examples",
        ),
        Misconception(
            id="SR003",
            topic="soft_robotics",
            misconception="Silicone is the only material for soft robots",
            correct_understanding="Soft robots use many materials: hydrogels, shape memory alloys, electroactive polymers, fabric, even paper and food-safe materials.",
            common_triggers=["always silicone", "only silicone", "made of silicone", "silicone robot"],
            severity="minor",
            correction_approach="List diverse materials, explain material selection criteria",
        ),
    ],
    "materials_science": [
        Misconception(
            id="MS001",
            topic="materials_science",
            misconception="Stiffness and strength are the same thing",
            correct_understanding="Stiffness (Young's modulus) measures resistance to deformation. Strength measures maximum stress before failure. A material can be stiff but weak, or flexible but strong.",
            common_triggers=["stiff means strong", "strength and stiffness", "stiffer is stronger"],
            severity="critical",
            correction_approach="Use stress-strain curves, give concrete examples (glass vs rubber)",
        ),
        Misconception(
            id="MS002",
            topic="materials_science",
            misconception="Elastic means bouncy",
            correct_understanding="Elastic means the material returns to its original shape after deformation. It doesn't necessarily mean it bounces well (that's resilience).",
            common_triggers=["elastic like rubber band", "bouncy elastic", "elastic bounce"],
            severity="minor",
            correction_approach="Distinguish elasticity, resilience, and damping",
        ),
    ],
    "machine_learning": [
        Misconception(
            id="ML001",
            topic="machine_learning",
            misconception="More data always means better models",
            correct_understanding="Data quality matters more than quantity. Biased, noisy, or irrelevant data can make models worse. Sometimes less, cleaner data beats more messy data.",
            common_triggers=["more data better", "need more data", "big data always"],
            severity="moderate",
            correction_approach="Discuss data quality, bias, and diminishing returns",
        ),
        Misconception(
            id="ML002",
            topic="machine_learning",
            misconception="Neural networks understand like humans do",
            correct_understanding="Neural networks find statistical patterns. They don't have understanding, reasoning, or common sense in the human sense.",
            common_triggers=["AI understands", "model knows", "network thinks", "understands language"],
            severity="critical",
            correction_approach="Explain pattern matching vs understanding, show failure cases",
        ),
        Misconception(
            id="ML003",
            topic="machine_learning",
            misconception="High accuracy means the model is good",
            correct_understanding="Accuracy can be misleading, especially with imbalanced data. A model predicting 'no cancer' 99% of the time has 99% accuracy if 99% of cases are healthy, but it's useless.",
            common_triggers=["99% accurate", "high accuracy good", "accuracy score"],
            severity="critical",
            correction_approach="Introduce precision, recall, F1, and domain-appropriate metrics",
        ),
    ],
    "physics": [
        Misconception(
            id="PH001",
            topic="physics",
            misconception="Heavier objects fall faster",
            correct_understanding="In a vacuum, all objects fall at the same rate regardless of mass. Air resistance creates differences in everyday experience.",
            common_triggers=["heavy falls faster", "heavier drops quicker", "weight affects falling"],
            severity="moderate",
            correction_approach="Explain Galileo's experiment, distinguish mass from air resistance",
        ),
        Misconception(
            id="PH002",
            topic="physics",
            misconception="Force is needed to maintain motion",
            correct_understanding="Newton's first law: objects in motion stay in motion unless acted upon by a force. Force changes motion, not maintains it.",
            common_triggers=["need force to move", "force keeps moving", "push to keep going"],
            severity="critical",
            correction_approach="Explain inertia, use frictionless examples, Newton's first law",
        ),
    ],
    "biology": [
        Misconception(
            id="BI001",
            topic="biology",
            misconception="Evolution is goal-directed or progressive",
            correct_understanding="Evolution has no goal. It's differential survival and reproduction based on current environment. There's no 'ladder' from simple to complex.",
            common_triggers=["evolved to", "evolution's purpose", "more evolved", "evolution wants"],
            severity="critical",
            correction_approach="Explain natural selection mechanism, random mutation, fitness as relative",
        ),
    ],
}


def detect_misconceptions(text: str, topic: Optional[str] = None) -> List[Tuple[Misconception, str]]:
    """
    Detect misconceptions in student text.

    Args:
        text: Student's explanation or answer
        topic: Optional topic filter

    Returns:
        List of (Misconception, matched_trigger) tuples
    """
    detected = []
    text_lower = (text or "").lower()

    if topic and topic in MISCONCEPTIONS:
        topics_to_check = [topic]
    else:
        # Try to match topic to keys (e.g. "soft robotics" -> soft_robotics)
        topic_norm = (topic or "").lower().replace(" ", "_").strip()
        if topic_norm in MISCONCEPTIONS:
            topics_to_check = [topic_norm]
        else:
            topics_to_check = list(MISCONCEPTIONS.keys())

    for t in topics_to_check:
        for misconception in MISCONCEPTIONS.get(t, []):
            for trigger in misconception.common_triggers:
                if trigger.lower() in text_lower:
                    detected.append((misconception, trigger))
                    break

    return detected


def get_topic_misconceptions(topic: str) -> List[Misconception]:
    """Get all known misconceptions for a topic."""
    if topic in MISCONCEPTIONS:
        return MISCONCEPTIONS[topic]

    topic_lower = (topic or "").lower().replace(" ", "_")
    for key in MISCONCEPTIONS:
        if key in topic_lower or topic_lower in key:
            return MISCONCEPTIONS[key]
    # Fuzzy: e.g. "what is soft robotics" -> soft_robotics
    if "soft" in topic_lower and "robotic" in topic_lower:
        return MISCONCEPTIONS.get("soft_robotics", [])
    if "material" in topic_lower:
        return MISCONCEPTIONS.get("materials_science", [])
    if "machine" in topic_lower and "learn" in topic_lower:
        return MISCONCEPTIONS.get("machine_learning", [])
    if "physic" in topic_lower or "force" in topic_lower or "fall" in topic_lower:
        return MISCONCEPTIONS.get("physics", [])
    if "evolut" in topic_lower or "biolog" in topic_lower:
        return MISCONCEPTIONS.get("biology", [])

    return []


def generate_preemptive_warning(topic: str) -> Optional[str]:
    """Generate a preemptive warning about common misconceptions for a topic."""
    misconceptions = get_topic_misconceptions(topic)

    if not misconceptions:
        return None

    critical = [m for m in misconceptions if m.severity == "critical"]

    if not critical:
        return None

    warning = "⚠️ **Common misconception alert:**\n\n"
    for m in critical[:2]:
        warning += f"Many people think: *\"{m.misconception}\"*\n"
        warning += f"Actually: {m.correct_understanding}\n\n"

    return warning


def generate_correction(misconception: Misconception, student_text: str) -> str:
    """Generate a gentle correction for a detected misconception."""
    correction = f"""
I noticed something that might be a common misconception. Let me clarify:

**What you said suggests:** {misconception.misconception}

**The more accurate understanding is:** {misconception.correct_understanding}

**How to think about it:** {misconception.correction_approach}

This is a really common point of confusion, so don't worry! Many people think this way initially.
"""
    return correction.strip()


def check_explanation_for_misconceptions(
    topic: str,
    student_explanation: str,
) -> Dict[str, Any]:
    """
    Check a student's explanation for misconceptions.

    Returns dict with:
    - has_misconceptions: bool
    - detected: list of detected misconceptions
    - corrections: list of correction messages
    - severity: highest severity found
    """
    detected = detect_misconceptions(student_explanation, topic)

    if not detected:
        return {
            "has_misconceptions": False,
            "detected": [],
            "corrections": [],
            "severity": None,
        }

    corrections = [generate_correction(m, student_explanation) for m, _ in detected]

    severities_order = {"critical": 3, "moderate": 2, "minor": 1}
    max_pair = max(detected, key=lambda x: severities_order.get(x[0].severity, 0))
    max_severity = max_pair[0].severity

    return {
        "has_misconceptions": True,
        "detected": [
            {
                "id": m.id,
                "misconception": m.misconception,
                "correct": m.correct_understanding,
                "trigger": trigger,
                "severity": m.severity,
            }
            for m, trigger in detected
        ],
        "corrections": corrections,
        "severity": max_severity,
    }


def add_misconception(
    topic: str,
    misconception: str,
    correct_understanding: str,
    triggers: List[str],
    severity: str = "moderate",
    correction_approach: str = "",
) -> str:
    """Add a new misconception to the database (runtime only)."""
    if topic not in MISCONCEPTIONS:
        MISCONCEPTIONS[topic] = []

    new_id = f"{topic[:2].upper()}{len(MISCONCEPTIONS[topic]) + 1:03d}"

    new_misconception = Misconception(
        id=new_id,
        topic=topic,
        misconception=misconception,
        correct_understanding=correct_understanding,
        common_triggers=triggers,
        severity=severity,
        correction_approach=correction_approach or "Review the correct understanding",
    )

    MISCONCEPTIONS[topic].append(new_misconception)
    return new_id
