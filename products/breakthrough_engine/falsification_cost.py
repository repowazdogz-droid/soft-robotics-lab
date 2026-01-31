"""
Falsification Cost Estimator

Given a hypothesis, estimate the cost (time, money, resources) to falsify it.
Integrate with VRFC to assess "is it worth testing?"
"""
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

class TestType(Enum):
    LITERATURE_REVIEW = "literature_review"
    COMPUTATIONAL = "computational"
    BENCH_EXPERIMENT = "bench_experiment"
    ANIMAL_STUDY = "animal_study"
    CLINICAL_PILOT = "clinical_pilot"
    CLINICAL_TRIAL = "clinical_trial"


@dataclass
class FalsificationPath:
    test_type: TestType
    description: str
    estimated_cost_usd: Tuple[int, int]
    estimated_time_days: Tuple[int, int]
    resources_needed: List[str]
    probability_of_resolution: float
    recommended: bool


@dataclass
class FalsificationEstimate:
    hypothesis_id: str
    hypothesis_claim: str
    paths: List[FalsificationPath]
    cheapest_path: Optional[FalsificationPath]
    fastest_path: Optional[FalsificationPath]
    recommended_path: Optional[FalsificationPath]
    total_paths: int
    worth_testing: bool
    worth_testing_reason: str


COST_TEMPLATES = {
    TestType.LITERATURE_REVIEW: {
        "cost": (0, 500),
        "time": (1, 7),
        "resources": ["Database access", "Research time"],
        "resolution_prob": 0.3,
    },
    TestType.COMPUTATIONAL: {
        "cost": (100, 5000),
        "time": (7, 30),
        "resources": ["Compute", "Software", "Developer time"],
        "resolution_prob": 0.5,
    },
    TestType.BENCH_EXPERIMENT: {
        "cost": (1000, 20000),
        "time": (14, 90),
        "resources": ["Lab access", "Materials", "Equipment", "Technician"],
        "resolution_prob": 0.7,
    },
    TestType.ANIMAL_STUDY: {
        "cost": (10000, 100000),
        "time": (60, 180),
        "resources": ["Animal facility", "IACUC approval", "Veterinary support"],
        "resolution_prob": 0.8,
    },
    TestType.CLINICAL_PILOT: {
        "cost": (50000, 500000),
        "time": (180, 365),
        "resources": ["IRB approval", "Clinical site", "Patient recruitment", "Medical staff"],
        "resolution_prob": 0.85,
    },
    TestType.CLINICAL_TRIAL: {
        "cost": (1000000, 50000000),
        "time": (365, 1825),
        "resources": ["Multi-site coordination", "CRO", "Regulatory affairs", "Data management"],
        "resolution_prob": 0.95,
    },
}


def determine_applicable_tests(domain: str, evidence_level: str, claim: str) -> List[TestType]:
    """Determine which test types apply to this hypothesis."""
    tests = [TestType.LITERATURE_REVIEW]
    claim_lower = claim.lower()
    if any(w in claim_lower for w in ["model", "predict", "simulate", "algorithm", "design"]):
        tests.append(TestType.COMPUTATIONAL)
    if any(w in claim_lower for w in ["material", "gripper", "robot", "actuator", "sensor", "physical", "mechanical"]):
        tests.append(TestType.BENCH_EXPERIMENT)
    if domain in ["medical", "medical_device", "therapeutic"]:
        if evidence_level in ["bench", "computational"]:
            tests.append(TestType.ANIMAL_STUDY)
        if evidence_level in ["animal", "preclinical"]:
            tests.append(TestType.CLINICAL_PILOT)
        if evidence_level in ["pilot", "clinical_pilot"]:
            tests.append(TestType.CLINICAL_TRIAL)
    if domain in ["robotics", "materials"]:
        tests.append(TestType.COMPUTATIONAL)
        tests.append(TestType.BENCH_EXPERIMENT)
    return list(set(tests))


def get_domain_cost_multiplier(domain: str) -> float:
    """Get cost multiplier based on domain."""
    multipliers = {
        "medical": 2.0,
        "medical_device": 1.5,
        "therapeutic": 2.5,
        "robotics": 1.0,
        "materials": 0.8,
        "software": 0.5,
        "general": 1.0,
    }
    return multipliers.get(domain, 1.0)


def generate_test_description(test_type: TestType, claim: str) -> str:
    """Generate a description of what the test would involve."""
    descriptions = {
        TestType.LITERATURE_REVIEW: "Systematic review of existing literature to find evidence for/against the claim",
        TestType.COMPUTATIONAL: "Computational modeling or simulation to test the hypothesis",
        TestType.BENCH_EXPERIMENT: "Laboratory experiment to physically test the hypothesis",
        TestType.ANIMAL_STUDY: "Preclinical animal study to assess safety and efficacy",
        TestType.CLINICAL_PILOT: "Small-scale human pilot study",
        TestType.CLINICAL_TRIAL: "Full clinical trial for regulatory approval",
    }
    return descriptions.get(test_type, "Test to evaluate hypothesis")


def assess_worth_testing(
    confidence: float,
    vrfc_status: str,
    cheapest_cost: int,
    resolution_prob: float,
) -> Tuple[bool, str]:
    """Assess if a hypothesis is worth the cost to test. Returns (worth_testing, reason)."""
    if confidence > 0.7 and vrfc_status == "GREEN":
        return True, "High confidence and clear translation path. Worth investment."
    if confidence < 0.3 and vrfc_status == "RED":
        return False, "Low confidence and blocked translation. Not worth investment yet."
    if cheapest_cost < 1000:
        return True, "Low cost to test. Worth resolving uncertainty."
    if vrfc_status == "AMBER":
        if cheapest_cost < 10000:
            return True, "Moderate cost, uncertain translation. Worth testing to reduce uncertainty."
        return False, "High cost with uncertain translation. Address VRFC issues first."
    expected_value = confidence * resolution_prob
    if expected_value > 0.4:
        return True, f"Expected value ({expected_value:.0%}) justifies testing."
    return False, f"Expected value ({expected_value:.0%}) too low for investment."


def estimate_falsification_cost(
    hypothesis_id: str,
    claim: str,
    domain: str = "general",
    current_evidence_level: str = "none",
    confidence: float = 0.5,
    vrfc_status: str = "AMBER",
) -> FalsificationEstimate:
    """Estimate the cost to falsify a hypothesis."""
    paths: List[FalsificationPath] = []
    applicable_tests = determine_applicable_tests(domain, current_evidence_level, claim)

    for test_type in applicable_tests:
        template = COST_TEMPLATES[test_type]
        cost_multiplier = get_domain_cost_multiplier(domain)
        path = FalsificationPath(
            test_type=test_type,
            description=generate_test_description(test_type, claim),
            estimated_cost_usd=(
                int(template["cost"][0] * cost_multiplier),
                int(template["cost"][1] * cost_multiplier),
            ),
            estimated_time_days=template["time"],
            resources_needed=template["resources"],
            probability_of_resolution=template["resolution_prob"],
            recommended=False,
        )
        paths.append(path)

    if not paths:
        return FalsificationEstimate(
            hypothesis_id=hypothesis_id,
            hypothesis_claim=claim,
            paths=[],
            cheapest_path=None,
            fastest_path=None,
            recommended_path=None,
            total_paths=0,
            worth_testing=False,
            worth_testing_reason="No applicable falsification paths identified",
        )

    cheapest = min(paths, key=lambda p: p.estimated_cost_usd[0])
    fastest = min(paths, key=lambda p: p.estimated_time_days[0])

    def path_score(p: FalsificationPath) -> float:
        avg_cost = (p.estimated_cost_usd[0] + p.estimated_cost_usd[1]) / 2
        avg_time = (p.estimated_time_days[0] + p.estimated_time_days[1]) / 2
        cost_score = 1 / (1 + avg_cost / 10000)
        time_score = 1 / (1 + avg_time / 30)
        return (cost_score + time_score + p.probability_of_resolution) / 3

    recommended = max(paths, key=path_score)
    recommended.recommended = True

    worth_testing, reason = assess_worth_testing(
        confidence=confidence,
        vrfc_status=vrfc_status,
        cheapest_cost=cheapest.estimated_cost_usd[0],
        resolution_prob=recommended.probability_of_resolution,
    )

    return FalsificationEstimate(
        hypothesis_id=hypothesis_id,
        hypothesis_claim=claim,
        paths=paths,
        cheapest_path=cheapest,
        fastest_path=fastest,
        recommended_path=recommended,
        total_paths=len(paths),
        worth_testing=worth_testing,
        worth_testing_reason=reason,
    )


def format_cost_range(cost_tuple: Tuple[int, int]) -> str:
    """Format cost range for display."""
    low, high = cost_tuple
    if low >= 1000000:
        return f"${low/1000000:.1f}M - ${high/1000000:.1f}M"
    if low >= 1000:
        return f"${low/1000:.0f}K - ${high/1000:.0f}K"
    return f"${low} - ${high}"


def format_time_range(time_tuple: Tuple[int, int]) -> str:
    """Format time range for display."""
    low, high = time_tuple
    if low >= 365:
        return f"{low/365:.1f} - {high/365:.1f} years"
    if low >= 30:
        return f"{low/30:.0f} - {high/30:.0f} months"
    return f"{low} - {high} days"
