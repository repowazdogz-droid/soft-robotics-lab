#!/usr/bin/env python3
"""
OMEGA Evaluation Framework
==========================

Regression testing and drift detection for OMEGA queries.

- Define expected outcomes for key queries
- Run nightly/on-demand
- Alert if confidence drops or recommendations change
- Track drift over time

Usage:
    from omega_eval import EvalSuite
    suite = EvalSuite()
    suite.add_test("spine query", expected_confidence_min=0.5)
    results = suite.run()
"""

import sys
import json
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Callable, Any
import hashlib

_repo_root = Path(__file__).resolve().parent.parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

try:
    from omega_sim import Omega
except ImportError:
    Omega = None


@dataclass
class EvalTest:
    """A single evaluation test."""
    id: str
    query: str
    domain: Optional[str] = None

    # Expected outcomes
    expected_confidence_min: float = 0.0
    expected_confidence_max: float = 1.0
    expected_recommendation_contains: Optional[str] = None
    expected_recommendation_not_contains: Optional[str] = None

    # Custom validator
    custom_validator: Optional[Callable] = None

    # Metadata
    description: str = ""
    critical: bool = False
    tags: List[str] = field(default_factory=list)


@dataclass
class EvalResult:
    """Result of running a single test."""
    test_id: str
    query: str
    passed: bool

    # Actual outcomes
    actual_confidence: float
    actual_recommendation: str

    # Failure details
    failure_reasons: List[str] = field(default_factory=list)

    # Timing
    run_time_ms: float = 0.0
    timestamp: str = ""

    # For drift tracking
    query_hash: str = ""
    result_hash: str = ""


@dataclass
class EvalSuiteResult:
    """Result of running full eval suite."""
    suite_name: str
    timestamp: str

    total_tests: int
    passed: int
    failed: int
    critical_failures: int

    results: List[EvalResult]

    # Drift detection
    drift_detected: bool = False
    drift_details: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return asdict(self)

    def to_json(self, path: str = None) -> str:
        data = json.dumps(self.to_dict(), indent=2, default=str)
        if path:
            Path(path).write_text(data)
        return data

    def summary(self) -> str:
        status = "PASS" if self.failed == 0 else "FAIL"
        failed_lines = (
            "\n".join(f"  - {r.test_id}: {r.failure_reasons}" for r in self.results if not r.passed)
            if self.failed > 0
            else ""
        )
        return f"""
OMEGA Eval Suite: {self.suite_name}
{'='*50}
Status: {status}
Timestamp: {self.timestamp}

Results: {self.passed}/{self.total_tests} passed
Critical Failures: {self.critical_failures}
Drift Detected: {self.drift_detected}

{failed_lines}
"""


class EvalSuite:
    """Evaluation suite for OMEGA regression testing."""

    def __init__(self, name: str = "default"):
        self.name = name
        self.tests: List[EvalTest] = []
        self.omega = Omega() if Omega else None
        self.history_path = Path(__file__).parent / "eval_history"
        self.history_path.mkdir(exist_ok=True)

    def add_test(
        self,
        query: str,
        expected_confidence_min: float = 0.0,
        expected_confidence_max: float = 1.0,
        expected_recommendation_contains: str = None,
        expected_recommendation_not_contains: str = None,
        description: str = "",
        critical: bool = False,
        tags: List[str] = None,
    ) -> EvalTest:
        """Add a test to the suite."""
        test = EvalTest(
            id=f"test_{len(self.tests)+1:03d}",
            query=query,
            expected_confidence_min=expected_confidence_min,
            expected_confidence_max=expected_confidence_max,
            expected_recommendation_contains=expected_recommendation_contains,
            expected_recommendation_not_contains=expected_recommendation_not_contains,
            description=description,
            critical=critical,
            tags=tags or [],
        )
        self.tests.append(test)
        return test

    def run(self, tags_filter: List[str] = None) -> EvalSuiteResult:
        """Run all tests in the suite."""
        results = []

        tests_to_run = self.tests
        if tags_filter:
            tests_to_run = [t for t in self.tests if any(tag in t.tags for tag in tags_filter)]

        for test in tests_to_run:
            result = self._run_single_test(test)
            results.append(result)

        drift_detected, drift_details = self._check_drift(results)
        self._save_to_history(results)

        passed = sum(1 for r in results if r.passed)
        failed = len(results) - passed
        critical_failures = sum(
            1 for r, t in zip(results, tests_to_run) if not r.passed and t.critical
        )

        return EvalSuiteResult(
            suite_name=self.name,
            timestamp=datetime.now().isoformat(),
            total_tests=len(results),
            passed=passed,
            failed=failed,
            critical_failures=critical_failures,
            results=results,
            drift_detected=drift_detected,
            drift_details=drift_details,
        )

    def _run_single_test(self, test: EvalTest) -> EvalResult:
        """Run a single test."""
        import time

        start = time.time()
        failure_reasons = []
        confidence = 0.0
        recommendation = ""

        if not self.omega:
            run_time = (time.time() - start) * 1000
            return EvalResult(
                test_id=test.id,
                query=test.query,
                passed=False,
                actual_confidence=0.0,
                actual_recommendation="",
                failure_reasons=["Omega not available (omega_sim not loaded)"],
                run_time_ms=run_time,
                timestamp=datetime.now().isoformat(),
                query_hash=self._hash(test.query),
                result_hash="",
            )

        try:
            result = self.omega.query(test.query)
            conf = getattr(result, "confidence", None)
            confidence = float(getattr(conf, "value", conf)) if conf is not None else 0.0
            recs = getattr(result, "recommendations", []) or []
            recommendation = recs[0] if recs else str(getattr(result, "value", ""))[:500]
        except Exception as e:
            run_time = (time.time() - start) * 1000
            return EvalResult(
                test_id=test.id,
                query=test.query,
                passed=False,
                actual_confidence=0.0,
                actual_recommendation="",
                failure_reasons=[f"Exception: {e}"],
                run_time_ms=run_time,
                timestamp=datetime.now().isoformat(),
                query_hash=self._hash(test.query),
                result_hash="",
            )

        run_time = (time.time() - start) * 1000

        if confidence < test.expected_confidence_min:
            failure_reasons.append(
                f"Confidence {confidence:.2f} below minimum {test.expected_confidence_min:.2f}"
            )
        if confidence > test.expected_confidence_max:
            failure_reasons.append(
                f"Confidence {confidence:.2f} above maximum {test.expected_confidence_max:.2f}"
            )

        if test.expected_recommendation_contains:
            if test.expected_recommendation_contains.lower() not in recommendation.lower():
                failure_reasons.append(
                    f"Recommendation missing expected: '{test.expected_recommendation_contains}'"
                )

        if test.expected_recommendation_not_contains:
            if test.expected_recommendation_not_contains.lower() in recommendation.lower():
                failure_reasons.append(
                    f"Recommendation contains forbidden: '{test.expected_recommendation_not_contains}'"
                )

        if test.custom_validator:
            try:
                custom_result = test.custom_validator(result)
                if not custom_result:
                    failure_reasons.append("Custom validator returned False")
            except Exception as e:
                failure_reasons.append(f"Custom validator error: {e}")

        return EvalResult(
            test_id=test.id,
            query=test.query,
            passed=len(failure_reasons) == 0,
            actual_confidence=confidence,
            actual_recommendation=recommendation[:500],
            failure_reasons=failure_reasons,
            run_time_ms=run_time,
            timestamp=datetime.now().isoformat(),
            query_hash=self._hash(test.query),
            result_hash=self._hash(f"{confidence:.4f}:{recommendation}"),
        )

    def _check_drift(self, results: List[EvalResult]) -> tuple:
        """Check for drift against historical results."""
        drift_detected = False
        drift_details = []

        history_files = sorted(self.history_path.glob("eval_*.json"))
        if not history_files:
            return False, ["No historical data for drift comparison"]

        try:
            last_run = json.loads(history_files[-1].read_text())
            last_results = {r["test_id"]: r for r in last_run}

            for result in results:
                if result.test_id in last_results:
                    last = last_results[result.test_id]
                    conf_diff = abs(result.actual_confidence - last.get("actual_confidence", 0))
                    if conf_diff > 0.15:
                        drift_detected = True
                        drift_details.append(
                            f"{result.test_id}: Confidence drifted {conf_diff:.2f} "
                            f"({last.get('actual_confidence', 0):.2f} -> {result.actual_confidence:.2f})"
                        )
                    if result.result_hash and result.result_hash != last.get("result_hash", ""):
                        drift_details.append(f"{result.test_id}: Recommendation changed")
        except Exception as e:
            drift_details.append(f"Drift check error: {e}")

        return drift_detected, drift_details

    def _save_to_history(self, results: List[EvalResult]) -> None:
        """Save results to history for drift tracking."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        history_file = self.history_path / f"eval_{timestamp}.json"
        data = [asdict(r) for r in results]
        history_file.write_text(json.dumps(data, indent=2, default=str))
        history_files = sorted(self.history_path.glob("eval_*.json"))
        for old_file in history_files[:-30]:
            try:
                old_file.unlink()
            except OSError:
                pass

    def _hash(self, text: str) -> str:
        """Create a short hash for drift comparison."""
        return hashlib.md5(text.encode()).hexdigest()[:12]


def create_default_suite() -> EvalSuite:
    """Create a default evaluation suite with standard tests."""
    suite = EvalSuite("omega_default")

    suite.add_test(
        query="spine surgery L4-L5 fusion 45 degree cobb angle",
        expected_confidence_min=0.4,
        description="Basic spine surgery query should have reasonable confidence",
        critical=True,
        tags=["spine", "core"],
    )

    suite.add_test(
        query="drug discovery for rare genetic disorder affecting 1 in 100000",
        expected_confidence_min=0.2,
        description="Drug discovery should work even for rare diseases",
        tags=["drug", "core"],
    )

    suite.add_test(
        query="epidemic spread model R0=2.5 urban population",
        expected_confidence_min=0.4,
        description="Epidemic model should have decent confidence for standard R0",
        tags=["epidemic", "core"],
    )

    suite.add_test(
        query="economic impact of 10% tariff on imported electronics",
        expected_confidence_min=0.2,
        description="Economics model should provide some analysis",
        tags=["economics", "core"],
    )

    return suite


# CLI
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Run OMEGA Evaluation Suite")
    parser.add_argument("--suite", "-s", default="default", help="Suite name")
    parser.add_argument("--tags", "-t", nargs="*", help="Filter by tags")
    parser.add_argument("--output", "-o", help="Output JSON file")

    args = parser.parse_args()

    if args.suite == "default":
        suite = create_default_suite()
    else:
        suite = EvalSuite(args.suite)

    print(f"Running OMEGA Eval Suite: {suite.name}")
    print(f"Tests: {len(suite.tests)}")
    print("-" * 50)

    result = suite.run(tags_filter=args.tags)

    print(result.summary())

    if args.output:
        result.to_json(args.output)
        print(f"Results saved to: {args.output}")
