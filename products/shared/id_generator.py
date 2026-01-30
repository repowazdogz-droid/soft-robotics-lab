"""
Shared OMEGA ID generator.
Format: PREFIX-YYYYMMDD-XXXXX or PREFIX-YYYYMMDD-SUFFIX
"""

import datetime
import uuid


def generate_id(prefix: str, suffix: str = None) -> str:
    """
    Generate standardized OMEGA ID.
    Format: PREFIX-YYYYMMDD-XXXXX or PREFIX-YYYYMMDD-SUFFIX

    Prefixes:
    - RUN: Simulation/training run
    - ART: Artifact (design, scene)
    - VAL: Validation result
    - HYP: Hypothesis
    - DEC: Decision brief
    - ERR: Error/failure
    - BDL: Audit bundle
    """
    date_str = datetime.datetime.now().strftime("%Y%m%d")
    if suffix:
        return f"{prefix}-{date_str}-{suffix}"
    else:
        short_uuid = uuid.uuid4().hex[:5].upper()
        return f"{prefix}-{date_str}-{short_uuid}"


# Convenience functions
def run_id(suffix=None):
    return generate_id("RUN", suffix)


def artifact_id(suffix=None):
    return generate_id("ART", suffix)


def validation_id(suffix=None):
    return generate_id("VAL", suffix)


def hypothesis_id():
    return f"HYP-{uuid.uuid4().hex[:8].upper()}"


def decision_id(suffix=None):
    return generate_id("DEC", suffix)


def error_id(error_type: str):
    return generate_id("ERR", error_type)


def bundle_id(suffix=None):
    return generate_id("BDL", suffix)
