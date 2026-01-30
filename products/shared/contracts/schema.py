"""
OMEGA Component Contract Schema
"""
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Optional
from enum import Enum
import json
from pathlib import Path


class ContractStatus(Enum):
    VALID = "valid"
    INVALID = "invalid"
    UNKNOWN = "unknown"


@dataclass
class Contract:
    """Component contract definition"""
    component: str  # e.g., "foundry", "reality_bridge"
    version: str  # e.g., "1.0"

    # What this component assumes about its inputs
    assumptions: Dict[str, Any] = field(default_factory=dict)

    # What this component guarantees about its outputs
    guarantees: List[str] = field(default_factory=list)

    # Conditions that cause this component to stop/fail
    stop_conditions: List[str] = field(default_factory=list)

    # Input schema (what it accepts)
    input_schema: Dict[str, Any] = field(default_factory=dict)

    # Output schema (what it produces)
    output_schema: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return asdict(self)

    def to_json(self, path: Path = None) -> str:
        data = self.to_dict()
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        return json.dumps(data, indent=2)

    @classmethod
    def from_json(cls, path: Path) -> "Contract":
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return cls(**data)


def validate_handoff(source_contract: Contract, target_contract: Contract, data: Dict) -> ContractStatus:
    """
    Validate that data from source meets target's assumptions.
    """
    # Check target's assumptions against data keys
    for assumption_key in target_contract.assumptions:
        if assumption_key not in data:
            return ContractStatus.INVALID
    return ContractStatus.VALID
