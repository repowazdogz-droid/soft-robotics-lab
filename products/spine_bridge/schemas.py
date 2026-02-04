"""
Spine Bridge case schema for design decision analysis.
Compatible with spine.runtime.schemas when available; otherwise used as local contract.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class CaseInput:
    """Input case for Spine decision analysis: design + problem context."""
    name: str
    domain: str
    objectives: List[str]
    constraints: List[str]  # from physics failures / hard requirements
    uncertainties: List[str]  # from validation warnings / unknowns
    context: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "domain": self.domain,
            "objectives": list(self.objectives),
            "constraints": list(self.constraints),
            "uncertainties": list(self.uncertainties),
            "context": dict(self.context),
        }
