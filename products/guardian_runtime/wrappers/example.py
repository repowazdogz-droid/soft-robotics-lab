"""
Example wrapped agent: demonstrates GuardianWrapper usage.
"""
from typing import Any, Dict, List

from ..schemas.policy import Policy
from .base import GuardianWrapper


class ExampleAgent(GuardianWrapper):
    """Example agent that performs read_file and write_file (governed)."""

    def __init__(self, policies: List[Policy]):
        super().__init__(agent_id="example_agent", policies=policies)

    def _execute_action(self, action: str, params: Dict[str, Any]) -> Any:
        if action == "read_file":
            path = params.get("path", "")
            # Simulate read
            return {"content": f"(simulated read of {path})", "path": path}
        if action == "write_file":
            path = params.get("path", "")
            content = params.get("content", "")
            # Simulate write
            return {"path": path, "bytes": len(content)}
        return None

    def perform_task(self) -> None:
        """Example flow: read then write (both go through Governor)."""
        self.request_action("read_file", {"path": "/data/input.txt"})
        self.request_action("write_file", {"path": "/data/output.txt", "content": "output"})
