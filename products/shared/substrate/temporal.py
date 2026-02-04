"""
Temporal Queries - Query knowledge at points in time

Features:
- Query what we believed at a specific date
- Track belief changes over time
- Understand when and why beliefs changed
"""
from dataclasses import dataclass
from typing import List, Dict, Optional, Any, Tuple
from datetime import datetime, timedelta
from pathlib import Path
import json


@dataclass
class BeliefSnapshot:
    """A snapshot of belief at a point in time."""
    concept: str
    timestamp: str
    confidence: float
    source: str
    evidence: str

    def to_dict(self) -> dict:
        return {
            "concept": self.concept,
            "timestamp": self.timestamp,
            "confidence": self.confidence,
            "source": self.source,
            "evidence": self.evidence
        }


@dataclass
class BeliefChange:
    """A change in belief over time."""
    concept: str
    from_confidence: float
    to_confidence: float
    from_timestamp: str
    to_timestamp: str
    change_reason: str
    change_magnitude: float  # Absolute change

    def to_dict(self) -> dict:
        return {
            "concept": self.concept,
            "from_confidence": self.from_confidence,
            "to_confidence": self.to_confidence,
            "from_timestamp": self.from_timestamp,
            "to_timestamp": self.to_timestamp,
            "change_reason": self.change_reason,
            "change_magnitude": self.change_magnitude
        }


@dataclass
class BeliefTimeline:
    """Timeline of belief in a concept."""
    concept: str
    snapshots: List[BeliefSnapshot]
    changes: List[BeliefChange]
    current_confidence: float
    trend: str  # "increasing", "decreasing", "stable", "volatile"

    def to_dict(self) -> dict:
        return {
            "concept": self.concept,
            "snapshots": [s.to_dict() for s in self.snapshots],
            "changes": [c.to_dict() for c in self.changes],
            "current_confidence": self.current_confidence,
            "trend": self.trend
        }


class TemporalQueryEngine:
    """Query knowledge across time."""

    def __init__(self, lineage_tracker=None):
        """
        Initialize with substrate components.

        Args:
            lineage_tracker: Lineage instance for provenance (optional)
        """
        self.lineage = lineage_tracker
        self._belief_store: Dict[str, List[BeliefSnapshot]] = {}
        self._load_beliefs()

    def _get_data_file(self) -> Path:
        """Get path to belief data file."""
        data_dir = Path(__file__).resolve().parent / "data"
        data_dir.mkdir(parents=True, exist_ok=True)
        return data_dir / "beliefs.json"

    def _load_beliefs(self) -> None:
        """Load belief history."""
        data_file = self._get_data_file()
        if data_file.exists():
            try:
                data = json.loads(data_file.read_text(encoding="utf-8"))
                for concept, snapshots in data.get("beliefs", {}).items():
                    self._belief_store[concept] = [
                        BeliefSnapshot(**s) for s in snapshots
                    ]
            except Exception:
                pass

    def _save_beliefs(self) -> None:
        """Save belief history."""
        data_file = self._get_data_file()
        data = {
            "beliefs": {
                concept: [s.to_dict() for s in snapshots]
                for concept, snapshots in self._belief_store.items()
            },
            "updated_at": datetime.now().isoformat()
        }
        data_file.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def record_belief(
        self,
        concept: str,
        confidence: float,
        source: str,
        evidence: str = ""
    ) -> BeliefSnapshot:
        """
        Record a belief at the current time.

        Args:
            concept: What we believe (normalized key)
            confidence: How confident (0-1)
            source: Where this belief comes from
            evidence: Supporting evidence

        Returns:
            The recorded snapshot
        """
        snapshot = BeliefSnapshot(
            concept=concept,
            timestamp=datetime.now().isoformat(),
            confidence=confidence,
            source=source,
            evidence=evidence
        )

        if concept not in self._belief_store:
            self._belief_store[concept] = []

        self._belief_store[concept].append(snapshot)
        self._save_beliefs()

        return snapshot

    def query_at_time(
        self,
        concept: str,
        timestamp: str
    ) -> Optional[BeliefSnapshot]:
        """
        Query what we believed at a specific time.

        Args:
            concept: The concept to query
            timestamp: ISO format timestamp

        Returns:
            BeliefSnapshot at or before that time, or None
        """
        if concept not in self._belief_store:
            return None

        snapshots = self._belief_store[concept]
        try:
            target = datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
        except ValueError:
            target = datetime.fromisoformat(timestamp.replace("Z", ""))

        best = None
        for snapshot in snapshots:
            try:
                snap_time = datetime.fromisoformat(snapshot.timestamp.replace("Z", "+00:00"))
            except ValueError:
                snap_time = datetime.fromisoformat(snapshot.timestamp.replace("Z", ""))
            if snap_time <= target:
                if best is None:
                    best = snapshot
                else:
                    try:
                        best_time = datetime.fromisoformat(best.timestamp.replace("Z", "+00:00"))
                    except ValueError:
                        best_time = datetime.fromisoformat(best.timestamp.replace("Z", ""))
                    if snap_time > best_time:
                        best = snapshot

        return best

    def get_belief_timeline(
        self,
        concept: str
    ) -> Optional[BeliefTimeline]:
        """
        Get the full timeline of belief in a concept.

        Args:
            concept: The concept to get timeline for

        Returns:
            BeliefTimeline with all snapshots and changes
        """
        if concept not in self._belief_store:
            return None

        snapshots = sorted(
            self._belief_store[concept],
            key=lambda s: s.timestamp
        )

        if not snapshots:
            return None

        changes: List[BeliefChange] = []
        for i in range(1, len(snapshots)):
            prev = snapshots[i - 1]
            curr = snapshots[i]
            change = BeliefChange(
                concept=concept,
                from_confidence=prev.confidence,
                to_confidence=curr.confidence,
                from_timestamp=prev.timestamp,
                to_timestamp=curr.timestamp,
                change_reason=f"New evidence from {curr.source}",
                change_magnitude=abs(curr.confidence - prev.confidence)
            )
            changes.append(change)

        if len(snapshots) < 2:
            trend = "stable"
        else:
            first_confidence = snapshots[0].confidence
            last_confidence = snapshots[-1].confidence
            volatility = sum(c.change_magnitude for c in changes) / len(changes) if changes else 0

            if volatility > 0.2:
                trend = "volatile"
            elif last_confidence > first_confidence + 0.1:
                trend = "increasing"
            elif last_confidence < first_confidence - 0.1:
                trend = "decreasing"
            else:
                trend = "stable"

        return BeliefTimeline(
            concept=concept,
            snapshots=snapshots,
            changes=changes,
            current_confidence=snapshots[-1].confidence,
            trend=trend
        )

    def get_beliefs_at_time(
        self,
        timestamp: str,
        min_confidence: float = 0.0
    ) -> List[BeliefSnapshot]:
        """
        Get all beliefs at a specific point in time.

        Args:
            timestamp: ISO format timestamp
            min_confidence: Minimum confidence to include

        Returns:
            List of beliefs at that time
        """
        beliefs: List[BeliefSnapshot] = []
        for concept in self._belief_store:
            snapshot = self.query_at_time(concept, timestamp)
            if snapshot and snapshot.confidence >= min_confidence:
                beliefs.append(snapshot)
        return sorted(beliefs, key=lambda b: b.confidence, reverse=True)

    def get_belief_changes_since(
        self,
        since: str,
        min_change: float = 0.1
    ) -> List[BeliefChange]:
        """
        Get all belief changes since a timestamp.

        Args:
            since: ISO format timestamp
            min_change: Minimum change magnitude to include

        Returns:
            List of significant changes
        """
        try:
            since_dt = datetime.fromisoformat(since.replace("Z", "+00:00"))
        except ValueError:
            since_dt = datetime.fromisoformat(since.replace("Z", ""))
        all_changes: List[BeliefChange] = []

        for concept in self._belief_store:
            timeline = self.get_belief_timeline(concept)
            if timeline:
                for change in timeline.changes:
                    try:
                        change_dt = datetime.fromisoformat(change.to_timestamp.replace("Z", "+00:00"))
                    except ValueError:
                        change_dt = datetime.fromisoformat(change.to_timestamp.replace("Z", ""))
                    if change_dt >= since_dt and change.change_magnitude >= min_change:
                        all_changes.append(change)

        return sorted(all_changes, key=lambda c: c.to_timestamp, reverse=True)

    def compare_beliefs(
        self,
        timestamp_a: str,
        timestamp_b: str
    ) -> Dict[str, Any]:
        """
        Compare beliefs between two points in time.

        Args:
            timestamp_a: First timestamp
            timestamp_b: Second timestamp

        Returns:
            Comparison summary
        """
        beliefs_a = {b.concept: b for b in self.get_beliefs_at_time(timestamp_a)}
        beliefs_b = {b.concept: b for b in self.get_beliefs_at_time(timestamp_b)}
        all_concepts = set(beliefs_a.keys()) | set(beliefs_b.keys())

        added: List[BeliefSnapshot] = []
        removed: List[BeliefSnapshot] = []
        increased: List[Dict] = []
        decreased: List[Dict] = []
        unchanged: List[str] = []

        for concept in all_concepts:
            in_a = concept in beliefs_a
            in_b = concept in beliefs_b

            if in_b and not in_a:
                added.append(beliefs_b[concept])
            elif in_a and not in_b:
                removed.append(beliefs_a[concept])
            elif in_a and in_b:
                diff = beliefs_b[concept].confidence - beliefs_a[concept].confidence
                if diff > 0.05:
                    increased.append({
                        "concept": concept,
                        "from": beliefs_a[concept].confidence,
                        "to": beliefs_b[concept].confidence,
                        "change": diff
                    })
                elif diff < -0.05:
                    decreased.append({
                        "concept": concept,
                        "from": beliefs_a[concept].confidence,
                        "to": beliefs_b[concept].confidence,
                        "change": diff
                    })
                else:
                    unchanged.append(concept)

        return {
            "timestamp_a": timestamp_a,
            "timestamp_b": timestamp_b,
            "added": [b.to_dict() for b in added],
            "removed": [b.to_dict() for b in removed],
            "increased": increased,
            "decreased": decreased,
            "unchanged": unchanged,
            "summary": {
                "total_concepts_a": len(beliefs_a),
                "total_concepts_b": len(beliefs_b),
                "added_count": len(added),
                "removed_count": len(removed),
                "increased_count": len(increased),
                "decreased_count": len(decreased)
            }
        }

    def list_concepts(self) -> List[str]:
        """List all tracked concepts."""
        return list(self._belief_store.keys())
