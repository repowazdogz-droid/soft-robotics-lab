#!/usr/bin/env python3
"""
OMEGA Hypothesis Ledger
=======================

A living, versioned registry of explicit causal hypotheses.
Uses shared OMEGA ID: hypothesis_id() for HYP- format.

Each hypothesis includes:
- Precise claim (mechanism, not narrative)
- Scope & assumptions
- Predicted observables
- Competing hypotheses it conflicts with
- Current confidence interval
- Status: active / weakened / falsified / merged / killed

Key rule: No experiment exists without a parent hypothesis.

Usage:
    from hypothesis_ledger import HypothesisLedger
    ledger = HypothesisLedger()
    h = ledger.create("Drug X inhibits pathway Y", domain="drug_discovery")
    ledger.add_competing(h.id, "Drug X acts via pathway Z instead")
    ledger.update_confidence(h.id, 0.6, evidence="Experiment 1 results")
"""

import sys
from pathlib import Path

_PRODUCTS = Path(__file__).resolve().parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.id_generator import hypothesis_id

import sys
import json
import sqlite3
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import List, Dict, Optional
from enum import Enum
import hashlib

_repo_root = Path(__file__).resolve().parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))


class HypothesisStatus(Enum):
    ACTIVE = "active"
    WEAKENED = "weakened"
    STRENGTHENED = "strengthened"
    FALSIFIED = "falsified"
    MERGED = "merged"
    KILLED = "killed"
    PAUSED = "paused"


@dataclass
class Evidence:
    """A piece of evidence linked to a hypothesis."""
    id: str
    timestamp: str
    description: str
    direction: str  # "supports", "contradicts", "neutral"
    strength: float  # 0-1
    source: str  # experiment_id, paper, observation
    uncertainty: float = 0.2


# OMEGA-MAX substrates (when would this matter? which substrates?)
SUBSTRATES_CHOICES = ["materials", "compute", "bio", "mfg", "coordination", "env"]
TEMPORAL_CHOICES = ["t1", "t2", "t3", "t4"]
FALSIFICATION_COST_CHOICES = ["low", "medium", "high"]


@dataclass
class Hypothesis:
    """A falsifiable hypothesis. OMEGA-MAX aligned."""
    id: str
    created_at: str
    updated_at: str

    # Core
    claim: str  # Precise causal claim
    domain: str
    status: HypothesisStatus

    # Confidence
    confidence: float  # 0-1
    confidence_low: float
    confidence_high: float

    # Structure
    assumptions: List[str]
    predicted_observables: List[str]
    falsifiers: List[str]  # What would prove this wrong

    # Relationships
    competing_hypotheses: List[str]  # IDs of competing hypotheses
    parent_hypothesis: Optional[str] = None  # If derived from another
    child_hypotheses: List[str] = None

    # Evidence
    evidence: List[Evidence] = None

    # Metadata
    author: str = "unknown"
    tags: List[str] = None
    notes: str = ""

    # OMEGA-MAX alignment
    temporal_horizon: str = "t2"  # t1, t2, t3, t4 - when would this matter?
    substrates: List[str] = None  # materials, compute, bio, mfg, coordination, env
    srfc_status: str = "AMBER"  # GREEN/AMBER/RED - Can it work physically?
    vrfc_status: str = "AMBER"  # GREEN/AMBER/RED - Will it survive reality?
    who_benefits: List[str] = None
    who_loses: List[str] = None
    falsification_cost: str = "medium"  # low, medium, high - how expensive to test?
    next_step: str = ""  # Required if status=active (concrete action)

    def __post_init__(self):
        if self.child_hypotheses is None:
            self.child_hypotheses = []
        if self.evidence is None:
            self.evidence = []
        if self.tags is None:
            self.tags = []
        if self.substrates is None:
            self.substrates = []
        if self.who_benefits is None:
            self.who_benefits = []
        if self.who_loses is None:
            self.who_loses = []

    def is_stale(self) -> bool:
        """True if status=active and next_step is empty (validation rule)."""
        return (
            self.status == HypothesisStatus.ACTIVE
            and (not self.next_step or not self.next_step.strip())
        )

    def to_dict(self) -> Dict:
        d = asdict(self)
        d['status'] = self.status.value
        d['evidence'] = [asdict(e) for e in self.evidence]
        return d

    def to_card(self) -> str:
        """Generate a 1-page hypothesis card. OMEGA-MAX layout."""
        horizon_label = {"t1": "T1 (0-1y)", "t2": "T2 (1-5y)", "t3": "T3 (5-30y)", "t4": "T4 (30+y)"}.get(
            getattr(self, "temporal_horizon", "t2") or "t2", "T2 (1-5y)"
        )
        substrates_str = ", ".join(getattr(self, "substrates", []) or []) or "—"
        srfc = getattr(self, "srfc_status", "AMBER") or "AMBER"
        vrfc = getattr(self, "vrfc_status", "AMBER") or "AMBER"
        fals_cost = getattr(self, "falsification_cost", "medium") or "medium"
        next_step_str = (getattr(self, "next_step", "") or "").strip() or "—"
        who_ben = ", ".join(getattr(self, "who_benefits", []) or []) or "—"
        who_lose = ", ".join(getattr(self, "who_loses", []) or []) or "—"
        stale_warn = "\n║  ⚠ STALE - no next step defined\n" if self.is_stale() else ""

        return f"""
╔══════════════════════════════════════════════════════════════════╗
║  HYPOTHESIS CARD: {self.id}
╠══════════════════════════════════════════════════════════════════╣
║
║  CLAIM:
║  {self.claim}
║
║  STATUS: {self.status.value.upper()}
║  CONFIDENCE: {self.confidence:.0%} ({self.confidence_low:.0%} - {self.confidence_high:.0%})
║  DOMAIN: {self.domain}
║
║  HORIZON: {horizon_label}
║  SUBSTRATES: {substrates_str}
║  SRFC: {srfc} | VRFC: {vrfc}
║  FALSIFICATION COST: {fals_cost}
║  NEXT STEP: {next_step_str}
║{stale_warn}
║  WHO BENEFITS: {who_ben}
║  WHO LOSES: {who_lose}
║
╠══════════════════════════════════════════════════════════════════╣
║  ASSUMPTIONS:
║  {chr(10).join(f'  • {a}' for a in self.assumptions) or '  (none specified)'}
║
║  PREDICTED OBSERVABLES:
║  {chr(10).join(f'  • {p}' for p in self.predicted_observables) or '  (none specified)'}
║
║  FALSIFIERS (what would prove this wrong):
║  {chr(10).join(f'  • {f}' for f in self.falsifiers) or '  (none specified)'}
║
╠══════════════════════════════════════════════════════════════════╣
║  COMPETING HYPOTHESES:
║  {chr(10).join(f'  • {c}' for c in self.competing_hypotheses) or '  (none)'}
║
║  EVIDENCE ({len(self.evidence)} items):
║  {chr(10).join(f'  [{e.direction}] {e.description[:50]}' for e in self.evidence[:5]) or '  (none yet)'}
║
╠══════════════════════════════════════════════════════════════════╣
║  Created: {self.created_at[:10]}  |  Updated: {self.updated_at[:10]}  |  Author: {self.author}
╚══════════════════════════════════════════════════════════════════╝
"""


class HypothesisLedger:
    """Manages the hypothesis ledger."""

    def __init__(self, db_path: str = None):
        if db_path:
            self.db_path = Path(db_path)
        else:
            self.db_path = _repo_root / "data" / "hypothesis_ledger.db"
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _init_db(self) -> None:
        """Initialize database."""
        conn = sqlite3.connect(self.db_path)
        conn.executescript("""
            CREATE TABLE IF NOT EXISTS hypotheses (
                id TEXT PRIMARY KEY,
                created_at TEXT,
                updated_at TEXT,
                claim TEXT,
                domain TEXT,
                status TEXT,
                confidence REAL,
                confidence_low REAL,
                confidence_high REAL,
                assumptions TEXT,
                predicted_observables TEXT,
                falsifiers TEXT,
                competing_hypotheses TEXT,
                parent_hypothesis TEXT,
                child_hypotheses TEXT,
                author TEXT,
                tags TEXT,
                notes TEXT,
                temporal_horizon TEXT,
                substrates TEXT,
                srfc_status TEXT,
                vrfc_status TEXT,
                who_benefits TEXT,
                who_loses TEXT,
                falsification_cost TEXT,
                next_step TEXT
            );

            CREATE TABLE IF NOT EXISTS evidence (
                id TEXT PRIMARY KEY,
                hypothesis_id TEXT,
                timestamp TEXT,
                description TEXT,
                direction TEXT,
                strength REAL,
                source TEXT,
                uncertainty REAL,
                FOREIGN KEY (hypothesis_id) REFERENCES hypotheses(id)
            );

            CREATE TABLE IF NOT EXISTS history (
                id TEXT PRIMARY KEY,
                hypothesis_id TEXT,
                timestamp TEXT,
                action TEXT,
                old_value TEXT,
                new_value TEXT,
                reason TEXT,
                FOREIGN KEY (hypothesis_id) REFERENCES hypotheses(id)
            );

            CREATE INDEX IF NOT EXISTS idx_hypotheses_domain ON hypotheses(domain);
            CREATE INDEX IF NOT EXISTS idx_hypotheses_status ON hypotheses(status);
            CREATE INDEX IF NOT EXISTS idx_evidence_hypothesis ON evidence(hypothesis_id);
        """)
        # Migration: add OMEGA-MAX columns if table existed without them
        for col in [
            "temporal_horizon", "substrates", "srfc_status", "vrfc_status",
            "who_benefits", "who_loses", "falsification_cost", "next_step",
        ]:
            try:
                conn.execute(f"ALTER TABLE hypotheses ADD COLUMN {col} TEXT")
            except sqlite3.OperationalError:
                pass  # column already exists
        conn.commit()
        conn.close()

    def create(
        self,
        claim: str,
        domain: str,
        assumptions: List[str] = None,
        predicted_observables: List[str] = None,
        falsifiers: List[str] = None,
        author: str = "unknown",
        tags: List[str] = None,
        horizon: str = "t2",
        substrates: List[str] = None,
        falsification_cost: str = "medium",
        srfc_status: str = "AMBER",
        vrfc_status: str = "AMBER",
        who_benefits: List[str] = None,
        who_loses: List[str] = None,
        next_step: str = "",
    ) -> Hypothesis:
        """Create a new hypothesis. OMEGA-MAX: horizon, substrates, falsification_cost, etc."""
        h_id = hypothesis_id()
        now = datetime.now().isoformat()

        hypothesis = Hypothesis(
            id=h_id,
            created_at=now,
            updated_at=now,
            claim=claim,
            domain=domain,
            status=HypothesisStatus.ACTIVE,
            confidence=0.5,
            confidence_low=0.3,
            confidence_high=0.7,
            assumptions=assumptions or [],
            predicted_observables=predicted_observables or [],
            falsifiers=falsifiers or [],
            competing_hypotheses=[],
            author=author,
            tags=tags or [],
            temporal_horizon=horizon or "t2",
            substrates=substrates or [],
            srfc_status=srfc_status or "AMBER",
            vrfc_status=vrfc_status or "AMBER",
            who_benefits=who_benefits or [],
            who_loses=who_loses or [],
            falsification_cost=falsification_cost or "medium",
            next_step=(next_step or "").strip(),
        )

        self._save_hypothesis(hypothesis)
        self._log_history(h_id, "created", None, claim, "Initial creation")

        return hypothesis

    def get(self, hypothesis_id: str) -> Optional[Hypothesis]:
        """Get a hypothesis by ID."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute(
            "SELECT * FROM hypotheses WHERE id = ?", (hypothesis_id,)
        )
        row = cursor.fetchone()

        if not row:
            conn.close()
            return None

        ev_cursor = conn.execute(
            "SELECT * FROM evidence WHERE hypothesis_id = ?", (hypothesis_id,)
        )
        evidence = []
        for ev_row in ev_cursor.fetchall():
            evidence.append(
                Evidence(
                    id=ev_row[0],
                    timestamp=ev_row[2],
                    description=ev_row[3],
                    direction=ev_row[4],
                    strength=ev_row[5],
                    source=ev_row[6],
                    uncertainty=ev_row[7],
                )
            )

        conn.close()

        def _row(i, default=None):
            return row[i] if len(row) > i else default

        return Hypothesis(
            id=row[0],
            created_at=row[1],
            updated_at=row[2],
            claim=row[3],
            domain=row[4],
            status=HypothesisStatus(row[5]),
            confidence=row[6],
            confidence_low=row[7],
            confidence_high=row[8],
            assumptions=json.loads(row[9]) if row[9] else [],
            predicted_observables=json.loads(row[10]) if row[10] else [],
            falsifiers=json.loads(row[11]) if row[11] else [],
            competing_hypotheses=json.loads(row[12]) if row[12] else [],
            parent_hypothesis=row[13],
            child_hypotheses=json.loads(row[14]) if row[14] else [],
            author=row[15] or "unknown",
            tags=json.loads(row[16]) if row[16] else [],
            notes=row[17] or "",
            temporal_horizon=_row(18) or "t2",
            substrates=json.loads(_row(19)) if _row(19) else [],
            srfc_status=_row(20) or "AMBER",
            vrfc_status=_row(21) or "AMBER",
            who_benefits=json.loads(_row(22)) if _row(22) else [],
            who_loses=json.loads(_row(23)) if _row(23) else [],
            falsification_cost=_row(24) or "medium",
            next_step=_row(25) or "",
            evidence=evidence,
        )

    def list(
        self,
        domain: str = None,
        status: HypothesisStatus = None,
        tags: List[str] = None,
    ) -> List[Hypothesis]:
        """List hypotheses with optional filters."""
        conn = sqlite3.connect(self.db_path)

        query = "SELECT id FROM hypotheses WHERE 1=1"
        params = []

        if domain:
            query += " AND domain = ?"
            params.append(domain)

        if status:
            query += " AND status = ?"
            params.append(status.value)

        query += " ORDER BY updated_at DESC"

        cursor = conn.execute(query, params)
        ids = [row[0] for row in cursor.fetchall()]
        conn.close()

        hypotheses = [self.get(h_id) for h_id in ids]

        if tags:
            hypotheses = [h for h in hypotheses if any(t in h.tags for t in tags)]

        return hypotheses

    def update_confidence(
        self,
        hypothesis_id: str,
        new_confidence: float,
        evidence_description: str = None,
        reason: str = None,
    ) -> Hypothesis:
        """Update confidence based on new evidence."""
        h = self.get(hypothesis_id)
        if not h:
            raise ValueError(f"Hypothesis {hypothesis_id} not found")

        old_confidence = h.confidence
        h.confidence = max(0.0, min(1.0, new_confidence))
        h.confidence_low = max(0.0, h.confidence - 0.2)
        h.confidence_high = min(1.0, h.confidence + 0.2)
        h.updated_at = datetime.now().isoformat()

        if h.confidence < 0.2:
            h.status = HypothesisStatus.WEAKENED
        elif h.confidence > 0.8:
            h.status = HypothesisStatus.STRENGTHENED
        elif h.confidence < old_confidence - 0.1:
            h.status = HypothesisStatus.WEAKENED
        elif h.confidence > old_confidence + 0.1:
            h.status = HypothesisStatus.STRENGTHENED

        if evidence_description:
            direction = "supports" if new_confidence > old_confidence else "contradicts"
            self.add_evidence(
                hypothesis_id,
                evidence_description,
                direction,
                strength=abs(new_confidence - old_confidence),
            )

        self._save_hypothesis(h)
        self._log_history(
            hypothesis_id, "confidence_updated",
            str(old_confidence), str(new_confidence), reason
        )

        return h

    def update_fields(
        self,
        hypothesis_id: str,
        *,
        next_step: Optional[str] = None,
        substrates: Optional[List[str]] = None,
        who_benefits: Optional[List[str]] = None,
        who_loses: Optional[List[str]] = None,
        srfc_status: Optional[str] = None,
        vrfc_status: Optional[str] = None,
        temporal_horizon: Optional[str] = None,
        falsification_cost: Optional[str] = None,
    ) -> Optional[Hypothesis]:
        """Update only the provided OMEGA-MAX fields. Does not overwrite with None."""
        h = self.get(hypothesis_id)
        if not h:
            return None
        changes = []
        if next_step is not None:
            h.next_step = next_step.strip()
            changes.append("next_step")
        if substrates is not None:
            h.substrates = substrates
            changes.append("substrates")
        if who_benefits is not None:
            h.who_benefits = who_benefits
            changes.append("who_benefits")
        if who_loses is not None:
            h.who_loses = who_loses
            changes.append("who_loses")
        if srfc_status is not None:
            h.srfc_status = srfc_status
            changes.append("srfc_status")
        if vrfc_status is not None:
            h.vrfc_status = vrfc_status
            changes.append("vrfc_status")
        if temporal_horizon is not None:
            h.temporal_horizon = temporal_horizon
            changes.append("temporal_horizon")
        if falsification_cost is not None:
            h.falsification_cost = falsification_cost
            changes.append("falsification_cost")
        if not changes:
            return h
        h.updated_at = datetime.now().isoformat()
        self._save_hypothesis(h)
        self._log_history(hypothesis_id, "fields_updated", None, ",".join(changes), "CLI update")
        return h

    def add_evidence(
        self,
        hypothesis_id: str,
        description: str,
        direction: str,
        strength: float = 0.5,
        source: str = "observation",
    ) -> Evidence:
        """Add evidence to a hypothesis."""
        ev = Evidence(
            id=f"E-{datetime.now().strftime('%Y%m%d%H%M%S')}",
            timestamp=datetime.now().isoformat(),
            description=description,
            direction=direction,
            strength=strength,
            source=source,
        )

        conn = sqlite3.connect(self.db_path)
        conn.execute(
            """
            INSERT INTO evidence (id, hypothesis_id, timestamp, description, direction, strength, source, uncertainty)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """,
            (
                ev.id,
                hypothesis_id,
                ev.timestamp,
                ev.description,
                ev.direction,
                ev.strength,
                ev.source,
                ev.uncertainty,
            ),
        )
        conn.commit()
        conn.close()

        self._log_history(
            hypothesis_id, "evidence_added", None, description, f"Direction: {direction}"
        )

        return ev

    def add_competing(self, hypothesis_id: str, competing_claim: str) -> str:
        """Add a competing hypothesis."""
        h = self.get(hypothesis_id)
        if not h:
            raise ValueError(f"Hypothesis {hypothesis_id} not found")

        competing = self.create(
            claim=competing_claim,
            domain=h.domain,
            author=h.author,
            tags=h.tags,
        )

        h.competing_hypotheses.append(competing.id)
        competing.competing_hypotheses.append(hypothesis_id)

        self._save_hypothesis(h)
        self._save_hypothesis(competing)

        return competing.id

    def kill(self, hypothesis_id: str, reason: str) -> Hypothesis:
        """Kill a hypothesis (mark as not worth pursuing)."""
        h = self.get(hypothesis_id)
        if not h:
            raise ValueError(f"Hypothesis {hypothesis_id} not found")

        old_status = h.status.value
        h.status = HypothesisStatus.KILLED
        h.updated_at = datetime.now().isoformat()
        h.notes += f"\n\nKILLED: {reason}"

        self._save_hypothesis(h)
        self._log_history(hypothesis_id, "killed", old_status, "killed", reason)

        return h

    def falsify(self, hypothesis_id: str, evidence_description: str) -> Hypothesis:
        """Falsify a hypothesis based on evidence."""
        h = self.get(hypothesis_id)
        if not h:
            raise ValueError(f"Hypothesis {hypothesis_id} not found")

        h.status = HypothesisStatus.FALSIFIED
        h.confidence = 0.05
        h.confidence_low = 0.0
        h.confidence_high = 0.1
        h.updated_at = datetime.now().isoformat()

        self.add_evidence(
            hypothesis_id, evidence_description, "contradicts", strength=0.9
        )

        self._save_hypothesis(h)
        self._log_history(
            hypothesis_id, "falsified", None, evidence_description, "Falsified by evidence"
        )

        return h

    def _save_hypothesis(self, h: Hypothesis) -> None:
        """Save hypothesis to database. OMEGA-MAX fields included."""
        conn = sqlite3.connect(self.db_path)
        conn.execute(
            """
            INSERT OR REPLACE INTO hypotheses
            (id, created_at, updated_at, claim, domain, status, confidence, confidence_low, confidence_high,
             assumptions, predicted_observables, falsifiers, competing_hypotheses, parent_hypothesis,
             child_hypotheses, author, tags, notes,
             temporal_horizon, substrates, srfc_status, vrfc_status, who_benefits, who_loses, falsification_cost, next_step)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
            (
                h.id,
                h.created_at,
                h.updated_at,
                h.claim,
                h.domain,
                h.status.value,
                h.confidence,
                h.confidence_low,
                h.confidence_high,
                json.dumps(h.assumptions),
                json.dumps(h.predicted_observables),
                json.dumps(h.falsifiers),
                json.dumps(h.competing_hypotheses),
                h.parent_hypothesis,
                json.dumps(h.child_hypotheses),
                h.author,
                json.dumps(h.tags),
                h.notes,
                getattr(h, "temporal_horizon", "t2") or "t2",
                json.dumps(getattr(h, "substrates", []) or []),
                getattr(h, "srfc_status", "AMBER") or "AMBER",
                getattr(h, "vrfc_status", "AMBER") or "AMBER",
                json.dumps(getattr(h, "who_benefits", []) or []),
                json.dumps(getattr(h, "who_loses", []) or []),
                getattr(h, "falsification_cost", "medium") or "medium",
                getattr(h, "next_step", "") or "",
            ),
        )
        conn.commit()
        conn.close()

    def _log_history(
        self,
        hypothesis_id: str,
        action: str,
        old_value: Optional[str],
        new_value: str,
        reason: str,
    ) -> None:
        """Log a change to history."""
        conn = sqlite3.connect(self.db_path)
        conn.execute(
            """
            INSERT INTO history (id, hypothesis_id, timestamp, action, old_value, new_value, reason)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """,
            (
                f"LOG-{datetime.now().strftime('%Y%m%d%H%M%S%f')}",
                hypothesis_id,
                datetime.now().isoformat(),
                action,
                old_value,
                new_value,
                reason,
            ),
        )
        conn.commit()
        conn.close()


# CLI
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="OMEGA Hypothesis Ledger")
    subparsers = parser.add_subparsers(dest="command")

    create_parser = subparsers.add_parser("create", help="Create hypothesis (OMEGA-MAX aligned)")
    create_parser.add_argument("claim", help="The hypothesis claim")
    create_parser.add_argument("--domain", "-d", default="general", help="Domain")
    create_parser.add_argument("--author", "-a", default="unknown", help="Author")
    create_parser.add_argument("--horizon", default="t2", choices=TEMPORAL_CHOICES, help="Temporal horizon (t1/t2/t3/t4)")
    create_parser.add_argument("--substrates", default="", help="Comma-separated substrates: materials,compute,bio,mfg,coordination,env")
    create_parser.add_argument("--falsification-cost", default="medium", choices=FALSIFICATION_COST_CHOICES, dest="falsification_cost", help="Falsification cost: low/medium/high")
    create_parser.add_argument("--srfc", default="AMBER", choices=["GREEN", "AMBER", "RED"], help="SRFC status")
    create_parser.add_argument("--vrfc", default="AMBER", choices=["GREEN", "AMBER", "RED"], help="VRFC status")
    create_parser.add_argument("--next-step", default="", dest="next_step", help="Concrete next step (required if active)")

    list_parser = subparsers.add_parser("list", help="List hypotheses")
    list_parser.add_argument("--domain", "-d", help="Filter by domain")
    list_parser.add_argument("--status", "-s", help="Filter by status")

    show_parser = subparsers.add_parser("show", help="Show hypothesis card (OMEGA-MAX fields)")
    show_parser.add_argument("id", help="Hypothesis ID")

    update_parser = subparsers.add_parser("update", help="Update hypothesis (confidence and/or OMEGA-MAX fields)")
    update_parser.add_argument("id", help="Hypothesis ID")
    update_parser.add_argument("--confidence", "-c", type=float, dest="confidence", default=None, help="New confidence (0-1)")
    update_parser.add_argument("--reason", "-r", dest="reason", default=None, help="Reason for update (e.g. with -c)")
    update_parser.add_argument("--next-step", dest="next_step", default=None, help="Concrete next step")
    update_parser.add_argument("--substrates", default=None, help="Comma-separated substrates")
    update_parser.add_argument("--who-benefits", dest="who_benefits", default=None, help="Comma-separated who benefits")
    update_parser.add_argument("--who-loses", dest="who_loses", default=None, help="Comma-separated who loses")
    update_parser.add_argument("--srfc", default=None, choices=["GREEN", "AMBER", "RED"], help="SRFC status")
    update_parser.add_argument("--vrfc", default=None, choices=["GREEN", "AMBER", "RED"], help="VRFC status")
    update_parser.add_argument("--horizon", default=None, choices=TEMPORAL_CHOICES, help="Temporal horizon (t1/t2/t3/t4)")
    update_parser.add_argument("--falsification-cost", dest="falsification_cost", default=None, choices=FALSIFICATION_COST_CHOICES, help="Falsification cost")

    kill_parser = subparsers.add_parser("kill", help="Kill hypothesis")
    kill_parser.add_argument("id", help="Hypothesis ID")
    kill_parser.add_argument("reason", help="Reason for killing")

    args = parser.parse_args()

    ledger = HypothesisLedger()

    if args.command == "create":
        substrates_list = [s.strip() for s in (getattr(args, "substrates", "") or "").split(",") if s and s.strip()]
        h = ledger.create(
            args.claim,
            args.domain,
            author=args.author,
            horizon=getattr(args, "horizon", "t2"),
            substrates=substrates_list if substrates_list else None,
            falsification_cost=getattr(args, "falsification_cost", "medium"),
            srfc_status=getattr(args, "srfc", "AMBER"),
            vrfc_status=getattr(args, "vrfc", "AMBER"),
            next_step=getattr(args, "next_step", "") or "",
        )
        print(f"Created: {h.id}")
        if h.is_stale():
            print("⚠ STALE - no next step defined")
        print(h.to_card())

    elif args.command == "list":
        status = HypothesisStatus(args.status) if getattr(args, "status", None) else None
        hypotheses = ledger.list(domain=getattr(args, "domain", None), status=status)
        print(f"\nHypotheses ({len(hypotheses)}):")
        print("-" * 72)
        for h in hypotheses:
            horizon = getattr(h, "temporal_horizon", "t2") or "t2"
            srfc = getattr(h, "srfc_status", "AMBER") or "AMBER"
            vrfc = getattr(h, "vrfc_status", "AMBER") or "AMBER"
            print(f"  {h.id} | {h.status.value:12} | {horizon} | SRFC:{srfc} VRFC:{vrfc} | {h.confidence:.0%} | {h.claim[:36]}")

    elif args.command == "show":
        h = ledger.get(args.id)
        if h:
            if h.is_stale():
                print("⚠ STALE - no next step defined\n")
            print(h.to_card())
        else:
            print(f"Hypothesis {args.id} not found")

    elif args.command == "update":
        h = ledger.get(args.id)
        if not h:
            print(f"Hypothesis {args.id} not found")
        else:
            if getattr(args, "confidence", None) is not None:
                h = ledger.update_confidence(
                    args.id, args.confidence, reason=getattr(args, "reason", None)
                )
                print(f"Updated {h.id} confidence to {h.confidence:.0%} (status: {h.status.value})")
            kw = {}
            if getattr(args, "next_step", None) is not None:
                kw["next_step"] = args.next_step
            if getattr(args, "substrates", None) is not None:
                kw["substrates"] = [s.strip() for s in args.substrates.split(",") if s and s.strip()]
            if getattr(args, "who_benefits", None) is not None:
                kw["who_benefits"] = [s.strip() for s in args.who_benefits.split(",") if s and s.strip()]
            if getattr(args, "who_loses", None) is not None:
                kw["who_loses"] = [s.strip() for s in args.who_loses.split(",") if s and s.strip()]
            if getattr(args, "srfc", None) is not None:
                kw["srfc_status"] = args.srfc
            if getattr(args, "vrfc", None) is not None:
                kw["vrfc_status"] = args.vrfc
            if getattr(args, "horizon", None) is not None:
                kw["temporal_horizon"] = args.horizon
            if getattr(args, "falsification_cost", None) is not None:
                kw["falsification_cost"] = args.falsification_cost
            if kw:
                h = ledger.update_fields(args.id, **kw)
                if h:
                    print(f"Updated fields: {', '.join(kw.keys())}")
            if h and (getattr(args, "confidence", None) is None and not kw):
                print(f"No updates provided. Use -c/--confidence or other flags (--next-step, --srfc, etc.)")
            elif h:
                print(h.to_card())

    elif args.command == "kill":
        h = ledger.kill(args.id, args.reason)
        print(f"Killed {h.id}: {args.reason}")

    else:
        hypotheses = ledger.list(status=HypothesisStatus.ACTIVE)
        print(f"\nActive Hypotheses ({len(hypotheses)}):")
        print("-" * 60)
        for h in hypotheses:
            print(f"  {h.id} | {h.confidence:.0%} | {h.claim[:50]}")
