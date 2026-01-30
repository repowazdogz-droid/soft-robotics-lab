#!/usr/bin/env python3
"""
OMEGA Dashboard (CLI)
=====================

View all queries, experiments, confidence levels, and audit trail.

Commands:
    python omega_dashboard.py recent          # Show recent queries
    python omega_dashboard.py experiments     # Show all experiments
    python omega_dashboard.py audit <id>      # Show audit trail for specific query
    python omega_dashboard.py stats           # Show overall statistics
    python omega_dashboard.py alerts          # Show low-confidence or flagged items
"""

import sys
import json
import sqlite3
from pathlib import Path
from datetime import datetime, timedelta
from dataclasses import dataclass
from typing import List, Dict, Optional
import argparse

_repo_root = Path(__file__).resolve().parent.parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))


@dataclass
class QueryRecord:
    """A record of a query."""
    id: str
    timestamp: str
    query: str
    domain: str
    confidence: float
    recommendation: str
    requires_approval: bool
    approved: Optional[bool] = None
    approved_by: Optional[str] = None


@dataclass
class ExperimentRecord:
    """A record of an experiment."""
    id: str
    timestamp: str
    name: str
    hypothesis: str
    status: str
    variants: int
    confidence: float


class OmegaDashboard:
    """CLI Dashboard for OMEGA audit and monitoring."""

    def __init__(self):
        self.db_path = _repo_root / "data" / "dashboard.db"
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()
        self.lab_db_path = _repo_root / "data" / "omega_lab.db"

    def _init_db(self) -> None:
        """Initialize dashboard database."""
        conn = sqlite3.connect(self.db_path)
        conn.executescript("""
            CREATE TABLE IF NOT EXISTS queries (
                id TEXT PRIMARY KEY,
                timestamp TEXT,
                query TEXT,
                domain TEXT,
                confidence REAL,
                recommendation TEXT,
                requires_approval INTEGER,
                approved INTEGER,
                approved_by TEXT,
                metadata TEXT
            );

            CREATE TABLE IF NOT EXISTS alerts (
                id TEXT PRIMARY KEY,
                timestamp TEXT,
                alert_type TEXT,
                query_id TEXT,
                message TEXT,
                acknowledged INTEGER DEFAULT 0
            );

            CREATE INDEX IF NOT EXISTS idx_queries_timestamp ON queries(timestamp);
            CREATE INDEX IF NOT EXISTS idx_queries_domain ON queries(domain);
            CREATE INDEX IF NOT EXISTS idx_alerts_acknowledged ON alerts(acknowledged);
        """)
        conn.commit()
        conn.close()

    def log_query(
        self,
        query_id: str,
        query: str,
        domain: str,
        confidence: float,
        recommendation: str,
        requires_approval: bool,
        metadata: Dict = None,
    ) -> None:
        """Log a query to the dashboard database."""
        conn = sqlite3.connect(self.db_path)
        conn.execute(
            """
            INSERT OR REPLACE INTO queries
            (id, timestamp, query, domain, confidence, recommendation, requires_approval, metadata)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """,
            (
                query_id,
                datetime.now().isoformat(),
                query,
                domain,
                confidence,
                recommendation,
                1 if requires_approval else 0,
                json.dumps(metadata or {}),
            ),
        )
        conn.commit()

        if confidence < 0.5 or requires_approval:
            alert_type = "low_confidence" if confidence < 0.5 else "requires_approval"
            message = (
                f"Query '{query[:50]}...' has {confidence:.0%} confidence"
                if confidence < 0.5
                else f"Query '{query[:50]}...' requires approval"
            )
            conn.execute(
                """
                INSERT INTO alerts (id, timestamp, alert_type, query_id, message)
                VALUES (?, ?, ?, ?, ?)
            """,
                (
                    f"alert_{datetime.now().strftime('%Y%m%d%H%M%S')}",
                    datetime.now().isoformat(),
                    alert_type,
                    query_id,
                    message,
                ),
            )
            conn.commit()

        conn.close()

    def get_recent_queries(
        self, limit: int = 20, domain: str = None
    ) -> List[QueryRecord]:
        """Get recent queries."""
        conn = sqlite3.connect(self.db_path)

        if domain:
            cursor = conn.execute(
                """
                SELECT id, timestamp, query, domain, confidence, recommendation, requires_approval, approved, approved_by
                FROM queries WHERE domain = ? ORDER BY timestamp DESC LIMIT ?
            """,
                (domain, limit),
            )
        else:
            cursor = conn.execute(
                """
                SELECT id, timestamp, query, domain, confidence, recommendation, requires_approval, approved, approved_by
                FROM queries ORDER BY timestamp DESC LIMIT ?
            """,
                (limit,),
            )

        records = []
        for row in cursor.fetchall():
            records.append(
                QueryRecord(
                    id=row[0],
                    timestamp=row[1],
                    query=row[2],
                    domain=row[3],
                    confidence=row[4],
                    recommendation=(row[5][:100] if row[5] else ""),
                    requires_approval=bool(row[6]),
                    approved=bool(row[7]) if row[7] is not None else None,
                    approved_by=row[8],
                )
            )

        conn.close()
        return records

    def get_experiments(self, status: str = None) -> List[ExperimentRecord]:
        """Get experiments from omega_lab database."""
        if not self.lab_db_path.exists():
            return []

        conn = sqlite3.connect(self.lab_db_path)

        try:
            if status:
                cursor = conn.execute(
                    """
                    SELECT id, timestamp, name, hypothesis, status, variants, confidence
                    FROM experiments WHERE status = ? ORDER BY timestamp DESC
                """,
                    (status,),
                )
            else:
                cursor = conn.execute(
                    """
                    SELECT id, timestamp, name, hypothesis, status, variants, confidence
                    FROM experiments ORDER BY timestamp DESC LIMIT 50
                """
                )

            records = []
            for row in cursor.fetchall():
                records.append(
                    ExperimentRecord(
                        id=row[0],
                        timestamp=row[1],
                        name=row[2],
                        hypothesis=row[3] or "",
                        status=row[4] or "unknown",
                        variants=row[5] or 0,
                        confidence=row[6] or 0.0,
                    )
                )

            conn.close()
            return records

        except sqlite3.OperationalError:
            conn.close()
            return []

    def get_alerts(self, acknowledged: bool = False) -> List[Dict]:
        """Get alerts."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute(
            """
            SELECT id, timestamp, alert_type, query_id, message
            FROM alerts WHERE acknowledged = ? ORDER BY timestamp DESC
        """,
            (1 if acknowledged else 0,),
        )

        alerts = []
        for row in cursor.fetchall():
            alerts.append(
                {
                    "id": row[0],
                    "timestamp": row[1],
                    "type": row[2],
                    "query_id": row[3],
                    "message": row[4],
                }
            )

        conn.close()
        return alerts

    def acknowledge_alert(self, alert_id: str) -> None:
        """Acknowledge an alert."""
        conn = sqlite3.connect(self.db_path)
        conn.execute("UPDATE alerts SET acknowledged = 1 WHERE id = ?", (alert_id,))
        conn.commit()
        conn.close()

    def get_stats(self) -> Dict:
        """Get overall statistics."""
        conn = sqlite3.connect(self.db_path)

        stats = {}

        cursor = conn.execute("SELECT COUNT(*) FROM queries")
        stats["total_queries"] = cursor.fetchone()[0]

        cursor = conn.execute("SELECT domain, COUNT(*) FROM queries GROUP BY domain")
        stats["by_domain"] = {row[0]: row[1] for row in cursor.fetchall()}

        cursor = conn.execute("SELECT AVG(confidence) FROM queries")
        avg = cursor.fetchone()[0]
        stats["avg_confidence"] = float(avg) if avg is not None else 0.0

        cursor = conn.execute("SELECT COUNT(*) FROM queries WHERE requires_approval = 1")
        stats["requiring_approval"] = cursor.fetchone()[0]

        cursor = conn.execute(
            "SELECT COUNT(*) FROM queries WHERE requires_approval = 1 AND approved IS NULL"
        )
        stats["pending_approvals"] = cursor.fetchone()[0]

        cursor = conn.execute("SELECT COUNT(*) FROM alerts WHERE acknowledged = 0")
        stats["unacknowledged_alerts"] = cursor.fetchone()[0]

        yesterday = (datetime.now() - timedelta(days=1)).isoformat()
        cursor = conn.execute("SELECT COUNT(*) FROM queries WHERE timestamp > ?", (yesterday,))
        stats["queries_24h"] = cursor.fetchone()[0]

        conn.close()
        return stats

    def get_audit_trail(self, query_id: str) -> Dict:
        """Get full audit trail for a query."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute(
            """
            SELECT id, timestamp, query, domain, confidence, recommendation,
                   requires_approval, approved, approved_by, metadata
            FROM queries WHERE id = ?
        """,
            (query_id,),
        )

        row = cursor.fetchone()
        conn.close()

        if not row:
            return {"error": "Query not found"}

        return {
            "id": row[0],
            "timestamp": row[1],
            "query": row[2],
            "domain": row[3],
            "confidence": row[4],
            "recommendation": row[5],
            "requires_approval": bool(row[6]),
            "approved": bool(row[7]) if row[7] is not None else None,
            "approved_by": row[8],
            "metadata": json.loads(row[9]) if row[9] else {},
        }


def print_table(
    headers: List[str], rows: List[List], max_widths: List[int] = None
) -> None:
    """Print a simple ASCII table."""
    if not rows:
        print("  (no data)")
        return

    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell)[:50]))

    if max_widths:
        widths = [min(w, m) for w, m in zip(widths, max_widths)]

    header_line = " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
    print(header_line)
    print("-" * len(header_line))

    for row in rows:
        cells = []
        for i, cell in enumerate(row):
            cell_str = str(cell)[: widths[i]].ljust(widths[i])
            cells.append(cell_str)
        print(" | ".join(cells))


def cmd_recent(args, dashboard: OmegaDashboard) -> None:
    """Show recent queries."""
    queries = dashboard.get_recent_queries(limit=args.limit, domain=args.domain)

    print(f"\n📋 Recent Queries (last {args.limit})")
    print("=" * 80)

    rows = []
    for q in queries:
        conf_str = (
            f"{q.confidence:.0%}" if isinstance(q.confidence, (int, float)) else str(q.confidence)
        )
        approval = (
            "⚠️"
            if q.requires_approval and q.approved is None
            else ("✓" if q.approved else "")
        )
        rows.append(
            [
                q.timestamp[:16] if q.timestamp else "",
                (q.domain or "")[:12],
                conf_str,
                approval,
                (q.query or "")[:40],
            ]
        )

    print_table(
        ["Timestamp", "Domain", "Conf", "Appr", "Query"],
        rows,
        [16, 12, 5, 4, 40],
    )


def cmd_experiments(args, dashboard: OmegaDashboard) -> None:
    """Show experiments."""
    experiments = dashboard.get_experiments(status=args.status)

    print("\n🧪 Experiments")
    print("=" * 80)

    rows = []
    for e in experiments:
        rows.append(
            [
                (e.timestamp or "")[:16],
                (e.name or "")[:20],
                e.status or "",
                str(e.variants),
                f"{e.confidence:.0%}" if isinstance(e.confidence, (int, float)) else "",
            ]
        )

    print_table(
        ["Timestamp", "Name", "Status", "Vars", "Conf"],
        rows,
        [16, 20, 10, 5, 5],
    )


def cmd_stats(args, dashboard: OmegaDashboard) -> None:
    """Show statistics."""
    stats = dashboard.get_stats()

    print("\n📊 OMEGA Statistics")
    print("=" * 40)
    print(f"  Total Queries:        {stats['total_queries']}")
    print(f"  Queries (24h):        {stats['queries_24h']}")
    print(f"  Avg Confidence:       {stats['avg_confidence']:.0%}")
    print(f"  Requiring Approval:   {stats['requiring_approval']}")
    print(f"  Pending Approvals:   {stats['pending_approvals']}")
    print(f"  Unacked Alerts:       {stats['unacknowledged_alerts']}")
    print()
    print("  By Domain:")
    for domain, count in stats.get("by_domain", {}).items():
        print(f"    {domain}: {count}")


def cmd_alerts(args, dashboard: OmegaDashboard) -> None:
    """Show alerts."""
    acknowledged = getattr(args, "all", False)
    alerts = dashboard.get_alerts(acknowledged=acknowledged)

    status = "All" if acknowledged else "Unacknowledged"
    print(f"\n🚨 Alerts ({status})")
    print("=" * 80)

    rows = []
    for a in alerts:
        rows.append(
            [
                (a["timestamp"] or "")[:16],
                a["type"],
                (a["message"] or "")[:50],
            ]
        )

    print_table(
        ["Timestamp", "Type", "Message"],
        rows,
        [16, 20, 50],
    )


def cmd_audit(args, dashboard: OmegaDashboard) -> None:
    """Show audit trail for a query."""
    trail = dashboard.get_audit_trail(args.query_id)

    if "error" in trail:
        print(f"Error: {trail['error']}")
        return

    print(f"\n🔍 Audit Trail: {args.query_id}")
    print("=" * 60)
    print(f"  Timestamp:       {trail['timestamp']}")
    print(f"  Query:           {trail['query']}")
    print(f"  Domain:          {trail['domain']}")
    conf = trail["confidence"]
    print(
        f"  Confidence:      {conf:.0%}"
        if isinstance(conf, (int, float))
        else f"  Confidence:      {conf}"
    )
    rec = trail.get("recommendation", "")
    print(f"  Recommendation:  {(rec[:100] if rec else '')}")
    print(f"  Requires Approval: {trail['requires_approval']}")
    print(f"  Approved:        {trail['approved']}")
    print(f"  Approved By:     {trail['approved_by']}")
    print()
    print("  Metadata:")
    for k, v in trail.get("metadata", {}).items():
        print(f"    {k}: {v}")


def main() -> None:
    parser = argparse.ArgumentParser(description="OMEGA Dashboard CLI")
    subparsers = parser.add_subparsers(dest="command", help="Command")

    recent_parser = subparsers.add_parser("recent", help="Show recent queries")
    recent_parser.add_argument("--limit", "-l", type=int, default=20, help="Number of queries")
    recent_parser.add_argument("--domain", "-d", help="Filter by domain")

    exp_parser = subparsers.add_parser("experiments", help="Show experiments")
    exp_parser.add_argument("--status", "-s", help="Filter by status")

    subparsers.add_parser("stats", help="Show statistics")

    alerts_parser = subparsers.add_parser("alerts", help="Show alerts")
    alerts_parser.add_argument("--all", "-a", action="store_true", help="Include acknowledged")

    audit_parser = subparsers.add_parser("audit", help="Show audit trail")
    audit_parser.add_argument("query_id", help="Query ID")

    args = parser.parse_args()
    dashboard = OmegaDashboard()

    if args.command == "recent":
        cmd_recent(args, dashboard)
    elif args.command == "experiments":
        cmd_experiments(args, dashboard)
    elif args.command == "stats":
        cmd_stats(args, dashboard)
    elif args.command == "alerts":
        cmd_alerts(args, dashboard)
    elif args.command == "audit":
        cmd_audit(args, dashboard)
    else:
        cmd_stats(args, dashboard)
        print()
        cmd_alerts(args, dashboard)


if __name__ == "__main__":
    main()
