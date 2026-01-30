#!/usr/bin/env python3
"""
Calibration System
==================

Logs design outcomes (success/failure) to enable learning over time.

Features:
- Log experiment results against designs
- Track which parameter combinations work
- Export data for ML training
- Suggest parameter adjustments based on history
"""

import json
import sqlite3
import uuid
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple

@dataclass
class ExperimentResult:
    """Result of testing a gripper design."""
    id: str
    design_id: str
    timestamp: str
    success: bool
    task: str  # e.g., "grasp_egg", "pick_and_place", "surgical_pickup"
    environment: str  # e.g., "dry", "wet", "simulated"

    # Outcome metrics
    grasp_success_rate: Optional[float] = None  # 0-1
    damage_occurred: Optional[bool] = None
    slip_occurred: Optional[bool] = None
    cycle_count: Optional[int] = None  # How many grasps before failure

    # Design parameters (copied for analysis)
    gesture: Optional[str] = None
    num_fingers: Optional[int] = None
    actuator_type: Optional[str] = None
    material: Optional[str] = None
    compliance: Optional[float] = None

    # Notes
    notes: Optional[str] = None
    experimenter: Optional[str] = None


class CalibrationSystem:
    """Tracks design performance and learns from outcomes."""

    def __init__(self, db_path: str = None):
        if db_path is None:
            db_path = Path(__file__).resolve().parent.parent / "data" / "calibration.db"

        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)

        self._init_db()

    def _init_db(self):
        """Initialize database schema."""
        conn = sqlite3.connect(self.db_path)
        conn.execute("""
            CREATE TABLE IF NOT EXISTS experiments (
                id TEXT PRIMARY KEY,
                design_id TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                success INTEGER NOT NULL,
                task TEXT,
                environment TEXT,
                grasp_success_rate REAL,
                damage_occurred INTEGER,
                slip_occurred INTEGER,
                cycle_count INTEGER,
                gesture TEXT,
                num_fingers INTEGER,
                actuator_type TEXT,
                material TEXT,
                compliance REAL,
                notes TEXT,
                experimenter TEXT
            )
        """)

        conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_design_id ON experiments(design_id)
        """)

        conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_gesture ON experiments(gesture)
        """)

        conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_task ON experiments(task)
        """)

        conn.commit()
        conn.close()

    def log_result(self, result: ExperimentResult) -> str:
        """Log an experiment result."""
        conn = sqlite3.connect(self.db_path)

        conn.execute("""
            INSERT INTO experiments (
                id, design_id, timestamp, success, task, environment,
                grasp_success_rate, damage_occurred, slip_occurred, cycle_count,
                gesture, num_fingers, actuator_type, material, compliance,
                notes, experimenter
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            result.id,
            result.design_id,
            result.timestamp,
            int(result.success),
            result.task,
            result.environment,
            result.grasp_success_rate,
            int(result.damage_occurred) if result.damage_occurred is not None else None,
            int(result.slip_occurred) if result.slip_occurred is not None else None,
            result.cycle_count,
            result.gesture,
            result.num_fingers,
            result.actuator_type,
            result.material,
            result.compliance,
            result.notes,
            result.experimenter
        ))

        conn.commit()
        conn.close()

        return result.id

    def log_quick(
        self,
        design_id: str,
        success: bool,
        task: str = "general",
        notes: str = None,
        design_params: Dict = None
    ) -> str:
        """Quick logging with minimal info."""
        result = ExperimentResult(
            id=f"EXP-{uuid.uuid4().hex[:8].upper()}",
            design_id=design_id,
            timestamp=datetime.now().isoformat(),
            success=success,
            task=task,
            environment="unknown",
            notes=notes
        )

        # Extract design params if provided
        if design_params:
            result.gesture = design_params.get("source_gesture")
            result.num_fingers = design_params.get("num_fingers")
            result.actuator_type = design_params.get("primary_actuator")
            result.compliance = design_params.get("source_parameters", {}).get("object_compliance")

            fingers = design_params.get("finger_designs", [{}])
            if fingers:
                result.material = fingers[0].get("material")

        return self.log_result(result)

    def get_design_history(self, design_id: str) -> List[ExperimentResult]:
        """Get all experiments for a design."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute(
            "SELECT * FROM experiments WHERE design_id = ? ORDER BY timestamp DESC",
            (design_id,)
        )

        results = []
        for row in cursor.fetchall():
            results.append(self._row_to_result(row))

        conn.close()
        return results

    def get_stats_by_gesture(self) -> Dict[str, Dict]:
        """Get success statistics grouped by gesture type."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute("""
            SELECT gesture,
                   COUNT(*) as total,
                   SUM(success) as successes,
                   AVG(grasp_success_rate) as avg_rate
            FROM experiments
            WHERE gesture IS NOT NULL
            GROUP BY gesture
        """)

        stats = {}
        for row in cursor.fetchall():
            gesture, total, successes, avg_rate = row
            stats[gesture] = {
                "total_experiments": total,
                "successes": successes,
                "success_rate": successes / total if total > 0 else 0,
                "avg_grasp_rate": avg_rate
            }

        conn.close()
        return stats

    def get_stats_by_task(self) -> Dict[str, Dict]:
        """Get success statistics grouped by task."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute("""
            SELECT task,
                   COUNT(*) as total,
                   SUM(success) as successes,
                   AVG(grasp_success_rate) as avg_rate
            FROM experiments
            WHERE task IS NOT NULL
            GROUP BY task
        """)

        stats = {}
        for row in cursor.fetchall():
            task, total, successes, avg_rate = row
            stats[task] = {
                "total_experiments": total,
                "successes": successes,
                "success_rate": successes / total if total > 0 else 0,
                "avg_grasp_rate": avg_rate
            }

        conn.close()
        return stats

    def get_best_params_for_task(self, task: str) -> Dict:
        """Find best-performing parameter combinations for a task."""
        conn = sqlite3.connect(self.db_path)

        cursor = conn.execute("""
            SELECT gesture, num_fingers, actuator_type, material, compliance,
                   COUNT(*) as count,
                   AVG(grasp_success_rate) as avg_rate
            FROM experiments
            WHERE task = ? AND success = 1
            GROUP BY gesture, num_fingers, actuator_type, material, compliance
            ORDER BY avg_rate DESC, count DESC
            LIMIT 5
        """, (task,))

        recommendations = []
        for row in cursor.fetchall():
            recommendations.append({
                "gesture": row[0],
                "num_fingers": row[1],
                "actuator_type": row[2],
                "material": row[3],
                "compliance": row[4],
                "experiment_count": row[5],
                "avg_success_rate": row[6]
            })

        conn.close()

        return {
            "task": task,
            "recommendations": recommendations
        }

    def get_failure_patterns(self) -> Dict:
        """Identify common failure patterns."""
        conn = sqlite3.connect(self.db_path)

        cursor = conn.execute("""
            SELECT gesture, material, environment, COUNT(*) as count
            FROM experiments
            WHERE slip_occurred = 1
            GROUP BY gesture, material, environment
            ORDER BY count DESC
            LIMIT 10
        """)

        slip_patterns = [
            {"gesture": r[0], "material": r[1], "environment": r[2], "count": r[3]}
            for r in cursor.fetchall()
        ]

        cursor = conn.execute("""
            SELECT gesture, actuator_type, task, COUNT(*) as count
            FROM experiments
            WHERE damage_occurred = 1
            GROUP BY gesture, actuator_type, task
            ORDER BY count DESC
            LIMIT 10
        """)

        damage_patterns = [
            {"gesture": r[0], "actuator": r[1], "task": r[2], "count": r[3]}
            for r in cursor.fetchall()
        ]

        conn.close()

        return {
            "slip_patterns": slip_patterns,
            "damage_patterns": damage_patterns
        }

    def export_for_training(self, output_path: str) -> int:
        """Export all data as JSON for ML training."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.execute("SELECT * FROM experiments")

        data = []
        for row in cursor.fetchall():
            result = self._row_to_result(row)
            data.append(asdict(result))

        conn.close()

        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(data, f, indent=2)

        return len(data)

    def get_summary(self) -> Dict:
        """Get overall calibration summary."""
        conn = sqlite3.connect(self.db_path)

        cursor = conn.execute("""
            SELECT COUNT(*) as total,
                   SUM(success) as successes,
                   COUNT(DISTINCT design_id) as unique_designs,
                   COUNT(DISTINCT task) as unique_tasks
            FROM experiments
        """)

        row = cursor.fetchone()
        conn.close()

        total, successes, designs, tasks = row

        return {
            "total_experiments": total or 0,
            "successes": successes or 0,
            "success_rate": (successes / total * 100) if total else 0,
            "unique_designs_tested": designs or 0,
            "unique_tasks": tasks or 0
        }

    def _row_to_result(self, row) -> ExperimentResult:
        """Convert database row to ExperimentResult."""
        return ExperimentResult(
            id=row[0],
            design_id=row[1],
            timestamp=row[2],
            success=bool(row[3]),
            task=row[4],
            environment=row[5],
            grasp_success_rate=row[6],
            damage_occurred=bool(row[7]) if row[7] is not None else None,
            slip_occurred=bool(row[8]) if row[8] is not None else None,
            cycle_count=row[9],
            gesture=row[10],
            num_fingers=row[11],
            actuator_type=row[12],
            material=row[13],
            compliance=row[14],
            notes=row[15],
            experimenter=row[16]
        )


# ════════════════════════════════════════════════════════════════════════════════
# CLI
# ════════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Calibration System")
    parser.add_argument("command", choices=["summary", "log", "stats", "export", "recommend"])
    parser.add_argument("--design", "-d", help="Design ID")
    parser.add_argument("--success", "-s", action="store_true", help="Mark as success")
    parser.add_argument("--fail", "-f", action="store_true", help="Mark as failure")
    parser.add_argument("--task", "-t", default="general", help="Task name")
    parser.add_argument("--notes", "-n", help="Notes")
    parser.add_argument("--output", "-o", help="Output path")

    args = parser.parse_args()

    cal = CalibrationSystem()

    if args.command == "summary":
        summary = cal.get_summary()
        print("\n📊 CALIBRATION SUMMARY")
        print("=" * 40)
        print(f"  Total experiments:    {summary['total_experiments']}")
        print(f"  Successes:            {summary['successes']}")
        print(f"  Success rate:         {summary['success_rate']:.1f}%")
        print(f"  Unique designs:       {summary['unique_designs_tested']}")
        print(f"  Unique tasks:         {summary['unique_tasks']}")
        print("=" * 40)

    elif args.command == "log":
        if not args.design:
            print("Error: --design required")
            exit(1)

        success = args.success or (not args.fail)
        exp_id = cal.log_quick(args.design, success, args.task, args.notes)
        status = "✅ SUCCESS" if success else "❌ FAILURE"
        print(f"Logged: {exp_id} - {args.design} - {status}")

    elif args.command == "stats":
        print("\n📊 STATS BY GESTURE")
        print("-" * 40)
        for gesture, stats in cal.get_stats_by_gesture().items():
            print(f"  {gesture}: {stats['success_rate']*100:.0f}% ({stats['total_experiments']} experiments)")

        print("\n📊 STATS BY TASK")
        print("-" * 40)
        for task, stats in cal.get_stats_by_task().items():
            print(f"  {task}: {stats['success_rate']*100:.0f}% ({stats['total_experiments']} experiments)")

    elif args.command == "export":
        output = args.output or "calibration_data.json"
        count = cal.export_for_training(output)
        print(f"✓ Exported {count} experiments to {output}")

    elif args.command == "recommend":
        if not args.task:
            print("Error: --task required")
            exit(1)

        recs = cal.get_best_params_for_task(args.task)
        print(f"\n🎯 RECOMMENDATIONS FOR: {args.task}")
        print("=" * 50)

        if recs["recommendations"]:
            for i, r in enumerate(recs["recommendations"], 1):
                avg = r.get("avg_success_rate")
                pct = (avg * 100) if avg is not None else 0
                print(f"\n  {i}. {r['gesture']} gripper")
                print(f"     Fingers: {r['num_fingers']}, Actuator: {r['actuator_type']}")
                print(f"     Success rate: {pct:.0f}% ({r['experiment_count']} tests)")
        else:
            print("  No data yet for this task. Run some experiments!")
