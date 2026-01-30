#!/usr/bin/env python3
"""
OMEGA Weekly Review Generator
=============================

Auto-generates weekly/monthly/quarterly review from active hypotheses and substrate.

Agenda includes:
- Hypotheses updated this week
- Hypotheses stale (no update in 2+ weeks)
- Related concepts from knowledge graph
- Suggested next actions
- Risks/blockers
- Kill candidates, strengthened/weakened

Usage:
    python weekly_review.py generate
    python weekly_review.py generate --output review.md
    python weekly_review.py generate --period monthly
    python weekly_review.py generate --period quarterly
"""

import sys
from pathlib import Path
from datetime import datetime, timedelta
from typing import List, Optional

_repo_root = Path(__file__).resolve().parent.parent.parent
_breakthrough_dir = Path(__file__).resolve().parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))
if str(_breakthrough_dir) not in sys.path:
    sys.path.insert(0, str(_breakthrough_dir))

try:
    from hypothesis_ledger import HypothesisLedger, HypothesisStatus
except ImportError:
    from products.breakthrough_engine.hypothesis_ledger import HypothesisLedger, HypothesisStatus

try:
    from substrate_integration import get_related_concepts_from_graph
except ImportError:
    get_related_concepts_from_graph = lambda h: []


class WeeklyReviewGenerator:
    """Generates weekly/monthly/quarterly review agendas with substrate context."""

    def __init__(self):
        self.ledger = HypothesisLedger()
        self.review_period_days = 7

    def generate(self, period: str = "weekly") -> str:
        """Generate weekly/monthly/quarterly review agenda. period: weekly, monthly, quarterly."""
        now = datetime.now()
        if period == "quarterly":
            self.review_period_days = 90
        elif period == "monthly":
            self.review_period_days = 30
        else:
            self.review_period_days = 7
        period_start = now - timedelta(days=self.review_period_days)

        all_hypotheses = self.ledger.list()

        strengthened = [h for h in all_hypotheses if h.status == HypothesisStatus.STRENGTHENED]
        weakened = [h for h in all_hypotheses if h.status == HypothesisStatus.WEAKENED]
        active = [h for h in all_hypotheses if h.status == HypothesisStatus.ACTIVE]
        killed = [h for h in all_hypotheses if h.status == HypothesisStatus.KILLED]
        falsified = [h for h in all_hypotheses if h.status == HypothesisStatus.FALSIFIED]

        kill_candidates = [h for h in active if h.confidence < 0.3]

        two_weeks_ago = (now - timedelta(days=14)).isoformat()
        stale = [h for h in active if h.updated_at < two_weeks_ago]
        updated_this_period = [h for h in all_hypotheses if h.updated_at >= period_start.isoformat()]

        related_concepts = []
        try:
            for h in active[:10]:
                rel = get_related_concepts_from_graph(h.id)
                for r in rel:
                    nid = r.get("node_id") or (r.get("node") or {}).get("id", "")
                    if nid and nid not in related_concepts:
                        related_concepts.append(nid)
        except Exception:
            pass

        agenda = f"""
# OMEGA {period.title()} Review
**Generated:** {now.strftime('%Y-%m-%d %H:%M')}
**Period:** {period_start.strftime('%Y-%m-%d')} to {now.strftime('%Y-%m-%d')}

---

## 📊 Summary

| Category | Count |
|----------|-------|
| Total Hypotheses | {len(all_hypotheses)} |
| Active | {len(active)} |
| Updated this period | {len(updated_this_period)} |
| Stale (2+ weeks) | {len(stale)} |
| Strengthened | {len(strengthened)} |
| Weakened | {len(weakened)} |
| Killed | {len(killed)} |
| Falsified | {len(falsified)} |

---

## 📅 Hypotheses Updated This Period

"""
        if updated_this_period:
            for h in updated_this_period[:10]:
                agenda += f"- **{h.id}:** {h.claim[:50]}… (updated {h.updated_at[:10]})\n"
            agenda += "\n"
        else:
            agenda += "*None*\n\n"

        agenda += """
---

## ✅ Hypotheses That Gained Support

"""
        if strengthened:
            for h in strengthened[:5]:
                agenda += f"### {h.id}: {h.claim[:60]}\n"
                agenda += f"- **Confidence:** {h.confidence:.0%}\n"
                agenda += f"- **Evidence:** {len(h.evidence)} items\n\n"
        else:
            agenda += "*None this week*\n\n"

        agenda += """
---

## ⚠️ Hypotheses That Weakened

"""
        if weakened:
            for h in weakened[:5]:
                agenda += f"### {h.id}: {h.claim[:60]}\n"
                agenda += f"- **Confidence:** {h.confidence:.0%}\n"
                agenda += f"- **Consider:** Review evidence or kill\n\n"
        else:
            agenda += "*None this week*\n\n"

        agenda += """
---

## 🔪 Kill Candidates (Confidence < 30%)

**Key rule:** Killing a hypothesis is a win, not a loss.

"""
        if kill_candidates:
            for h in kill_candidates:
                agenda += f"- **{h.id}:** {h.claim[:50]}... ({h.confidence:.0%})\n"
            agenda += "\n**Action:** Vote to kill or provide new evidence.\n"
        else:
            agenda += "*None - all active hypotheses above threshold*\n"

        agenda += """
---

## 😴 Stale Hypotheses (No update in 2 weeks)

"""
        if stale:
            for h in stale:
                agenda += f"- **{h.id}:** {h.claim[:50]}... (last update: {h.updated_at[:10]})\n"
            agenda += "\n**Action:** Update, design experiment, or pause.\n"
        else:
            agenda += "*None - all hypotheses recently updated*\n"

        agenda += """
---

## 🧪 Experiments to Design

Based on active hypotheses, consider experiments that would:

1. **Discriminate** between competing hypotheses
2. **Falsify** low-confidence hypotheses cheaply
3. **Strengthen** promising hypotheses

"""
        for h in active[:3]:
            if h.falsifiers:
                agenda += f"### For {h.id}:\n"
                agenda += f"- Claim: {h.claim[:60]}\n"
                agenda += f"- Falsifiers to test: {', '.join(h.falsifiers[:2])}\n\n"

        agenda += """
---

## 🔗 Related Concepts (Knowledge Graph)

"""
        if related_concepts:
            agenda += ", ".join(related_concepts[:15]) + "\n\n"
        else:
            agenda += "*None linked yet — link hypotheses to concepts in substrate.*\n\n"

        agenda += """
---

## 🎯 Suggested Next Actions

1. **High confidence:** Translate strengthened hypotheses into experiments or decisions.
2. **Stale:** Assign owner or design next step for hypotheses with no update in 2+ weeks.
3. **Kill candidates:** Vote to kill or provide new evidence for hypotheses below 30% confidence.
4. **Risks/blockers:** Note any SRFC/RED or VRFC/RED that block progress.

---

## 🔴 Red Team Questions

- What are we overconfident about?
- Which assumptions would hurt most if false?
- Where are incentives biasing interpretation?

---

## 📋 Action Items

1. [ ] Review strengthened hypotheses - any ready for translation?
2. [ ] Vote on kill candidates
3. [ ] Assign owners to stale hypotheses
4. [ ] Design next period's experiments

---

*Generated by OMEGA Breakthrough Engine*
"""

        return agenda

    def save(self, output_path: str = None, period: str = "weekly") -> str:
        """Generate and save the review."""
        agenda = self.generate(period=period)

        if output_path is None:
            output_path = f"{period}_review_{datetime.now().strftime('%Y%m%d')}.md"

        Path(output_path).write_text(agenda, encoding="utf-8")
        return output_path


# CLI
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="OMEGA Weekly/Monthly/Quarterly Review Generator")
    parser.add_argument("command", nargs="?", default="generate", help="Command (generate)")
    parser.add_argument("--output", "-o", help="Output file path")
    parser.add_argument("--period", "-p", choices=["weekly", "monthly", "quarterly"], default="weekly", help="Review period")

    args = parser.parse_args()

    generator = WeeklyReviewGenerator()

    if args.command == "generate":
        agenda = generator.generate(period=getattr(args, "period", "weekly"))
        if args.output:
            Path(args.output).write_text(agenda, encoding="utf-8")
            print(f"Saved to: {args.output}")
        else:
            print(agenda)
