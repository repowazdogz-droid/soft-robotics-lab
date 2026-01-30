#!/usr/bin/env python3
"""
OMEGA-MAX ΦΩ η-CIV Decision Brief
=================================

Deterministic core: no LLM guessing for analysis. LLM only for narrative layer.
- Temporal manifold: T1 (0-1y), T2 (1-5y), T3 (5-30y), T4 (30+y)
- Substrate analysis: materials, compute, manufacturing, coordination, environment
- Plural futures: 3+ trajectories with trade-offs
- Validation Trinity: SRFC (feasibility), TSRFC (workflow), VRFC (reality)

Usage:
    from decision_brief import generate_brief
    brief = generate_brief("Should we dual-source suppliers?", params={"budget": 1e6})
    brief.to_markdown("decision.md")
    brief.to_json("decision.json")
"""

import sys
import json
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict, field
from typing import List, Dict, Optional, Any

_PRODUCTS = Path(__file__).resolve().parent.parent.parent
_DECISION_BRIEF_DIR = Path(__file__).resolve().parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
if str(_DECISION_BRIEF_DIR) not in sys.path:
    sys.path.insert(0, str(_DECISION_BRIEF_DIR))
from shared.id_generator import decision_id

_repo_root = Path(__file__).resolve().parent.parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

try:
    from domains import load_domain_model, list_domains
except ImportError:
    load_domain_model = lambda d: {}
    list_domains = lambda: []

try:
    from substrate_integration import search_past_decisions, get_related_knowledge, record_decision
except ImportError:
    search_past_decisions = lambda q, n=5: []
    get_related_knowledge = lambda q, d=None, n=5: []
    record_decision = lambda *a, **k: None

try:
    from omega_sim import Omega
except ImportError:
    Omega = None

try:
    from omega_cross_domain import CrossDomainReasoner
except ImportError:
    CrossDomainReasoner = None

# Deterministic constants
SUBSTRATES = ["materials", "compute", "manufacturing", "coordination", "environment"]
TEMPORAL_LABELS = {"t1": "T1 (Now-3mo)", "t2": "T2 (3-12mo)", "t3": "T3 (1-5yr)", "t4": "T4 (5+yr)"}
STATUS_ORDER = {"RED": 3, "AMBER": 2, "GREEN": 1}

# LLM response cache (key -> parsed JSON)
_llm_cache: Dict[str, Any] = {}


@dataclass
class Trajectory:
    """One of 3+ plural futures."""
    name: str
    description: str
    probability: float  # 0-1
    trade_offs: List[str]
    failure_modes: List[str]


@dataclass
class DecisionBrief:
    """OMEGA-MAX ΦΩ η-CIV decision brief. Deterministic core."""
    # Identity
    id: str
    query: str
    generated_at: str
    params: Dict = field(default_factory=dict)
    domains: List[str] = field(default_factory=list)

    # Temporal (T1-T4)
    t1_implications: Dict = field(default_factory=dict)  # implications, risks, opportunities
    t2_implications: Dict = field(default_factory=dict)
    t3_implications: Dict = field(default_factory=dict)
    t4_implications: Dict = field(default_factory=dict)

    # Stakeholders
    who_pays: List[str] = field(default_factory=list)
    who_benefits: List[str] = field(default_factory=list)
    who_loses: List[str] = field(default_factory=list)
    who_decides: List[str] = field(default_factory=list)  # with authority level
    hidden_stakeholders: List[str] = field(default_factory=list)
    adoption_barriers: List[str] = field(default_factory=list)

    # Validation Trinity (deterministic)
    srfc_status: str = "AMBER"  # GREEN/AMBER/RED - Can it work?
    srfc_reason: str = ""
    tsrfc_status: str = "AMBER"  # What does it replace?
    tsrfc_reason: str = ""
    vrfc_status: str = "AMBER"  # Will it survive reality?
    vrfc_reason: str = ""

    # Plural futures (min 3)
    trajectories: List[Trajectory] = field(default_factory=list)

    # Substrate
    substrate_analysis: Dict = field(default_factory=dict)

    # Output (deterministic)
    overall_status: str = "AMBER"  # GREEN/AMBER/RED
    recommendation_one_line: str = ""  # [GREEN/AMBER/RED] one-line recommendation
    confidence_drivers: List[str] = field(default_factory=list)
    uncertainty_drivers: List[str] = field(default_factory=list)
    recommended_next_action: str = ""
    requires_approval: bool = True
    approval_reason: Optional[str] = None

    # Scenarios (optimistic, realistic, pessimistic)
    scenarios: List[Dict[str, Any]] = field(default_factory=list)  # name, probability, key_drivers, implications, signals_to_watch

    # Substrate / related context
    related_past_decisions: List[Dict[str, Any]] = field(default_factory=list)  # [{id, text, score}]
    related_knowledge: List[str] = field(default_factory=list)

    # Audit
    audit_trail: List[str] = field(default_factory=list)
    params_complete: bool = False
    missing_params: List[str] = field(default_factory=list)

    # LLM enrichment (optional layer on top of deterministic core)
    llm_enriched_temporal: bool = False
    llm_enriched_stakeholders: bool = False
    llm_enriched_substrate: bool = False
    llm_enriched_recommended_action: bool = False
    llm_unavailable_note: str = ""  # set when LLM fails; show in output

    def to_dict(self) -> Dict:
        return asdict(self)

    def to_json(self, path: Optional[str] = None) -> str:
        data = json.dumps(self.to_dict(), indent=2, default=str)
        if path:
            Path(path).write_text(data, encoding="utf-8")
        return data

    def to_markdown(self, path: Optional[str] = None) -> str:
        def _horizon_table(t1: Dict, t2: Dict, t3: Dict, t4: Dict) -> str:
            rows = []
            for key, label in list(TEMPORAL_LABELS.items()):
                d = {"t1": t1, "t2": t2, "t3": t3, "t4": t4}.get(key, {})
                imp = d.get("implications", "—")
                risks = d.get("risks", "—")
                opps = d.get("opportunities", "—")
                if isinstance(imp, list):
                    imp = "; ".join(str(x) for x in imp[:3])
                if isinstance(risks, list):
                    risks = "; ".join(str(x) for x in risks[:3])
                if isinstance(opps, list):
                    opps = "; ".join(str(x) for x in opps[:3])
                rows.append(f"| {label} | {imp} | {risks} | {opps} |")
            return "\n".join(rows)

        _te = " [E]" if getattr(self, "llm_enriched_temporal", False) else " [D]"
        _se = " [E]" if getattr(self, "llm_enriched_stakeholders", False) else " [D]"
        _sbe = " [E]" if getattr(self, "llm_enriched_substrate", False) else " [D]"
        _re = " [E]" if getattr(self, "llm_enriched_recommended_action", False) else " [D]"
        _note = getattr(self, "llm_unavailable_note", "") or ""

        md = f"""# DECISION BRIEF
==============
**ID:** {self.id}
**Question:** {self.query}
**Domain:** {", ".join(self.domains) if self.domains else "general"}
**Generated:** {self.generated_at}

## RECOMMENDATION
--------------
{getattr(self, 'recommendation_one_line', '') or f'[{self.overall_status}] ' + (self.recommended_next_action or '—')}
"""
        if _note:
            md += f"\n> **Note:** {_note}\n"
        md += """
## TEMPORAL ANALYSIS
-----------------
| Horizon | Implications | Risks | Opportunities |
|---------|--------------|-------|----------------|
"""
        md += _horizon_table(
            self.t1_implications, self.t2_implications,
            self.t3_implications, self.t4_implications,
        )

        md += """

## STAKEHOLDERS
------------
- **Pays:** """ + ("; ".join(self.who_pays) if self.who_pays else "—")
        md += """
- **Benefits:** """ + ("; ".join(self.who_benefits) if self.who_benefits else "—")
        md += """
- **Loses:** """ + ("; ".join(self.who_loses) if self.who_loses else "—")
        who_dec = getattr(self, "who_decides", None) or []
        md += """
- **Decides:** """ + ("; ".join(who_dec) if who_dec else "—")
        hidden = getattr(self, "hidden_stakeholders", None) or []
        md += """
- **Hidden:** """ + ("; ".join(hidden) if hidden else "—")

        scenarios = getattr(self, "scenarios", None) or []
        if scenarios:
            md += """

## SCENARIOS
---------
"""
            for s in scenarios:
                pct = int((s.get("probability") or 0) * 100)
                md += f"- **{s.get('name', '')} ({pct}%):** {s.get('description', '')}\n"
                md += f"  - Key drivers: {', '.join(s.get('key_drivers', [])[:3])}\n"
                md += f"  - Signals to watch: {', '.join(s.get('signals_to_watch', [])[:3])}\n\n"
        md += """
## RELATED CONTEXT
---------------
- **Past decisions:** """ + (", ".join([r.get("id", r.get("text", ""))[:50] for r in (getattr(self, "related_past_decisions", None) or [])[:5]]) or "—")
        md += """
- **Related knowledge:** """ + (", ".join(getattr(self, "related_knowledge", None) or []) or "—")

        md += """

## VALIDATION
----------
- **SRFC:** """ + f"{self.srfc_status} - {self.srfc_reason or 'Can it work?'}"
        md += """
- **TSRFC:** """ + f"{self.tsrfc_status} - {self.tsrfc_reason or 'What does it replace?'}"
        md += """
- **VRFC:** """ + f"{self.vrfc_status} - {self.vrfc_reason or 'Will it survive reality?'}"

        md += """

## Plural Futures (trajectories)
"""
        for i, t in enumerate(self.trajectories, 1):
            md += f"{i}. **{t.name}:** {t.description}\n"
            md += f"   - Probability: {t.probability*100:.0f}%\n"
            md += f"   - Trade-offs: {', '.join(t.trade_offs[:3])}\n"
            md += f"   - Failure modes: {', '.join(t.failure_modes[:3])}\n\n"

        md += """---

### Substrate Interactions""" + _sbe + """

"""
        for k, v in self.substrate_analysis.items():
            md += f"- **{k}:** {v}\n"

        md += """
## AUDIT
-----
- **Lineage:** This decision depends on inputs (query, params, domains).
"""
        md += "\n### Recommended Action\n\n" + (self.recommended_next_action or "No concrete action—gather required params and re-run.")

        md += "\n**What we know:**\n"
        for c in self.confidence_drivers:
            md += f"- {c}\n"
        md += "**What we don't know:**\n"
        for u in self.uncertainty_drivers:
            md += f"- {u}\n"
        md += f"\n**Requires approval:** {'Yes' if self.requires_approval else 'No'}"
        if self.approval_reason:
            md += f" — {self.approval_reason}\n"
        md += "\n**Audit trail:**\n"
        for step in self.audit_trail:
            md += f"1. {step}\n"
        md += "\n---\n*OMEGA-MAX ΦΩ η-CIV Decision Brief. Domain-aware; substrate-integrated.*\n"
        if path:
            Path(path).write_text(md, encoding="utf-8")
        return md


def _detect_domains(query: str) -> List[str]:
    """Detect domains from query. Comparison -> strategy or multiple domains."""
    q = query.lower()
    domain_keywords = {
        "supply_chain": ["supplier", "supply", "procurement", "logistics", "inventory"],
        "spine_surgery": ["spine", "surgery", "surgical", "patient", "clinical"],
        "drug_discovery": ["drug", "compound", "trial", "pharma", "molecule"],
        "epidemic": ["disease", "outbreak", "infection", "spread", "pandemic"],
        "economics": ["market", "price", "economic", "trade", "financial"],
        "robotics": ["robot", "robotics", "gripper", "manipulation", "actuator", "arm", "soft robotics", "sim-to-real", "fabrication"],
        "synthetic_biology": ["gene", "pathway", "strain", "synbio", "biosensor", "fermentation", "plasmid", "crispr"],
        "research": ["hypothesis", "experiment", "publication", "grant", "lab", "paper", "reproducibility"],
        "enterprise": ["enterprise", "tools", "software", "platform", "saas", "product"],
        "business": ["pricing", "hiring", "market entry", "partnership", "prioritize", "q1", "roadmap", "budget"],
    }
    scores = {d: 0 for d in domain_keywords}
    for domain, keywords in domain_keywords.items():
        for kw in keywords:
            if kw in q:
                scores[domain] += 1
    detected = [d for d, s in scores.items() if s > 0]
    is_comparison = any(p in q for p in (" or ", " vs ", " versus ", " prioritize ", " compare ", " prioritiz"))
    if is_comparison and len(detected) >= 2:
        return ["strategy"]
    if len(detected) >= 2:
        return ["strategy"]
    if detected:
        return detected
    return ["general"]


def _detect_domain_for_model(domains: List[str], query: str) -> str:
    """Map detected domains to domain model id: robotics, synthetic_biology, research, business."""
    q = query.lower()
    if "robotics" in domains or any(x in q for x in ["gripper", "actuator", "sim-to-real", "fabrication"]):
        return "robotics"
    if "synthetic_biology" in domains or any(x in q for x in ["gene", "pathway", "strain", "synbio", "biosensor"]):
        return "synthetic_biology"
    if "research" in domains or any(x in q for x in ["hypothesis", "experiment", "publication", "grant"]):
        return "research"
    if "business" in domains or "enterprise" in domains or "strategy" in domains or "supply_chain" in domains:
        return "business"
    return "business"


def _required_params_for_domains(domains: List[str]) -> List[str]:
    """Deterministic: required param keys per domain (stub; extend as needed)."""
    required = []
    if "strategy" in domains or "general" in domains:
        required = ["horizon", "budget"]  # example
    if "supply_chain" in domains:
        required = list(set(required) | {"suppliers", "lead_time"})
    if "robotics" in domains:
        required = list(set(required) | {"capability", "environment"})
    if "enterprise" in domains:
        required = list(set(required) | {"users", "integration"})
    return required if required else ["horizon"]


def _check_params_complete(params: Dict, domains: List[str]) -> tuple:
    """Return (params_complete: bool, missing_params: List[str])."""
    required = _required_params_for_domains(domains)
    missing = [k for k in required if not (params and params.get(k) is not None)]
    return (len(missing) == 0, missing)


def _aggregate_status(statuses: List[str]) -> str:
    """RED > AMBER > GREEN. Any RED -> overall RED."""
    order = {"RED": 3, "AMBER": 2, "GREEN": 1}
    if not statuses:
        return "AMBER"
    if any(s == "RED" for s in statuses):
        return "RED"
    if any(s == "AMBER" for s in statuses):
        return "AMBER"
    return "GREEN"


def analyze_temporal(decision: str, domain: str, horizon: int = 4) -> Dict[str, Dict[str, Any]]:
    """
    Sharper temporal analysis. T1 (0-3mo), T2 (3-12mo), T3 (1-5y), T4 (5+y).
    Uses domain model to inform what matters at each horizon.
    Returns dict with t1, t2, t3, t4; each value has actions, blockers, milestones as appropriate.
    """
    model = load_domain_model(domain) if domain else {}
    horizons_desc = (model.get("time_horizons") or {}).copy()
    q = decision.lower()
    out = {}
    # T1: Immediate actions, quick wins, blockers
    t1_actions = ["Lock scope and success criteria", "Identify quick wins and first deliverables"]
    if "prioritize" in q or "strategy" in domain:
        t1_actions = ["Portfolio allocation", "Quick wins and 90-day checkpoint", "Resource commit"]
    if domain == "robotics":
        t1_actions = ["Prototype or bench test", "Material/supplier shortlist", "Sim-to-real pilot"]
    if domain == "synthetic_biology":
        t1_actions = ["Construct design and cloning", "First assays", "Strain shortlist"]
    if domain == "research":
        t1_actions = ["Protocol lock", "Pilot data", "First submissions or grant draft"]
    out["t1"] = {"implications": t1_actions, "risks": ["Resource or timeline slip"], "opportunities": ["Early signal on feasibility"]}
    # T2: Medium-term milestones, dependencies
    t2_actions = ["Milestone delivery", "Stakeholder alignment", "Go/no-go checkpoint"]
    if domain == "robotics":
        t2_actions = ["Pilot builds", "Sim-to-real validation", "Design freeze"]
    if domain == "business":
        t2_actions = ["Product milestones", "First revenue or LOIs", "Key hires"]
    out["t2"] = {"implications": t2_actions, "risks": ["Scope creep", "Dependency delay"], "opportunities": ["Proof of value"]}
    # T3: Strategic positioning, capability building
    t3_actions = ["Scale or expand", "Capability build", "Optionality (partnership, raise)"]
    out["t3"] = {"implications": t3_actions, "risks": ["Market shift", "Competitive response"], "opportunities": ["Category position"]}
    # T4: Long-term bets, existential risks
    t4_actions = ["Platform or category leadership", "Exit or sustainable growth", "Next-generation bets"]
    out["t4"] = {"implications": t4_actions, "risks": ["Existential disruption"], "opportunities": ["Durable advantage"]}
    return out


def _compute_temporal_implications(query: str, params: Dict, domains: List[str], complete: bool) -> tuple:
    """T1-T4 using domain-aware analyze_temporal when possible. Returns (t1, t2, t3, t4) dicts."""
    domain_id = _detect_domain_for_model(domains, query)
    temp = analyze_temporal(query, domain_id, 4)
    t1 = temp.get("t1", {"implications": ["Immediate actions and quick wins"], "risks": [], "opportunities": []})
    t2 = temp.get("t2", {"implications": ["Medium-term milestones"], "risks": [], "opportunities": []})
    t3 = temp.get("t3", {"implications": ["Strategic positioning"], "risks": [], "opportunities": []})
    t4 = temp.get("t4", {"implications": ["Long-term considerations"], "risks": [], "opportunities": []})
    if not complete:
        t1["risks"] = list(t1.get("risks", [])) + ["INCOMPLETE - params missing; implications tentative"]
    return (t1, t2, t3, t4)


def analyze_stakeholders(decision: str, domain: str) -> Dict[str, Any]:
    """
    Stakeholder analysis with who_pays, who_benefits, who_loses, who_decides, hidden_stakeholders.
    Each list item can be "Role: reason" or "Role (authority level)".
    """
    model = load_domain_model(domain) if domain else {}
    stakeholders = model.get("stakeholders") or ["Leadership", "Ops", "End user"]
    q = decision.lower()
    who_pays = ["Primary budget owner (funds execution)"]
    who_benefits = ["Decision sponsor (owns outcome)", "End users if adopted"]
    who_loses = ["Alternatives foregone (opportunity cost)"]
    who_decides = ["Decision maker (final authority)", "Sponsor (recommendation)"]
    hidden_stakeholders = ["Regulators (often overlooked until late)", "Adjacent teams (dependencies)"]
    if domain == "robotics":
        who_pays = ["R&D / lab (prototype cost)", "Manufacturing / ops (production cost)"]
        who_benefits = ["End customer / integrator", "Quality / validation (clear spec)"]
        hidden_stakeholders = ["Certification bodies", "Supply chain (long lead items)"]
    if domain == "synthetic_biology":
        who_pays = ["Lab / PI (experiments)", "Fermentation / process (scale-up)"]
        who_decides = ["PI / lab head", "Regulatory / biosafety (containment)"]
        hidden_stakeholders = ["IP / legal (freedom-to-operate)", "Investors / partners"]
    if domain == "research":
        who_pays = ["Grant / funder", "Institution (overhead)"]
        who_decides = ["PI", "Collaborators (co-authorship)"]
        hidden_stakeholders = ["Tech transfer", "Reviewers / editors"]
    if domain == "business":
        who_pays = ["Leadership (budget)", "Finance / ops"]
        who_decides = ["Leadership (final)", "Board / investors (major bets)"]
        hidden_stakeholders = ["Board", "Key customers (reference)"]
    if "prioritize" in q or "strategy" in domain:
        who_pays.append("Strategy/PMO (resource allocation)")
        who_decides.append("Portfolio owner (priority)")
    return {"who_pays": who_pays, "who_benefits": who_benefits, "who_loses": who_loses, "who_decides": who_decides, "hidden_stakeholders": hidden_stakeholders}


def _compute_stakeholders(query: str, domains: List[str]) -> tuple:
    """Stakeholders using analyze_stakeholders when domain model available; plus adoption_barriers."""
    domain_id = _detect_domain_for_model(domains, query)
    sh = analyze_stakeholders(query, domain_id)
    barriers = ["Resource constraint", "Organizational alignment"]
    if "strategy" in domains:
        barriers.append("Portfolio trade-offs")
    return (
        sh.get("who_pays", ["Primary budget owner"]),
        sh.get("who_benefits", ["Decision sponsor", "End users if adopted"]),
        sh.get("who_loses", ["Alternatives foregone"]),
        sh.get("who_decides", ["Decision maker"]),
        sh.get("hidden_stakeholders", ["Regulators", "Adjacent teams"]),
        barriers,
    )


def _compute_validation_trinity(query: str, params: Dict, complete: bool, domains: List[str]) -> tuple:
    """Deterministic SRFC, TSRFC, VRFC. Returns (srfc, srfc_reason, tsrfc, tsrfc_reason, vrfc, vrfc_reason)."""
    if not complete:
        return (
            "AMBER", "Incomplete params - feasibility assessment deferred",
            "AMBER", "Workflow impact unclear without full params",
            "AMBER", "Adoption/regulation uncertain without full context",
        )
    srfc, tsrfc, vrfc = "GREEN", "AMBER", "AMBER"
    srfc_r = "Params sufficient for feasibility check"
    tsrfc_r = "Workflow replacement scope implied by query"
    vrfc_r = "Regulation and adoption depend on execution context"
    if "supply" in query.lower():
        tsrfc_r = "Supply chain workflow changes implied"
    if "strategy" in domains:
        vrfc_r = "Multi-domain strategy requires coordination validation"
    return (srfc, srfc_r, tsrfc, tsrfc_r, vrfc, vrfc_r)


def generate_scenarios(decision: str, domain: str) -> List[Dict[str, Any]]:
    """
    Plural futures: optimistic (20%), realistic (60%), pessimistic (20%).
    Each scenario: probability, key_drivers, implications, signals_to_watch.
    """
    model = load_domain_model(domain) if domain else {}
    risks = model.get("risks") or ["Execution", "Market", "Resource"]
    q = decision.lower()
    return [
        {
            "name": "Optimistic",
            "probability": 0.2,
            "description": "What if everything goes right?",
            "key_drivers": ["Strong execution", "Market tailwinds", "Key hires or partnerships"],
            "implications": ["Ahead of plan", "Optionality for expansion"],
            "signals_to_watch": ["Milestones hit on time", "Stakeholder NPS", "Pipeline conversion"],
        },
        {
            "name": "Realistic",
            "probability": 0.6,
            "description": "Most likely outcome.",
            "key_drivers": ["Mixed execution", "Some scope or timeline slip", "Stable demand"],
            "implications": ["Delivery with delay or scope trim", "Re-prioritization at checkpoints"],
            "signals_to_watch": ["Burn rate vs plan", "Go/no-go criteria", "Competitor moves"],
        },
        {
            "name": "Pessimistic",
            "probability": 0.2,
            "description": "What if key assumptions fail?",
            "key_drivers": [f"{r} materializes" for r in risks[:2]],  # domain risks
            "implications": ["Pivot or pause", "Resource reallocation", "Fallback plan"],
            "signals_to_watch": ["Early warning metrics", "Sponsor commitment", "Budget or regulatory change"],
        },
    ]


def _compute_trajectories(query: str, domains: List[str]) -> List[Trajectory]:
    """Deterministic: at least 3 trajectories with trade-offs and failure modes."""
    return [
        Trajectory(
            name="Conservative",
            description="Minimal change, focus on de-risking and reversible steps.",
            probability=0.4,
            trade_offs=["Lower upside", "Slower time-to-value", "Preserves optionality"],
            failure_modes=["Opportunity cost", "Competitor move first", "Drift from market"],
        ),
        Trajectory(
            name="Balanced",
            description="Phased commitment with checkpoints and clear go/no-go.",
            probability=0.35,
            trade_offs=["Moderate risk/reward", "Requires discipline", "Adaptable"],
            failure_modes=["Checkpoint creep", "Resource dilution", "Delayed payoff"],
        ),
        Trajectory(
            name="Aggressive",
            description="Full commitment to chosen direction; fast execution.",
            probability=0.25,
            trade_offs=["Higher upside", "Less reversible", "Requires alignment"],
            failure_modes=["Overcommit", "Execution gaps", "External shock"],
        ),
    ]


def _compute_substrate_analysis(query: str, domains: List[str]) -> Dict[str, str]:
    """Deterministic: effect per substrate (materials, compute, manufacturing, coordination, environment)."""
    q = query.lower()
    out = {}
    for s in SUBSTRATES:
        out[s] = f"Decision touches {s}; impact derived from query and domain."
    if "supply" in q:
        out["materials"] = "Supplier and material availability; dual-source affects material flow."
        out["coordination"] = "Procurement and operations coordination."
    if "robotics" in domains or "robot" in q:
        out["compute"] = "Compute for control/simulation; materials for end-effector."
        out["manufacturing"] = "Assembly and integration; tooling and fixtures."
    if "strategy" in domains:
        out["coordination"] = "Cross-domain coordination and resource allocation."
    return out


def _recommended_next_action(overall_status: str, complete: bool, missing: List[str]) -> str:
    """Concrete next step; never vague. No 80% on empty data."""
    if not complete and missing:
        return f"INCOMPLETE - provide params and re-run. Required: {', '.join(missing)}"
    if overall_status == "RED":
        return "Do not proceed until SRFC/TSRFC/VRFC are addressed; re-run with updated params."
    if overall_status == "AMBER":
        return "Proceed with phased pilot; set go/no-go checkpoint at 90 days and re-assess."
    return "Proceed to next phase per roadmap; document assumptions and review at next gate."


def _assess_approval(overall_status: str, srfc: str, tsrfc: str, vrfc: str) -> tuple:
    """Requires approval and reason (deterministic)."""
    if overall_status == "RED":
        return True, "Validation trinity contains RED; approval required before commitment."
    if srfc == "AMBER" or vrfc == "AMBER":
        return True, "Feasibility or reality check at AMBER; approval recommended."
    return False, None


def _call_llm_for_enrichment(query: str, params: Dict, brief_summary: str) -> Optional[Dict]:
    """
    Call LLM: try LM Studio (local) first, then Gemini, then Claude/OpenAI.
    Cache responses to avoid repeated calls. Graceful degradation: return None if all fail.
    """
    import os
    import hashlib
    cache_key = hashlib.sha256((query + json.dumps(params, default=str) + brief_summary[:500]).encode()).hexdigest()
    if cache_key in _llm_cache:
        return _llm_cache[cache_key]

    prompt = f"""You are enriching a deterministic decision brief with specific narrative. Do NOT change statuses or scores.

Query: {query}
Params: {json.dumps(params, default=str)}
Current brief summary:
{brief_summary[:1500]}

Respond with ONLY a valid JSON object (no markdown, no code fence) with these exact keys:
- "temporal": object with keys "t1", "t2", "t3", "t4". Each value is object with "implications" (array of strings), "risks" (array), "opportunities" (array). Be SPECIFIC to this decision.
- "stakeholders": object with "who_pays" (array of specific names/roles), "who_benefits" (array), "who_loses" (array). Name specific parties.
- "substrate": object with one key per substrate (materials, compute, manufacturing, coordination, environment). Each value is a short string.
- "recommended_action_steps": array of exactly 3 concrete, actionable steps (strings).
"""

    def _parse_json(text: str) -> Optional[Dict]:
        if not text:
            return None
        text = text.strip()
        if text.startswith("```"):
            parts = text.split("```")
            if len(parts) > 1:
                text = parts[1]
                if text.startswith("json"):
                    text = text[4:]
        try:
            return json.loads(text)
        except Exception:
            return None

    # 1. Try LM Studio (local) first
    try:
        from openai import OpenAI
        client = OpenAI(base_url="http://localhost:1234/v1", api_key="lm-studio")
        r = client.chat.completions.create(
            model="local-model",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=2048,
        )
        text = (r.choices[0].message.content or "").strip()
        out = _parse_json(text)
        if out:
            _llm_cache[cache_key] = out
            return out
    except Exception:
        pass

    # 2. Fall back to Gemini
    try:
        api_key = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
        if api_key:
            import google.generativeai as genai
            genai.configure(api_key=api_key)
            model = genai.GenerativeModel("gemini-1.5-flash")
            response = model.generate_content(prompt)
            if response and response.text:
                out = _parse_json(response.text.strip())
                if out:
                    _llm_cache[cache_key] = out
                    return out
    except Exception:
        pass

    # 3. Try Anthropic Claude
    try:
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if api_key:
            import anthropic
            client = anthropic.Anthropic(api_key=api_key)
            msg = client.messages.create(
                model="claude-3-5-haiku-20241022",
                max_tokens=2048,
                messages=[{"role": "user", "content": prompt}],
            )
            text = msg.content[0].text if msg.content else ""
            out = _parse_json(text) if text else None
            if out:
                _llm_cache[cache_key] = out
                return out
    except Exception:
        pass

    # 4. Try OpenAI
    try:
        api_key = os.environ.get("OPENAI_API_KEY")
        if api_key:
            from openai import OpenAI
            client = OpenAI(api_key=api_key)
            r = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=2048,
            )
            text = (r.choices[0].message.content or "").strip()
            out = _parse_json(text) if text else None
            if out:
                _llm_cache[cache_key] = out
                return out
    except Exception:
        pass

    return None


def enrich_with_llm(brief: DecisionBrief, query: str, params: Dict) -> DecisionBrief:
    """
    Enrich deterministic brief with LLM narrative (temporal, stakeholders, substrate, 3-step action).
    Falls back gracefully if LLM unavailable: keep deterministic output, set llm_unavailable_note.
    """
    summary = (
        f"Status: {brief.overall_status}. Domains: {brief.domains}. "
        f"T1-T4: {brief.t1_implications}, {brief.t2_implications}, {brief.t3_implications}, {brief.t4_implications}. "
        f"Who pays: {brief.who_pays}. Who benefits: {brief.who_benefits}. Who loses: {brief.who_loses}. "
        f"Substrate: {brief.substrate_analysis}. Recommended: {brief.recommended_next_action}"
    )
    try:
        out = _call_llm_for_enrichment(query, params or {}, summary)
    except Exception:
        out = None
    if not out:
        brief.llm_unavailable_note = "LLM enrichment unavailable - showing deterministic analysis only"
        brief.audit_trail.append("LLM enrichment skipped (unavailable or error)")
        return brief

    # Apply enrichment
    def _to_list(v):
        return v if isinstance(v, list) else [v] if v else []

    if isinstance(out.get("temporal"), dict):
        for key, attr in [("t1", "t1_implications"), ("t2", "t2_implications"), ("t3", "t3_implications"), ("t4", "t4_implications")]:
            t = out["temporal"].get(key) or {}
            extra = {k: _to_list(t.get(k)) for k in ("implications", "risks", "opportunities") if t.get(k)}
            if extra:
                setattr(brief, attr, {**getattr(brief, attr), **extra})
        brief.llm_enriched_temporal = True

    if isinstance(out.get("stakeholders"), dict):
        s = out["stakeholders"]
        if s.get("who_pays"):
            brief.who_pays = s["who_pays"] if isinstance(s["who_pays"], list) else [s["who_pays"]]
            brief.llm_enriched_stakeholders = True
        if s.get("who_benefits"):
            brief.who_benefits = s["who_benefits"] if isinstance(s["who_benefits"], list) else [s["who_benefits"]]
        if s.get("who_loses"):
            brief.who_loses = s["who_loses"] if isinstance(s["who_loses"], list) else [s["who_loses"]]

    if isinstance(out.get("substrate"), dict) and out["substrate"]:
        brief.substrate_analysis = {**brief.substrate_analysis, **{k: str(v) for k, v in out["substrate"].items()}}
        brief.llm_enriched_substrate = True

    if isinstance(out.get("recommended_action_steps"), list) and len(out["recommended_action_steps"]) >= 1:
        steps = [str(s) for s in out["recommended_action_steps"][:3]]
        brief.recommended_next_action = "\n".join(f"{i}. {s}" for i, s in enumerate(steps, 1))
        brief.llm_enriched_recommended_action = True

    brief.audit_trail.append("LLM enrichment applied (temporal, stakeholders, substrate, recommended action)")
    return brief


class DecisionBriefGenerator:
    """OMEGA-MAX ΦΩ η-CIV. Deterministic core; no LLM for analysis."""

    def __init__(self):
        self.brief_count = 0
        self.omega = Omega() if Omega else None
        self.cross_domain = CrossDomainReasoner(omega_instance=self.omega) if CrossDomainReasoner and self.omega else None

    def generate(
        self,
        query: str,
        params: Optional[Dict] = None,
        temporal_focus: Optional[str] = None,
        substrate_focus: Optional[str] = None,
        context: Optional[Dict] = None,
    ) -> DecisionBrief:
        """Build brief deterministically from query and params."""
        context = context or {}
        params = params or context.get("params") or {}
        self.brief_count += 1
        audit = []

        # 1. Parse query, detect domains
        domains = _detect_domains(query)
        audit.append("Parsed query and detected domains: " + ", ".join(domains))

        # 2. Params completeness (deterministic)
        complete, missing = _check_params_complete(params, domains)
        if not complete:
            audit.append("Params INCOMPLETE - missing: " + ", ".join(missing))
        else:
            audit.append("Params complete for domain check")

        # 3. Temporal T1-T4
        t1, t2, t3, t4 = _compute_temporal_implications(query, params, domains, complete)
        audit.append("Computed T1-T4 implications (deterministic)")

        # 4. Stakeholders (with who_decides, hidden_stakeholders)
        who_pays, who_benefits, who_loses, who_decides, hidden_stakeholders, barriers = _compute_stakeholders(query, domains)
        audit.append("Computed stakeholders (deterministic)")

        # 4b. Scenarios (optimistic, realistic, pessimistic)
        domain_id = _detect_domain_for_model(domains, query)
        scenarios = generate_scenarios(query, domain_id)
        audit.append("Computed 3 scenarios (plural futures)")

        # 4c. Related context (substrate)
        related_past = []
        try:
            related_past = search_past_decisions(query, n=5)
        except Exception:
            pass
        related_knowledge_list = []
        try:
            related_knowledge_list = get_related_knowledge(query, domain_id, n=5)
        except Exception:
            pass

        # 5. Validation Trinity
        srfc, srfc_r, tsrfc, tsrfc_r, vrfc, vrfc_r = _compute_validation_trinity(query, params, complete, domains)
        audit.append("Computed SRFC/TSRFC/VRFC (deterministic)")

        # 6. Overall status: RED > AMBER > GREEN
        overall = _aggregate_status([srfc, tsrfc, vrfc])
        audit.append("Aggregated overall status: " + overall)

        # 7. Plural futures (min 3)
        trajectories = _compute_trajectories(query, domains)
        audit.append("Computed " + str(len(trajectories)) + " trajectories (plural futures)")

        # 8. Substrate
        substrate = _compute_substrate_analysis(query, domains)
        if substrate_focus and substrate_focus in substrate:
            substrate = {substrate_focus: substrate[substrate_focus]}
        audit.append("Computed substrate analysis")

        # 9. Confidence / uncertainty (deterministic; never hallucinate 80% on empty)
        if complete:
            confidence_drivers = ["Params provided and used for feasibility", "Domain mapping applied"]
            uncertainty_drivers = ["Execution context", "External factors", "Timeline variance"]
        else:
            confidence_drivers = []
            uncertainty_drivers = ["INCOMPLETE - requires: " + ", ".join(missing), "No confidence on empty params"]

        # 10. Recommended action (concrete) and one-line recommendation
        recommended = _recommended_next_action(overall, complete, missing)
        requires_approval, approval_reason = _assess_approval(overall, srfc, tsrfc, vrfc)
        recommendation_one_line = f"[{overall}] " + (recommended.split(".")[0] if recommended else "Proceed with phased pilot.")

        brief = DecisionBrief(
            id=decision_id(suffix=f"{self.brief_count:04d}"),
            query=query,
            generated_at=datetime.now().isoformat(),
            params=params,
            domains=domains,
            t1_implications=t1,
            t2_implications=t2,
            t3_implications=t3,
            t4_implications=t4,
            who_pays=who_pays,
            who_benefits=who_benefits,
            who_loses=who_loses,
            who_decides=who_decides,
            hidden_stakeholders=hidden_stakeholders,
            adoption_barriers=barriers,
            scenarios=scenarios,
            related_past_decisions=related_past,
            related_knowledge=related_knowledge_list,
            recommendation_one_line=recommendation_one_line,
            srfc_status=srfc,
            srfc_reason=srfc_r,
            tsrfc_status=tsrfc,
            tsrfc_reason=tsrfc_r,
            vrfc_status=vrfc,
            vrfc_reason=vrfc_r,
            trajectories=trajectories,
            substrate_analysis=substrate,
            overall_status=overall,
            confidence_drivers=confidence_drivers,
            uncertainty_drivers=uncertainty_drivers,
            recommended_next_action=recommended,
            requires_approval=requires_approval,
            approval_reason=approval_reason,
            audit_trail=audit,
            params_complete=complete,
            missing_params=missing,
        )
        # Record in substrate (lineage + vector store)
        try:
            record_decision(brief.id, query, domain_id, brief.query + " | " + brief.recommendation_one_line, inputs={"params": params, "domains": domains})
        except Exception:
            pass
        return brief


def generate_brief(
    query: str,
    params: Optional[Dict] = None,
    temporal_focus: Optional[str] = None,
    substrate_focus: Optional[str] = None,
    context: Optional[Dict] = None,
    enrich: bool = False,
) -> DecisionBrief:
    """Convenience: generate OMEGA-MAX decision brief. If enrich=True, add LLM narrative layer."""
    gen = DecisionBriefGenerator()
    brief = gen.generate(query, params=params, temporal_focus=temporal_focus, substrate_focus=substrate_focus, context=context)
    if enrich:
        brief = enrich_with_llm(brief, query, params or {})
    return brief


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="OMEGA-MAX ΦΩ η-CIV Decision Brief")
    parser.add_argument("query", nargs="?", default="", help="Decision query (or leave empty for stdin)")
    parser.add_argument("--output", "-o", default="decision_brief", help="Output filename (no extension)")
    parser.add_argument("--format", "-f", choices=["md", "json", "both"], default="both")
    parser.add_argument("--params", "-p", type=str, default=None, help="JSON params e.g. '{\"horizon\": 2, \"budget\": 1e6}'")
    parser.add_argument("--domain", "-d", type=str, default=None, help="Domain: robotics, synthetic_biology, research, business (auto-detect if omitted)")
    parser.add_argument("--temporal", "-t", choices=["t1", "t2", "t3", "t4"], default=None, help="Focus timescale")
    parser.add_argument("--substrate", "-s", choices=SUBSTRATES, default=None, help="Focus substrate")
    parser.add_argument("--enrich", "-e", action="store_true", help="Add LLM narrative layer (LM Studio → Gemini); deterministic only if omitted")
    parser.add_argument("--save", action="store_true", help="Always save to file (default: save when --output given)")

    args = parser.parse_args()
    query = (args.query or "").strip()
    if not query and sys.stdin.isatty() is False:
        query = sys.stdin.read().strip()
    if not query:
        parser.error("Provide a decision query as argument or via stdin.")
    params = json.loads(args.params) if args.params else {}

    # Progress (rich if available)
    try:
        from rich.console import Console
        from rich.panel import Panel
        from rich.progress import Progress, SpinnerColumn, TextColumn
        _rich = True
        console = Console()
    except ImportError:
        _rich = False

    if _rich:
        console.print(Panel(f"[bold]Decision:[/bold] {query[:80]}…" if len(query) > 80 else f"[bold]Decision:[/bold] {query}", title="OMEGA Decision Brief"))
        domains = _detect_domains(query)
        domain_id = _detect_domain_for_model(domains, query)
        console.print(f"[dim]Detected domain: {domain_id}[/dim]")
        with Progress(SpinnerColumn(), TextColumn("[progress.description]{task.description}"), console=console) as progress:
            task = progress.add_task("Generating brief…", total=None)
            brief = generate_brief(query, params=params, temporal_focus=args.temporal, substrate_focus=args.substrate, enrich=getattr(args, "enrich", False))
            progress.update(task, description="Done.")
    else:
        print("Generating OMEGA-MAX decision brief for:", query[:60] + ("…" if len(query) > 60 else ""))
        domains = _detect_domains(query)
        domain_id = _detect_domain_for_model(domains, query)
        print("Auto-detected domain:", domain_id)
        if getattr(args, "enrich", False):
            print("(With LLM enrichment)")
        brief = generate_brief(query, params=params, temporal_focus=args.temporal, substrate_focus=args.substrate, enrich=getattr(args, "enrich", False))

    do_save = args.save or args.output != "decision_brief"
    if do_save:
        if args.format in ["md", "both"]:
            md_path = f"{args.output}.md"
            brief.to_markdown(md_path)
            print("Markdown:", md_path) if not _rich else console.print(f"[green]Markdown:[/green] {md_path}")
        if args.format in ["json", "both"]:
            json_path = f"{args.output}.json"
            brief.to_json(json_path)
            print("JSON:", json_path) if not _rich else console.print(f"[green]JSON:[/green] {json_path}")

    if _rich:
        console.print()
        console.print(Panel(brief.to_markdown(), title=f"Brief {brief.id}", border_style="blue"))
    else:
        print("\n" + "=" * 60)
        print(brief.to_markdown())
