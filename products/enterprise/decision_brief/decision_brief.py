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
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.id_generator import decision_id

_repo_root = Path(__file__).resolve().parent.parent.parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

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
TEMPORAL_LABELS = {"t1": "T1 (0-1y)", "t2": "T2 (1-5y)", "t3": "T3 (5-30y)", "t4": "T4 (30+y)"}
STATUS_ORDER = {"RED": 3, "AMBER": 2, "GREEN": 1}


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
    confidence_drivers: List[str] = field(default_factory=list)
    uncertainty_drivers: List[str] = field(default_factory=list)
    recommended_next_action: str = ""
    requires_approval: bool = True
    approval_reason: Optional[str] = None

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
            for key, label in [("t1", "T1 (0-1y)"), ("t2", "T2 (1-5y)"), ("t3", "T3 (5-30y)"), ("t4", "T4 (30+y)")]:
                d = {"t1": t1, "t2": t2, "t3": t3, "t4": t4}[key]
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

        md = f"""# Decision Brief: {self.query}

## Status: {self.overall_status}

**Generated:** {self.generated_at}  
**Domains:** {", ".join(self.domains) if self.domains else "general"}
"""
        if _note:
            md += f"\n> **Note:** {_note}\n"
        md += """
---

### Temporal Analysis""" + _te + """

| Horizon | Implications | Risks | Opportunities |
|---------|--------------|-------|----------------|
"""
        md += _horizon_table(
            self.t1_implications, self.t2_implications,
            self.t3_implications, self.t4_implications,
        )

        md += """

---

### Validation Trinity [D]

- **SRFC (Feasibility):** """ + f"{self.srfc_status} - {self.srfc_reason or 'Can it work? (physics, feasibility)'}"
        md += """
- **TSRFC (Workflow):** """ + f"{self.tsrfc_status} - {self.tsrfc_reason or 'What does it replace? (workflow)'}"
        md += """
- **VRFC (Reality):** """ + f"{self.vrfc_status} - {self.vrfc_reason or 'Will it survive reality? (regulation, adoption)'}"

        md += """

---

### Stakeholders""" + _se + """

- **Who pays:** """ + ("; ".join(self.who_pays) if self.who_pays else "—")
        md += """
- **Who benefits:** """ + ("; ".join(self.who_benefits) if self.who_benefits else "—")
        md += """
- **Who loses:** """ + ("; ".join(self.who_loses) if self.who_loses else "—")
        md += """
- **Adoption barriers:** """ + ("; ".join(self.adoption_barriers) if self.adoption_barriers else "—")

        md += """

---

### Plural Futures [D]

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
---

### Recommended Action""" + _re + """

""" + (self.recommended_next_action or "No concrete action—gather required params and re-run.")

        md += """

---

### Confidence & Uncertainty [D]

**What we know:**
"""
        for c in self.confidence_drivers:
            md += f"- {c}\n"
        md += """
**What we don't know:**
"""
        for u in self.uncertainty_drivers:
            md += f"- {u}\n"

        md += f"""
---

### Approval [D]

- **Requires approval:** {"Yes" if self.requires_approval else "No"}
"""
        if self.approval_reason:
            md += f"- **Reason:** {self.approval_reason}\n"

        md += """
---

### Audit Trail [D]

"""
        for step in self.audit_trail:
            md += f"1. {step}\n"

        md += """
---
*OMEGA-MAX ΦΩ η-CIV Decision Brief. Deterministic core; narrative layer optional.*
"""
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
        "robotics": ["robot", "robotics", "gripper", "manipulation", "actuator", "arm", "soft robotics"],
        "enterprise": ["enterprise", "tools", "software", "platform", "saas", "product"],
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


def _compute_temporal_implications(query: str, params: Dict, domains: List[str], complete: bool) -> tuple:
    """Deterministic T1-T4 from query/domains. Returns (t1, t2, t3, t4) dicts."""
    q = query.lower()
    base = {
        "implications": ["Decision scope derived from query and domain"],
        "risks": ["Resource and timeline uncertainty"],
        "opportunities": ["Alignment with stated objectives"],
    }
    t1 = {**base, "implications": ["Prototypes, experiments, immediate costs (0-1y)"]}
    t2 = {**base, "implications": ["Products, markets, institutions (1-5y)"]}
    t3 = {**base, "implications": ["Infrastructure, ecosystems (5-30y)"]}
    t4 = {**base, "implications": ["Intergenerational, planetary (30+y)"]}
    if "supply" in q or "supply_chain" in domains:
        t1["implications"] = ["Supplier qualification, dual-source pilots"]
        t2["implications"] = ["Contract lock-in, capacity scaling"]
    if "prioritize" in q or "strategy" in domains:
        t1["implications"] = ["Portfolio allocation, quick wins"]
        t2["implications"] = ["Roadmap commitment, capability build"]
    if not complete:
        t1["risks"] = ["INCOMPLETE - params missing; implications tentative"]
    return (t1, t2, t3, t4)


def _compute_stakeholders(query: str, domains: List[str]) -> tuple:
    """Deterministic: who_pays, who_benefits, who_loses, adoption_barriers."""
    q = query.lower()
    who_pays = ["Primary budget owner"]
    who_benefits = ["Decision sponsor", "End users if adopted"]
    who_loses = ["Alternatives foregone"]
    barriers = ["Resource constraint", "Organizational alignment"]
    if "supply" in q:
        who_pays.append("Procurement")
        who_benefits.append("Operations")
    if "prioritize" in q or "strategy" in domains:
        who_pays.append("Strategy/PMO")
        barriers.append("Portfolio trade-offs")
    return (who_pays, who_benefits, who_loses, barriers)


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
    Call configured LLM (Gemini or Claude/OpenAI) to get enrichment JSON.
    Returns parsed dict with temporal, stakeholders, substrate, recommended_action_steps; or None on failure.
    """
    import os
    prompt = f"""You are enriching a deterministic decision brief with specific narrative. Do NOT change statuses or scores.

Query: {query}
Params: {json.dumps(params, default=str)}
Current brief summary:
{brief_summary}

Respond with ONLY a valid JSON object (no markdown, no code fence) with these exact keys:
- "temporal": object with keys "t1", "t2", "t3", "t4". Each value is object with "implications" (array of strings), "risks" (array), "opportunities" (array). Be SPECIFIC to this decision.
- "stakeholders": object with "who_pays" (array of specific names/roles), "who_benefits" (array), "who_loses" (array). Name specific parties (e.g. "Bristol Robotics Lab", "enterprise SaaS customers").
- "substrate": object with one key per substrate (materials, compute, manufacturing, coordination, environment). Each value is a short string explaining HOW this decision affects that substrate.
- "recommended_action_steps": array of exactly 3 concrete, actionable steps (strings).

Example shape: {{"temporal": {{"t1": {{"implications": [...], "risks": [...], "opportunities": [...]}}, ...}}, "stakeholders": {{...}}, "substrate": {{...}}, "recommended_action_steps": ["Step 1", "Step 2", "Step 3"]}}
"""

    # Try Gemini first
    try:
        api_key = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
        if api_key:
            import google.generativeai as genai
            genai.configure(api_key=api_key)
            model = genai.GenerativeModel("gemini-1.5-flash")
            response = model.generate_content(prompt)
            if response and response.text:
                text = response.text.strip()
                if text.startswith("```"):
                    text = text.split("```")[1]
                    if text.startswith("json"):
                        text = text[4:]
                return json.loads(text)
    except Exception:
        pass

    # Try Anthropic Claude
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
            if text:
                if text.startswith("```"):
                    text = text.split("```")[1]
                    if text.startswith("json"):
                        text = text[4:]
                return json.loads(text.strip())
    except Exception:
        pass

    # Try OpenAI
    try:
        api_key = os.environ.get("OPENAI_API_KEY")
        if api_key:
            import openai
            client = openai.OpenAI(api_key=api_key)
            r = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=2048,
            )
            text = (r.choices[0].message.content or "").strip()
            if text:
                if text.startswith("```"):
                    text = text.split("```")[1]
                    if text.startswith("json"):
                        text = text[4:]
                return json.loads(text.strip())
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

        # 4. Stakeholders
        who_pays, who_benefits, who_loses, barriers = _compute_stakeholders(query, domains)
        audit.append("Computed stakeholders (deterministic)")

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

        # 10. Recommended action (concrete)
        recommended = _recommended_next_action(overall, complete, missing)
        requires_approval, approval_reason = _assess_approval(overall, srfc, tsrfc, vrfc)

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
            adoption_barriers=barriers,
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
    parser.add_argument("query", help="Decision query")
    parser.add_argument("--output", "-o", default="decision_brief", help="Output filename (no extension)")
    parser.add_argument("--format", "-f", choices=["md", "json", "both"], default="both")
    parser.add_argument("--params", "-p", type=str, default=None, help="JSON params e.g. '{\"horizon\": 2, \"budget\": 1e6}'")
    parser.add_argument("--temporal", "-t", choices=["t1", "t2", "t3", "t4"], default=None, help="Focus timescale")
    parser.add_argument("--substrate", "-s", choices=SUBSTRATES, default=None, help="Focus substrate")
    parser.add_argument("--enrich", "-e", action="store_true", help="Add LLM narrative layer (Gemini/Claude/OpenAI); deterministic only if omitted")

    args = parser.parse_args()
    params = json.loads(args.params) if args.params else None

    print("Generating OMEGA-MAX decision brief for:", args.query)
    if getattr(args, "enrich", False):
        print("(With LLM enrichment)")
    brief = generate_brief(
        args.query,
        params=params,
        temporal_focus=args.temporal,
        substrate_focus=args.substrate,
        enrich=getattr(args, "enrich", False),
    )

    if args.format in ["md", "both"]:
        md_path = f"{args.output}.md"
        brief.to_markdown(md_path)
        print("Markdown:", md_path)
    if args.format in ["json", "both"]:
        json_path = f"{args.output}.json"
        brief.to_json(json_path)
        print("JSON:", json_path)

    print("\n" + "=" * 60)
    print(brief.to_markdown())
