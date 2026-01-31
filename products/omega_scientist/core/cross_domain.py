"""
Cross-Domain Collider - Find unexpected connections between domains

Discoveries often come from connecting ideas across fields:
- Robotics + Biology = Bio-inspired actuators
- Materials + Medicine = Drug delivery systems
- ML + Chemistry = Molecular design
"""
import json
import re
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Set

from .claim_extractor import Claim


class DomainType(Enum):
    ROBOTICS = "robotics"
    BIOLOGY = "biology"
    MATERIALS = "materials"
    MEDICINE = "medicine"
    CHEMISTRY = "chemistry"
    PHYSICS = "physics"
    COMPUTING = "computing"
    ENGINEERING = "engineering"
    NEUROSCIENCE = "neuroscience"
    GENETICS = "genetics"


DOMAIN_KEYWORDS = {
    DomainType.ROBOTICS: {"robot", "actuator", "gripper", "manipulator", "servo", "motor", "kinematics", "end-effector", "locomotion"},
    DomainType.BIOLOGY: {"cell", "tissue", "organism", "protein", "enzyme", "biological", "living", "species", "evolution", "membrane"},
    DomainType.MATERIALS: {"polymer", "elastomer", "silicone", "composite", "alloy", "stiffness", "modulus", "tensile", "material", "substrate"},
    DomainType.MEDICINE: {"patient", "clinical", "therapy", "treatment", "diagnosis", "disease", "surgical", "medical", "drug", "dose"},
    DomainType.CHEMISTRY: {"molecule", "reaction", "synthesis", "chemical", "compound", "catalyst", "bond", "molecular", "reagent"},
    DomainType.PHYSICS: {"force", "energy", "momentum", "quantum", "electromagnetic", "thermodynamic", "mechanics", "field", "wave"},
    DomainType.COMPUTING: {"algorithm", "neural network", "machine learning", "ai", "computation", "software", "data", "model", "training"},
    DomainType.ENGINEERING: {"design", "fabrication", "manufacture", "system", "structural", "mechanical", "prototype", "cad"},
    DomainType.NEUROSCIENCE: {"neuron", "brain", "cortex", "synapse", "cognitive", "neural", "perception", "motor control"},
    DomainType.GENETICS: {"gene", "dna", "rna", "genome", "mutation", "crispr", "expression", "sequencing", "plasmid"},
}


@dataclass
class DomainConnection:
    domain_a: DomainType
    domain_b: DomainType
    concept_a: str
    concept_b: str
    connection_type: str
    strength: float
    description: str
    paper_sources: List[str]
    novel: bool
    hypothesis: Optional[str]


@dataclass
class CollisionReport:
    papers_analyzed: List[str]
    domains_found: List[DomainType]
    connections: List[DomainConnection]
    novel_connections: List[DomainConnection]
    synthesis_opportunities: List[dict]


def detect_domains(text: str) -> Set[DomainType]:
    """Detect which domains a text belongs to."""
    text_lower = text.lower()
    domains = set()

    for domain, keywords in DOMAIN_KEYWORDS.items():
        matches = sum(1 for kw in keywords if kw in text_lower)
        if matches >= 2:
            domains.add(domain)

    return domains


def detect_paper_domains(claims: List[Claim]) -> Set[DomainType]:
    """Detect domains covered by a paper based on its claims."""
    all_domains = set()
    for claim in claims:
        all_domains |= detect_domains(claim.text)
    return all_domains


def extract_key_concept(text: str, domain: DomainType) -> str:
    """Extract the key concept from text for a given domain."""
    keywords = DOMAIN_KEYWORDS.get(domain, set())
    text_lower = text.lower()

    for kw in sorted(keywords, key=len, reverse=True):
        if kw in text_lower:
            idx = text_lower.find(kw)
            start = max(0, idx - 20)
            end = min(len(text), idx + len(kw) + 20)
            return text[start:end].strip()

    return text[:50].strip()


def detect_pattern_connection(
    claim_a: Claim,
    claim_b: Claim,
    domain_a: DomainType,
    domain_b: DomainType,
) -> Optional[dict]:
    """Rule-based connection detection."""
    text_a = claim_a.text.lower()
    text_b = claim_b.text.lower()

    structural_words = {"structure", "shape", "geometry", "form", "architecture", "configuration"}
    if any(w in text_a for w in structural_words) and any(w in text_b for w in structural_words):
        return {
            "type": "structural",
            "strength": 0.7,
            "description": f"Structural similarity between {domain_a.value} and {domain_b.value}",
        }

    functional_words = {"function", "perform", "achieve", "enable", "accomplish", "mechanism"}
    if any(w in text_a for w in functional_words) and any(w in text_b for w in functional_words):
        return {
            "type": "functional",
            "strength": 0.6,
            "description": f"Functional parallel between {domain_a.value} and {domain_b.value}",
        }

    property_words = {"stiffness", "flexibility", "strength", "compliance", "elasticity", "viscosity"}
    if any(w in text_a for w in property_words) and any(w in text_b for w in property_words):
        return {
            "type": "mechanistic",
            "strength": 0.8,
            "description": f"Shared mechanical properties across {domain_a.value} and {domain_b.value}",
        }

    return None


def generate_cross_domain_hypothesis(
    claim_a: Claim,
    claim_b: Claim,
    domain_a: DomainType,
    domain_b: DomainType,
) -> str:
    """Generate a hypothesis that bridges two domains."""
    templates = [
        f"The {domain_a.value} principle could be applied to {domain_b.value} to achieve similar effects",
        f"Combining {domain_a.value} and {domain_b.value} approaches may yield novel solutions",
        f"The mechanism observed in {domain_a.value} may have analogues in {domain_b.value}",
        f"Cross-domain transfer from {domain_a.value} to {domain_b.value} could enable new capabilities",
    ]
    return templates[hash(claim_a.text + claim_b.text) % len(templates)]


def detect_llm_connection(
    claim_a: Claim,
    claim_b: Claim,
    domain_a: DomainType,
    domain_b: DomainType,
) -> Optional[dict]:
    """Use LLM to find deeper connections."""
    try:
        from openai import OpenAI

        client = OpenAI(
            base_url="http://localhost:1234/v1",
            api_key="not-needed",
            timeout=30,
        )

        prompt = f"""Find connections between these two concepts from different domains.

Domain A ({domain_a.value}): {claim_a.text}
Domain B ({domain_b.value}): {claim_b.text}

If there's a meaningful connection (structural, functional, mechanistic, or analogical), respond with JSON:
{{
    "connected": true,
    "type": "structural|functional|mechanistic|analogical",
    "strength": 0.0-1.0,
    "concept_a": "key concept from A",
    "concept_b": "key concept from B",
    "description": "how they connect",
    "hypothesis": "a novel hypothesis combining both"
}}

If no meaningful connection, respond: {{"connected": false}}

Only return JSON."""

        response = client.chat.completions.create(
            model="phi-3-mini-4k-instruct",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=300,
            temperature=0.5,
        )

        raw = response.choices[0].message.content or ""
        raw = re.sub(r"^```(?:json)?\s*", "", raw)
        raw = re.sub(r"\s*```\s*$", "", raw).strip()
        result = json.loads(raw)

        if result.get("connected"):
            return result

    except Exception:
        pass

    return None


def detect_connection(
    claim_a: Claim,
    claim_b: Claim,
    domain_a: DomainType,
    domain_b: DomainType,
    source: str,
    use_llm: bool,
) -> Optional[DomainConnection]:
    """Detect if two claims have a meaningful cross-domain connection."""
    connection = detect_pattern_connection(claim_a, claim_b, domain_a, domain_b)

    if connection:
        return DomainConnection(
            domain_a=domain_a,
            domain_b=domain_b,
            concept_a=extract_key_concept(claim_a.text, domain_a),
            concept_b=extract_key_concept(claim_b.text, domain_b),
            connection_type=connection["type"],
            strength=connection["strength"],
            description=connection["description"],
            paper_sources=[source],
            novel=False,
            hypothesis=generate_cross_domain_hypothesis(claim_a, claim_b, domain_a, domain_b),
        )

    if use_llm:
        connection = detect_llm_connection(claim_a, claim_b, domain_a, domain_b)
        if connection:
            return DomainConnection(
                domain_a=domain_a,
                domain_b=domain_b,
                concept_a=connection.get("concept_a", claim_a.text[:50]),
                concept_b=connection.get("concept_b", claim_b.text[:50]),
                connection_type=connection.get("type", "analogical"),
                strength=float(connection.get("strength", 0.5)),
                description=connection.get("description", "LLM-detected connection"),
                paper_sources=[source],
                novel=False,
                hypothesis=connection.get("hypothesis"),
            )

    return None


def extract_connections_from_paper(
    paper: str,
    claims: List[Claim],
    domains: List[DomainType],
    use_llm: bool,
) -> List[DomainConnection]:
    """Extract cross-domain connections within a single paper."""
    connections = []

    claims_by_domain: Dict[DomainType, List[Claim]] = {d: [] for d in domains}
    for claim in claims:
        claim_domains = detect_domains(claim.text)
        for d in claim_domains:
            if d in claims_by_domain:
                claims_by_domain[d].append(claim)

    domain_list = list(domains)
    for i, domain_a in enumerate(domain_list):
        for domain_b in domain_list[i + 1 :]:
            for claim_a in claims_by_domain.get(domain_a, []):
                for claim_b in claims_by_domain.get(domain_b, []):
                    connection = detect_connection(
                        claim_a, claim_b, domain_a, domain_b, paper, use_llm
                    )
                    if connection:
                        connections.append(connection)

    return connections


def find_connections_between_papers(
    paper_a: str,
    claims_a: List[Claim],
    domains_a: Set[DomainType],
    paper_b: str,
    claims_b: List[Claim],
    domains_b: Set[DomainType],
    use_llm: bool,
) -> List[DomainConnection]:
    """Find cross-domain connections between two papers."""
    connections = []
    source = f"{paper_a} + {paper_b}"

    for claim_a in claims_a:
        for claim_b in claims_b:
            for domain_a in domains_a:
                for domain_b in domains_b:
                    if domain_a != domain_b:
                        connection = detect_connection(
                            claim_a, claim_b, domain_a, domain_b, source, use_llm
                        )
                        if connection:
                            connections.append(connection)

    return connections


def is_novel_connection(connection: DomainConnection) -> bool:
    """Determine if a connection is novel/unexpected."""
    common_pairings = {
        (DomainType.ROBOTICS, DomainType.ENGINEERING),
        (DomainType.BIOLOGY, DomainType.MEDICINE),
        (DomainType.CHEMISTRY, DomainType.MATERIALS),
        (DomainType.COMPUTING, DomainType.ENGINEERING),
        (DomainType.PHYSICS, DomainType.ENGINEERING),
    }

    pair = (connection.domain_a, connection.domain_b)
    reverse_pair = (connection.domain_b, connection.domain_a)

    if pair in common_pairings or reverse_pair in common_pairings:
        return connection.strength > 0.8

    return connection.strength > 0.5


def generate_synthesis_opportunities(
    connections: List[DomainConnection],
    use_llm: bool,
) -> List[dict]:
    """Generate synthesis opportunities from connections."""
    opportunities = []
    pairs: Dict[tuple, List[DomainConnection]] = {}

    for conn in connections:
        pair = tuple(sorted([conn.domain_a.value, conn.domain_b.value]))
        if pair not in pairs:
            pairs[pair] = []
        pairs[pair].append(conn)

    for pair, conns in pairs.items():
        if len(conns) >= 2:
            concepts = list(set([c.concept_a for c in conns] + [c.concept_b for c in conns]))[:5]
            opportunities.append({
                "domains": list(pair),
                "connection_count": len(conns),
                "avg_strength": sum(c.strength for c in conns) / len(conns),
                "concepts": concepts,
                "synthesis_idea": f"Multiple connections between {pair[0]} and {pair[1]} suggest synthesis opportunity",
            })

    return sorted(opportunities, key=lambda x: x["avg_strength"], reverse=True)


def find_cross_domain_connections(
    claims_by_paper: Dict[str, List[Claim]],
    use_llm: bool = True,
) -> CollisionReport:
    """
    Find connections between different research domains.

    Args:
        claims_by_paper: Dict mapping paper IDs to their claims
        use_llm: Use LLM for deeper analysis

    Returns:
        CollisionReport with all found connections
    """
    domains_by_paper = {
        paper: detect_paper_domains(claims)
        for paper, claims in claims_by_paper.items()
    }

    all_domains: Set[DomainType] = set()
    for domains in domains_by_paper.values():
        all_domains |= domains

    cross_domain_papers = {
        paper: domains
        for paper, domains in domains_by_paper.items()
        if len(domains) >= 2
    }

    connections: List[DomainConnection] = []

    for paper, domains in cross_domain_papers.items():
        claims = claims_by_paper[paper]
        paper_connections = extract_connections_from_paper(
            paper, claims, list(domains), use_llm
        )
        connections.extend(paper_connections)

    paper_ids = list(claims_by_paper.keys())
    for i, paper_a in enumerate(paper_ids):
        for paper_b in paper_ids[i + 1 :]:
            domains_a = domains_by_paper[paper_a]
            domains_b = domains_by_paper[paper_b]

            if domains_a and domains_b and not (domains_a & domains_b):
                cross_connections = find_connections_between_papers(
                    paper_a,
                    claims_by_paper[paper_a],
                    domains_a,
                    paper_b,
                    claims_by_paper[paper_b],
                    domains_b,
                    use_llm,
                )
                connections.extend(cross_connections)

    for conn in connections:
        conn.novel = is_novel_connection(conn)

    novel_connections = [c for c in connections if c.novel]
    synthesis_opportunities = generate_synthesis_opportunities(connections, use_llm)

    return CollisionReport(
        papers_analyzed=list(claims_by_paper.keys()),
        domains_found=list(all_domains),
        connections=connections,
        novel_connections=novel_connections,
        synthesis_opportunities=synthesis_opportunities,
    )


def summarize_collisions(report: CollisionReport) -> str:
    """Generate human-readable summary."""
    lines = [
        "# Cross-Domain Analysis",
        "",
        f"**Papers analyzed:** {len(report.papers_analyzed)}",
        f"**Domains found:** {', '.join(d.value for d in report.domains_found)}",
        f"**Connections found:** {len(report.connections)}",
        f"**Novel connections:** {len(report.novel_connections)}",
        "",
        "## Top Novel Connections",
        "",
    ]

    for i, conn in enumerate(report.novel_connections[:5], 1):
        lines.append(f"### {i}. {conn.domain_a.value.title()} ↔ {conn.domain_b.value.title()}")
        lines.append(f"**Type:** {conn.connection_type}")
        lines.append(f"**Strength:** {conn.strength:.0%}")
        lines.append(f"**Description:** {conn.description}")
        if conn.hypothesis:
            lines.append(f"**Hypothesis:** {conn.hypothesis}")
        lines.append("")

    if report.synthesis_opportunities:
        lines.append("## Synthesis Opportunities")
        lines.append("")
        for opp in report.synthesis_opportunities[:3]:
            lines.append(f"- **{opp['domains'][0]} + {opp['domains'][1]}**: {opp['connection_count']} connections, {opp['avg_strength']:.0%} avg strength")

    return "\n".join(lines)
