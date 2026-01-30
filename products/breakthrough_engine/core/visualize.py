"""
OMEGA Breakthrough Engine — Hypothesis relationship visualization.
Nodes: hypotheses. Edges: supports, contradicts, depends_on, derived_from.
Color by status; size by impact/importance.
"""

from typing import List, Dict, Any, Optional

try:
    import networkx as nx
    _nx = nx
except ImportError:
    _nx = None

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _plt = plt
except ImportError:
    _plt = None


def _status_color(status: str) -> str:
    s = (status or "").lower()
    if s in ("validated", "strengthened"):
        return "#22c55e"  # green
    if s in ("killed", "falsified"):
        return "#ef4444"  # red
    return "#3b82f6"  # blue (active, weakened, paused, merged)


def _hyp_to_node_label(h: Any) -> str:
    claim = getattr(h, "claim", "") or ""
    return (claim[:40] + "…") if len(claim) > 40 else claim


def generate_hypothesis_graph(hypotheses: List[Any]) -> Optional[Any]:
    """
    Build a Figure showing hypotheses as nodes, edges: supports, contradicts, depends_on, derived_from.
    Color by status (green=validated, red=killed, blue=active). Size by confidence (impact proxy).
    Returns matplotlib Figure or None if matplotlib/networkx unavailable.
    """
    if _nx is None or _plt is None or not hypotheses:
        return None
    G = _nx.DiGraph()
    for h in hypotheses:
        hid = getattr(h, "id", "") or ""
        if not hid:
            continue
        status = getattr(h, "status", None)
        status_val = status.value if status else "active"
        conf = getattr(h, "confidence", 0.5) or 0.5
        G.add_node(hid, label=_hyp_to_node_label(h), status=status_val, confidence=conf)
    for h in hypotheses:
        hid = getattr(h, "id", "") or ""
        for comp_id in getattr(h, "competing_hypotheses", []) or []:
            if comp_id and G.has_node(comp_id):
                G.add_edge(hid, comp_id, edge_type="contradicts")
        parent = getattr(h, "parent_hypothesis", None)
        if parent and G.has_node(parent):
            G.add_edge(parent, hid, edge_type="derived_from")
    if G.order() == 0:
        return None
    pos = _nx.spring_layout(G, k=1.5, iterations=50, seed=42)
    fig, ax = _plt.subplots(figsize=(10, 8))
    colors = [_status_color(G.nodes[n].get("status", "active")) for n in G.nodes()]
    sizes = [300 + 500 * G.nodes[n].get("confidence", 0.5) for n in G.nodes()]
    _nx.draw_networkx_nodes(G, pos, node_color=colors, node_size=sizes, ax=ax, alpha=0.9)
    _nx.draw_networkx_edges(G, pos, ax=ax, edge_color="#666", arrows=True, arrowsize=12)
    labels = {n: G.nodes[n].get("label", n)[:20] for n in G.nodes()}
    _nx.draw_networkx_labels(G, pos, labels, font_size=7, ax=ax)
    ax.axis("off")
    ax.set_title("Hypothesis relationships")
    return fig


def generate_html_graph(hypotheses: List[Any]) -> str:
    """
    Generate HTML string for interactive hypothesis graph (for web).
    Uses inline SVG or minimal JS; works without pyvis. If pyvis available, use it for richer graph.
    """
    try:
        from pyvis.network import Network
        net = Network(height="400px", width="100%", directed=True)
        for h in hypotheses:
            hid = getattr(h, "id", "") or ""
            if not hid:
                continue
            status = getattr(h, "status", None)
            status_val = status.value if status else "active"
            color = _status_color(status_val)
            title = (getattr(h, "claim", "") or "")[:200]
            net.add_node(hid, label=hid, title=title, color=color)
        for h in hypotheses:
            hid = getattr(h, "id", "") or ""
            for comp_id in getattr(h, "competing_hypotheses", []) or []:
                if comp_id:
                    net.add_edge(hid, comp_id, title="contradicts")
            parent = getattr(h, "parent_hypothesis", None)
            if parent:
                net.add_edge(parent, hid, title="derived_from")
        return net.generate_html()
    except ImportError:
        pass
    if _nx is None or not hypotheses:
        return "<p>Graph unavailable (install networkx and matplotlib or pyvis)</p>"
    G = _nx.DiGraph()
    for h in hypotheses:
        hid = getattr(h, "id", "") or ""
        if hid:
            G.add_node(hid, label=_hyp_to_node_label(h), status=getattr(h, "status", None) and getattr(h.status, "value", "active") or "active")
    for h in hypotheses:
        hid = getattr(h, "id", "") or ""
        for comp_id in getattr(h, "competing_hypotheses", []) or []:
            if comp_id and G.has_node(comp_id):
                G.add_edge(hid, comp_id)
        parent = getattr(h, "parent_hypothesis", None)
        if parent and G.has_node(parent):
            G.add_edge(parent, hid)
    if G.order() == 0:
        return "<p>No hypotheses to display</p>"
    pos = _nx.spring_layout(G, k=1.2, seed=42)
    width, height = 600, 400
    scale_x = width / 2
    scale_y = height / 2
    nodes_svg = []
    for n in G.nodes():
        x = pos[n][0] * scale_x + width / 2
        y = pos[n][1] * scale_y + height / 2
        color = _status_color(G.nodes[n].get("status", "active"))
        label = G.nodes[n].get("label", n)[:15]
        nodes_svg.append(f'<circle cx="{x:.0f}" cy="{y:.0f}" r="20" fill="{color}" stroke="#333"/>')
        nodes_svg.append(f'<text x="{x:.0f}" y="{y:.0f+5}" text-anchor="middle" font-size="10">{label}</text>')
    edges_svg = []
    for u, v in G.edges():
        x1, y1 = pos[u][0] * scale_x + width / 2, pos[u][1] * scale_y + height / 2
        x2, y2 = pos[v][0] * scale_x + width / 2, pos[v][1] * scale_y + height / 2
        edges_svg.append(f'<line x1="{x1:.0f}" y1="{y1:.0f}" x2="{x2:.0f}" y2="{y2:.0f}" stroke="#666" stroke-width="1"/>')
    svg = f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">' + "".join(edges_svg) + "".join(nodes_svg) + "</svg>"
    return f'<div class="hypothesis-graph">{svg}</div>'
