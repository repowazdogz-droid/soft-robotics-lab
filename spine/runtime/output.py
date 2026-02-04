"""Output formatter for Spine Decision Runtime results."""

import yaml
import json
from datetime import datetime
from typing import Dict, Any, Optional
from .schemas import DecisionAnalysis
from .trace import DecisionTraceGraph, TraceNode


def format_analysis(analysis: DecisionAnalysis) -> str:
    """
    Format decision analysis as YAML string.
    
    Args:
        analysis: DecisionAnalysis to format
        
    Returns:
        Formatted YAML string
    """
    output_dict = {
        'decision_map': {
            'constraints_checked': [
                {
                    'constraint': check.constraint,
                    'checked': check.checked,
                    'contract_matched': check.contract_matched,
                    'violation': check.violation
                }
                for check in analysis.decision_map.constraints_checked
            ],
            'violations': analysis.decision_map.violations
        },
        'failure_modes': [
            {
                'mode': fm.mode,
                'severity': fm.severity,
                'mitigation': fm.mitigation,
                **({'epistemic': {
                    'confidence': fm.epistemic.confidence,
                    'evidence_type': fm.epistemic.evidence_type,
                    'provenance': fm.epistemic.provenance,
                    'requires_validation': fm.epistemic.requires_validation
                }} if fm.epistemic else {})
            }
            for fm in analysis.failure_modes
        ],
        'contradictions': [
            {
                'description': c.description,
                **({'epistemic': {
                    'confidence': c.epistemic.confidence,
                    'evidence_type': c.epistemic.evidence_type,
                    'provenance': c.epistemic.provenance,
                    'requires_validation': c.epistemic.requires_validation
                }} if c.epistemic else {})
            }
            for c in analysis.contradictions
        ],
        'unknowns': [
            {
                'item': u.item,
                'impact': u.impact,
                'resolution': u.resolution,
                **({'epistemic': {
                    'confidence': u.epistemic.confidence,
                    'evidence_type': u.epistemic.evidence_type,
                    'provenance': u.epistemic.provenance,
                    'requires_validation': u.epistemic.requires_validation
                }} if u.epistemic else {})
            }
            for u in analysis.unknowns
        ],
        'recommended_experiments': [
            {
                'name': exp.name,
                **({'epistemic': {
                    'confidence': exp.epistemic.confidence,
                    'evidence_type': exp.epistemic.evidence_type,
                    'provenance': exp.epistemic.provenance,
                    'requires_validation': exp.epistemic.requires_validation
                }} if exp.epistemic else {})
            }
            for exp in analysis.recommended_experiments
        ]
    }
    
    # Add trace graph summary if available
    if analysis.trace_graph:
        output_dict['trace_summary'] = {
            'run_id': analysis.trace_graph.get('run_id'),
            'node_count': len(analysis.trace_graph.get('nodes', {})),
            'timestamp': analysis.trace_graph.get('timestamp')
        }
    
    return yaml.dump(output_dict, default_flow_style=False, sort_keys=False)


def print_analysis(analysis: DecisionAnalysis) -> None:
    """
    Print formatted analysis to stdout.
    
    Args:
        analysis: DecisionAnalysis to print
    """
    print(format_analysis(analysis))


def export_trace_json(analysis: DecisionAnalysis) -> str:
    """
    Export full trace graph as JSON string.
    
    Args:
        analysis: DecisionAnalysis containing trace graph
        
    Returns:
        JSON string of trace graph
    """
    if not analysis.trace_graph:
        return json.dumps({"error": "No trace graph available"}, indent=2)
    
    return json.dumps(analysis.trace_graph, indent=2, default=str)


def explain_output(analysis: DecisionAnalysis, output_id: str) -> str:
    """
    Generate human-readable explanation for a specific output.
    
    Args:
        analysis: DecisionAnalysis containing trace graph
        output_id: ID of the output node to explain
        
    Returns:
        Human-readable explanation string
    """
    if not analysis.trace_graph:
        return f"No trace graph available for output {output_id}"
    
    # Reconstruct trace graph from dict
    trace = DecisionTraceGraph()
    trace.run_id = analysis.trace_graph.get('run_id', '')
    trace.nodes = {}
    
    # Reconstruct nodes from dict
    for node_id, node_data in analysis.trace_graph.get('nodes', {}).items():
        timestamp = datetime.fromisoformat(node_data['timestamp']) if isinstance(node_data.get('timestamp'), str) else datetime.utcnow()
        
        trace.nodes[node_id] = TraceNode(
            id=node_data['id'],
            node_type=node_data['type'],
            content=node_data['content'],
            timestamp=timestamp,
            parents=node_data.get('parents', []),
            metadata=node_data.get('metadata', {})
        )
    
    return trace.explain(output_id)
