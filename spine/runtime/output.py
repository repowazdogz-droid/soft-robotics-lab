"""Output formatter for Spine Decision Runtime results."""

import yaml
from typing import Dict, Any
from .schemas import DecisionAnalysis


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
                'mitigation': fm.mitigation
            }
            for fm in analysis.failure_modes
        ],
        'contradictions': [
            {
                'description': c.description
            }
            for c in analysis.contradictions
        ],
        'unknowns': [
            {
                'item': u.item,
                'impact': u.impact,
                'resolution': u.resolution
            }
            for u in analysis.unknowns
        ],
        'recommended_experiments': [
            {
                'name': exp.name
            }
            for exp in analysis.recommended_experiments
        ]
    }
    
    return yaml.dump(output_dict, default_flow_style=False, sort_keys=False)


def print_analysis(analysis: DecisionAnalysis) -> None:
    """
    Print formatted analysis to stdout.
    
    Args:
        analysis: DecisionAnalysis to print
    """
    print(format_analysis(analysis))
