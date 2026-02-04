"""CLI entry point for Spine Decision Runtime."""

import click
import sys
from pathlib import Path

from .parser import parse_case_file
from .analyzer import DecisionAnalyzer
from .output import print_analysis


@click.group()
def cli():
    """Spine Decision Runtime - Analyze decision problems against contracts."""
    pass


@cli.command()
@click.argument('case_file', type=click.Path(exists=True))
@click.option('--contracts-dir', type=click.Path(exists=True), help='Path to contracts directory')
def analyze(case_file: str, contracts_dir: str = None):
    """
    Analyze a case file and output decision analysis.
    
    CASE_FILE: Path to the case.yaml file to analyze
    """
    try:
        # Parse case file
        case = parse_case_file(case_file)
        
        # Initialize analyzer
        analyzer = DecisionAnalyzer(contracts_dir=contracts_dir)
        
        # Analyze
        analysis = analyzer.analyze(case)
        
        # Output results
        print_analysis(analysis)
        
    except FileNotFoundError as e:
        click.echo(f"Error: {e}", err=True)
        sys.exit(1)
    except Exception as e:
        click.echo(f"Error analyzing case: {e}", err=True)
        sys.exit(1)


if __name__ == '__main__':
    cli()
