"""OMEGA Scientist integrations."""

try:
    from .ledger_integration import (
        create_hypothesis_from_contradiction,
        create_hypothesis_from_cross_domain,
        create_hypothesis_from_failure,
        create_hypothesis_from_revival,
        bulk_create_from_scientist_session,
        get_ledger,
        LEDGER_AVAILABLE,
    )
except ImportError:
    LEDGER_AVAILABLE = False
    create_hypothesis_from_contradiction = None
    create_hypothesis_from_cross_domain = None
    create_hypothesis_from_failure = None
    create_hypothesis_from_revival = None
    bulk_create_from_scientist_session = None
    get_ledger = None
