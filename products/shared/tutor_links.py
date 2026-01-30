"""
Generate links to OMEGA Tutor for learning about failures.
"""
from typing import Optional


def get_tutor_link(topic: str, port: int = 8503) -> str:
    """
    Generate a Tutor URL with topic pre-filled.

    Args:
        topic: The topic to learn about
        port: Tutor port (default 8503)

    Returns:
        URL string like http://localhost:8503/?topic=physics+simulation+stability
    """
    encoded_topic = topic.replace(" ", "+")
    return f"http://localhost:{port}/?topic={encoded_topic}"


def get_failure_tutor_link(failure_code: str, port: int = 8503) -> Optional[str]:
    """
    Get Tutor link for a specific failure code.
    Returns None if the code has no tutor topic.
    """
    from shared.failures import FailureCode, FAILURE_TUTOR_TOPICS

    try:
        code = FailureCode(failure_code)
        topic = FAILURE_TUTOR_TOPICS.get(code)
        if topic:
            return get_tutor_link(topic, port)
    except ValueError:
        pass
    return None
