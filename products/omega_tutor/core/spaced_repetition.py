"""
OMEGA Tutor — SM-2 algorithm for optimal review scheduling.
"""

import math
from datetime import date, timedelta
from typing import Dict, Any


class SM2:
    """SM-2 spaced repetition: interval grows when correct, resets when wrong."""

    MIN_EF = 1.3
    DEFAULT_EF = 2.5

    @staticmethod
    def calculate_interval(repetitions: int, easiness: float, previous_interval: float) -> int:
        """Next interval in days. If repetitions==0, return 1. Else use SM-2 formula."""
        if repetitions <= 0:
            return 1
        if repetitions == 1:
            return 1
        if repetitions == 2:
            return 6
        return max(1, round(previous_interval * easiness))

    @staticmethod
    def update_card(correct: bool, quality: int, repetitions: int, easiness: float, previous_interval: float) -> Dict[str, Any]:
        """
        quality: 0-5 (0=blackout, 5=perfect). Returns next_review_date, interval, easiness, repetitions.
        """
        quality = max(0, min(5, quality))
        if not correct or quality < 3:
            return {
                "next_review_date": date.today().isoformat(),
                "interval": 1,
                "easiness": max(SM2.MIN_EF, easiness - 0.2),
                "repetitions": 0,
            }
        # Correct: update EF and interval
        ef = easiness + (0.1 - (5 - quality) * (0.08 + (5 - quality) * 0.02))
        ef = max(SM2.MIN_EF, ef)
        new_reps = repetitions + 1
        interval = SM2.calculate_interval(new_reps, ef, previous_interval)
        next_date = date.today() + timedelta(days=interval)
        return {
            "next_review_date": next_date.isoformat(),
            "interval": interval,
            "easiness": round(ef, 2),
            "repetitions": new_reps,
        }
