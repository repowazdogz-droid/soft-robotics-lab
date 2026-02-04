/**
 * OMEGA Tutor v2 — Spaced repetition hook. Phase 4.
 */

import type { ReviewItem } from "../services/spacedRepetition";

export function useSpacedRepetition() {
  const dueReviews: ReviewItem[] = [];
  return {
    dueReviews,
    scheduleReview: (_topic: string, _quality: number) => {},
  };
}
