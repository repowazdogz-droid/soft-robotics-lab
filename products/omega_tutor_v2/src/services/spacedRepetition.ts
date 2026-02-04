/**
 * OMEGA Tutor v2 — SM-2 spaced repetition.
 * Phase 4: scheduleReview, getDueReviews.
 */

export interface ReviewItem {
  topic: string;
  lastReview: Date;
  nextReview: Date;
  easeFactor: number;
  interval: number;
  repetitions: number;
}

export function scheduleReview(
  _topic: string,
  _quality: number
): Partial<ReviewItem> {
  return {};
}

export function getDueReviews(): ReviewItem[] {
  return [];
}
