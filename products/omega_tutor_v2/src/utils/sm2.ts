/**
 * OMEGA Tutor v2 — SM-2 spaced repetition algorithm.
 * Phase 4: full implementation.
 */

export interface SM2Result {
  nextReviewDate: string;
  interval: number;
  easiness: number;
  repetitions: number;
}

const MIN_EF = 1.3;

export function calculateInterval(
  repetitions: number,
  easiness: number,
  previousInterval: number
): number {
  if (repetitions <= 0) return 1;
  if (repetitions === 1) return 1;
  if (repetitions === 2) return 6;
  return Math.max(1, Math.round(previousInterval * easiness));
}

export function updateCard(
  correct: boolean,
  quality: number,
  repetitions: number,
  easiness: number,
  previousInterval: number
): SM2Result {
  const q = Math.max(0, Math.min(5, quality));
  if (!correct || q < 3) {
    return {
      nextReviewDate: new Date().toISOString().slice(0, 10),
      interval: 1,
      easiness: Math.max(MIN_EF, easiness - 0.2),
      repetitions: 0,
    };
  }
  const ef =
    easiness + (0.1 - (5 - q) * (0.08 + (5 - q) * 0.02));
  const newEf = Math.max(MIN_EF, ef);
  const newReps = repetitions + 1;
  const interval = calculateInterval(newReps, newEf, previousInterval);
  const next = new Date();
  next.setDate(next.getDate() + interval);
  return {
    nextReviewDate: next.toISOString().slice(0, 10),
    interval,
    easiness: Math.round(newEf * 100) / 100,
    repetitions: newReps,
  };
}
