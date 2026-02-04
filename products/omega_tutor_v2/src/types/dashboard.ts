/**
 * OMEGA Tutor v2 — Cognitive dashboard types.
 * Professional instrumentation, not gamification.
 */

export interface DomainMastery {
  name: string;
  topicsLearned: number;
  topicsTotal: number;
  averageScore: number;
  lastActivity: Date | null;
}

export interface RetentionItem {
  topic: string;
  domain: string;
  strength: number;
  lastReview: Date;
  nextReview: Date;
  easeFactor: number;
  interval: number;
  repetitions: number;
}

export interface RecordedMisconception {
  topic: string;
  misconception: string;
  correction: string;
  occurrences: number;
  lastSeen: Date;
}

export interface ReviewItem {
  topic: string;
  domain: string;
  dueDate: Date;
  isOverdue: boolean;
  strength: number;
}
