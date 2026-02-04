/**
 * OMEGA Tutor v2 — Misconception detection. Pre-stored DB + detection.
 * Phase 3: port from omega_tutor data.
 */

export interface Misconception {
  id: string;
  topic: string;
  misconception: string;
  correctUnderstanding: string;
  commonTriggers: string[];
  severity: "minor" | "moderate" | "critical";
}

export function getTopicMisconceptions(_topic: string): Misconception[] {
  return [];
}

export function checkExplanation(
  _topic: string,
  _userText: string
): { hasMisconceptions: boolean; corrections: string[] } {
  return { hasMisconceptions: false, corrections: [] };
}
