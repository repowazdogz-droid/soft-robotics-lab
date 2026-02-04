/**
 * OMEGA Tutor v2 — Misconception detection prompt.
 * Phase 3: preemptive warning + detection in explain-back.
 */

export const MISCONCEPTION_DETECTION_PROMPT = `Check the following explanation for common misconceptions about the topic.

Topic: {topic}
User explanation: {userExplanation}

Return JSON:
{
  "hasMisconceptions": true/false,
  "detected": [{"misconception": "...", "correction": "..."}],
  "corrections": ["Correction 1", "Correction 2"]
}`;
