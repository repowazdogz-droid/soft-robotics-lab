/**
 * OMEGA Tutor v2 — Depth level prompts. Four levels, not personas.
 */

export const LEVEL_PROMPTS: Record<string, string> = {
  intuitive:
    "Explain using simple language, concrete examples, and everyday analogies. Avoid jargon. Make it accessible to someone with no background.",

  structured:
    "Explain with clear structure, practical applications, and real-world relevance. Include key terminology with definitions. Suitable for undergraduate level.",

  technical:
    "Explain with full technical depth, mechanisms, edge cases, and nuances. Assume domain familiarity. Include mathematical formulations where relevant.",

  research:
    "Explain at research level. Include: current state of the field, open problems, competing theories, methodological debates, key citations, and temporal framing (T1-T4). Assume expert familiarity.",
};
