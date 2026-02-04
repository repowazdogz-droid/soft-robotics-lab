/**
 * OMEGA Tutor v2 — Learning workspace types.
 * Structured knowledge presentation, not chat.
 */

export type DepthLevel = "intuitive" | "structured" | "technical" | "research";

export interface TeachingResponse {
  coreExplanation: string;
  assumptions: string[];
  competingModels: CompetingModel[];
  temporalFraming: TemporalFraming;
  commonMisconceptions: string[];
  depthLayers: DepthLayer[];
  reflectionPrompt: string;
}

export interface CompetingModel {
  name: string;
  explanation: string;
  strengths: string;
  limitations: string;
}

export interface TemporalFraming {
  t1: string;
  t2: string;
  t3: string;
  t4: string;
}

export interface DepthLayer {
  level: string;
  content: string;
}

export interface ExplainBackResult {
  score: number;
  accurate: string[];
  missing: string[];
  misconceptions: string[];
  correction: string;
  reframing: string;
  deeperQuestion: string;
}
