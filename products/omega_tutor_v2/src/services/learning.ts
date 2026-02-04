/**
 * OMEGA Tutor v2 — Learning service. Teaching + explain-back.
 * Phase 2: teach(); Phase 3: evaluateExplanation().
 */

import { generateContent } from "./gemini";
import { TUTOR_SYSTEM_PROMPT } from "../prompts/tutorSystem";
import { LEVEL_PROMPTS } from "../prompts/levelPrompts";
import { buildExplainBackPrompt } from "../prompts/explainBack";
import type { DepthLevel, TeachingResponse, ExplainBackResult } from "../types/learning";

export async function teach(
  question: string,
  level: DepthLevel,
  apiKey?: string
): Promise<TeachingResponse> {
  const levelInstruction = LEVEL_PROMPTS[level] ?? "";

  const prompt = `${TUTOR_SYSTEM_PROMPT}

DEPTH LEVEL: ${level}
${levelInstruction}

USER QUESTION: ${question}

Return valid JSON matching the TeachingResponse structure.`;

  const payload = {
    contents: [{ role: "user", parts: [{ text: prompt }] }],
    generationConfig: {
      responseMimeType: "application/json",
      temperature: 0.3,
    },
  };

  const text = await generateContent(payload, apiKey);

  const parsed = JSON.parse(text) as TeachingResponse;

  if (!parsed.coreExplanation) {
    parsed.coreExplanation = String(parsed.coreExplanation ?? "").trim() || text;
  }
  if (!Array.isArray(parsed.assumptions)) parsed.assumptions = [];
  if (!Array.isArray(parsed.competingModels)) parsed.competingModels = [];
  if (!parsed.temporalFraming || typeof parsed.temporalFraming !== "object") {
    parsed.temporalFraming = { t1: "", t2: "", t3: "", t4: "" };
  }
  if (!Array.isArray(parsed.commonMisconceptions)) parsed.commonMisconceptions = [];
  if (!Array.isArray(parsed.depthLayers)) parsed.depthLayers = [];
  if (typeof parsed.reflectionPrompt !== "string") parsed.reflectionPrompt = "";

  return parsed;
}

export async function evaluateExplanation(
  userExplanation: string,
  originalTopic: string,
  originalResponse: TeachingResponse,
  apiKey?: string
): Promise<ExplainBackResult> {
  const keyConcepts =
    originalResponse.assumptions?.join(", ") || "N/A";

  const prompt = buildExplainBackPrompt(
    userExplanation,
    originalTopic,
    originalResponse.coreExplanation,
    keyConcepts
  );

  const payload = {
    contents: [{ role: "user", parts: [{ text: prompt }] }],
    generationConfig: {
      responseMimeType: "application/json",
      temperature: 0.3,
    },
  };

  const text = await generateContent(payload, apiKey);
  const parsed = JSON.parse(text) as ExplainBackResult;

  if (typeof parsed.score !== "number") parsed.score = 0;
  if (!Array.isArray(parsed.accurate)) parsed.accurate = [];
  if (!Array.isArray(parsed.missing)) parsed.missing = [];
  if (!Array.isArray(parsed.misconceptions)) parsed.misconceptions = [];
  if (typeof parsed.correction !== "string") parsed.correction = "";
  if (typeof parsed.reframing !== "string") parsed.reframing = "";
  if (typeof parsed.deeperQuestion !== "string") parsed.deeperQuestion = "";

  return parsed;
}
