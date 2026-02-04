import type { CaseSynthesisOutput } from '../types';

/**
 * Ensures LLM response has all required keys and arrays.
 * Used as a fallback if the API returns partial or malformed JSON.
 */
export function ensureOutputStructure(parsed: Partial<CaseSynthesisOutput>): CaseSynthesisOutput {
  return {
    caseSynthesis: {
      summary: parsed.caseSynthesis?.summary ?? 'Summary not available.',
      tensions: Array.isArray(parsed.caseSynthesis?.tensions) ? parsed.caseSynthesis.tensions : [],
      missingInformation: Array.isArray(parsed.caseSynthesis?.missingInformation) ? parsed.caseSynthesis.missingInformation : [],
    },
    considerations: {
      conservativeFactors: Array.isArray(parsed.considerations?.conservativeFactors) ? parsed.considerations.conservativeFactors : [],
      interventionalFactors: Array.isArray(parsed.considerations?.interventionalFactors) ? parsed.considerations.interventionalFactors : [],
      surgicalFactors: Array.isArray(parsed.considerations?.surgicalFactors) ? parsed.considerations.surgicalFactors : [],
      areasOfDebate: Array.isArray(parsed.considerations?.areasOfDebate) ? parsed.considerations.areasOfDebate : [],
    },
    riskRegretMap: {
      commonFailureModes: Array.isArray(parsed.riskRegretMap?.commonFailureModes) ? parsed.riskRegretMap.commonFailureModes : [],
      patientSpecificFactors: Array.isArray(parsed.riskRegretMap?.patientSpecificFactors) ? parsed.riskRegretMap.patientSpecificFactors : [],
      dissatisfactionDrivers: Array.isArray(parsed.riskRegretMap?.dissatisfactionDrivers) ? parsed.riskRegretMap.dissatisfactionDrivers : [],
    },
    assumptions: {
      currentAssumptions: Array.isArray(parsed.assumptions?.currentAssumptions) ? parsed.assumptions.currentAssumptions : [],
      wouldChangeThinking: Array.isArray(parsed.assumptions?.wouldChangeThinking) ? parsed.assumptions.wouldChangeThinking : [],
      possibleOverweighting: Array.isArray(parsed.assumptions?.possibleOverweighting) ? parsed.assumptions.possibleOverweighting : [],
      possibleUnderweighting: Array.isArray(parsed.assumptions?.possibleUnderweighting) ? parsed.assumptions.possibleUnderweighting : [],
    },
    patientFraming: {
      plainLanguageExplanation: parsed.patientFraming?.plainLanguageExplanation ?? '',
      outcomeRanges: parsed.patientFraming?.outcomeRanges ?? '',
      uncertaintyStatements: Array.isArray(parsed.patientFraming?.uncertaintyStatements) ? parsed.patientFraming.uncertaintyStatements : [],
      sharedDecisionPrompts: Array.isArray(parsed.patientFraming?.sharedDecisionPrompts) ? parsed.patientFraming.sharedDecisionPrompts : [],
    },
    cognitiveChecklist: {
      pauseQuestions: Array.isArray(parsed.cognitiveChecklist?.pauseQuestions) ? parsed.cognitiveChecklist.pauseQuestions : [],
      doNotProceedUnless: Array.isArray(parsed.cognitiveChecklist?.doNotProceedUnless) ? parsed.cognitiveChecklist.doNotProceedUnless : [],
      secondOrderEffects: Array.isArray(parsed.cognitiveChecklist?.secondOrderEffects) ? parsed.cognitiveChecklist.secondOrderEffects : [],
    },
    limitations: {
      cannotDetermine: Array.isArray(parsed.limitations?.cannotDetermine) ? parsed.limitations.cannotDetermine : [],
      requiresClinicalJudgment: Array.isArray(parsed.limitations?.requiresClinicalJudgment) ? parsed.limitations.requiresClinicalJudgment : [],
    },
    generatedAt: parsed.generatedAt ?? new Date().toISOString(),
    disclaimer: parsed.disclaimer ?? 'This synthesis is a cognitive support tool only. It does not constitute medical advice, diagnosis, or treatment recommendation.',
  };
}
