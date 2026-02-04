export interface CaseInput {
  // Structured fields
  age: number | null;
  sex: 'male' | 'female' | 'other' | null;
  symptomDuration: string;
  comorbidities: string;
  priorTreatments: string;

  // Free text fields
  presentingSymptoms: string;
  imagingSummary: string;
  patientGoals: string;
  clinicianLeaning: string; // Optional
}

export interface CaseSynthesisOutput {
  caseSynthesis: {
    summary: string;
    tensions: string[];
    missingInformation: string[];
  };
  considerations: {
    conservativeFactors: string[];
    interventionalFactors: string[];
    surgicalFactors: string[];
    areasOfDebate: string[];
  };
  riskRegretMap: {
    commonFailureModes: string[];
    patientSpecificFactors: string[];
    dissatisfactionDrivers: string[];
  };
  assumptions: {
    currentAssumptions: string[];
    wouldChangeThinking: string[];
    possibleOverweighting: string[];
    possibleUnderweighting: string[];
  };
  patientFraming: {
    plainLanguageExplanation: string;
    outcomeRanges: string;
    uncertaintyStatements: string[];
    sharedDecisionPrompts: string[];
  };
  cognitiveChecklist: {
    pauseQuestions: string[];
    doNotProceedUnless: string[];
    secondOrderEffects: string[];
  };
  limitations: {
    cannotDetermine: string[];
    requiresClinicalJudgment: string[];
  };
  generatedAt: string;
  disclaimer: string;
}

export type AppState = 'input' | 'loading' | 'output';
