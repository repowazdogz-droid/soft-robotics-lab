// Patterns that indicate unsafe requests
const UNSAFE_INPUT_PATTERNS = [
  /what should i do/i,
  /what do you recommend/i,
  /which (surgery|procedure|treatment) should/i,
  /tell me the diagnosis/i,
  /what are the chances/i,
  /what('s| is) the probability/i,
  /success rate/i,
  /should i operate/i,
  /is this (cancer|malignant|benign)/i,
  /give me (a |the )?diagnosis/i,
];

// Patterns that should never appear in output
const UNSAFE_OUTPUT_PATTERNS = [
  /i recommend/i,
  /you should/i,
  /the diagnosis is/i,
  /success rate of \d/i,
  /\d+%( chance| probability| likelihood)/i,
  /the best (option|choice|treatment) is/i,
  /you must/i,
  /definitely/i,
  /certainly/i,
  /guaranteed/i,
];

export function checkInputSafety(input: string): { safe: boolean; warning?: string } {
  for (const pattern of UNSAFE_INPUT_PATTERNS) {
    if (pattern.test(input)) {
      return {
        safe: true, // Still allow input, but we'll handle it safely
        warning: 'This tool provides thinking support only. It cannot provide diagnoses, recommendations, or probabilities.'
      };
    }
  }
  return { safe: true };
}

export function checkOutputSafety(output: string): { safe: boolean; violations: string[] } {
  const violations: string[] = [];

  for (const pattern of UNSAFE_OUTPUT_PATTERNS) {
    if (pattern.test(output)) {
      violations.push(pattern.toString());
    }
  }

  return {
    safe: violations.length === 0,
    violations
  };
}

export function sanitiseOutput(output: string): string {
  // Replace any unsafe patterns that slipped through
  let sanitised = output;

  sanitised = sanitised.replace(/I recommend/gi, 'Considerations include');
  sanitised = sanitised.replace(/You should/gi, 'It may be worth reflecting on');
  sanitised = sanitised.replace(/The diagnosis is/gi, 'Clinical assessment would determine');
  sanitised = sanitised.replace(/definitely/gi, 'potentially');
  sanitised = sanitised.replace(/certainly/gi, 'possibly');

  return sanitised;
}
