import { SYSTEM_PROMPT } from '../prompts/systemPrompt';
import type { CaseInput, CaseSynthesisOutput } from '../types';
import { checkOutputSafety, sanitiseOutput } from './safetyFilter';
import { ensureOutputStructure } from './formatOutput';

const LOCALE_LANGUAGE: Record<string, string> = {
  en: 'English',
  de: 'German',
  fr: 'French',
};

/** Call serverless proxy so the Gemini API key stays server-side only. */
const API_GENERATE_URL = '/api/generate';

export async function generateCaseSynthesis(caseInput: CaseInput, locale: string = 'en'): Promise<CaseSynthesisOutput> {
  const userMessage = formatCaseForLLM(caseInput);
  const languageInstruction = locale && LOCALE_LANGUAGE[locale]
    ? `\n\nIMPORTANT: Produce the entire JSON response in ${LOCALE_LANGUAGE[locale]}. All section content (strings and arrays) must be written in ${LOCALE_LANGUAGE[locale]}.\n\n`
    : '';

  const response = await fetch(API_GENERATE_URL, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      contents: [
        { role: 'user', parts: [{ text: SYSTEM_PROMPT + languageInstruction + '\n---\n\nCASE INFORMATION:\n\n' + userMessage }] }
      ],
      generationConfig: {
        responseMimeType: 'application/json',
        temperature: 0.3, // Lower temperature for more consistent, careful output
      }
    })
  });

  if (!response.ok) {
    throw new Error('Failed to generate case synthesis');
  }

  const data = await response.json();
  let content = data.candidates?.[0]?.content?.parts?.[0]?.text;

  if (!content) {
    throw new Error('No content in response');
  }

  // Safety check the output
  const safetyCheck = checkOutputSafety(content);
  if (!safetyCheck.safe) {
    console.warn('Safety violations detected, sanitising output:', safetyCheck.violations);
    content = sanitiseOutput(content);
  }

  let parsed: CaseSynthesisOutput;
  try {
    parsed = JSON.parse(content) as CaseSynthesisOutput;
  } catch {
    throw new Error('Invalid response format from synthesis service');
  }

  parsed = ensureOutputStructure(parsed);
  parsed.generatedAt = new Date().toISOString();
  parsed.disclaimer = 'This synthesis is a cognitive support tool only. It does not constitute medical advice, diagnosis, or treatment recommendation. All clinical decisions require professional judgment and direct patient assessment.';

  return parsed;
}

function formatCaseForLLM(input: CaseInput): string {
  const parts: string[] = [];

  if (input.age) parts.push(`Age: ${input.age}`);
  if (input.sex) parts.push(`Sex: ${input.sex}`);
  if (input.symptomDuration) parts.push(`Duration of symptoms: ${input.symptomDuration}`);
  if (input.comorbidities) parts.push(`Relevant comorbidities: ${input.comorbidities}`);
  if (input.priorTreatments) parts.push(`Prior treatments attempted: ${input.priorTreatments}`);
  if (input.presentingSymptoms) parts.push(`Presenting symptoms: ${input.presentingSymptoms}`);
  if (input.imagingSummary) parts.push(`Imaging summary: ${input.imagingSummary}`);
  if (input.patientGoals) parts.push(`Patient goals and expectations: ${input.patientGoals}`);
  if (input.clinicianLeaning) parts.push(`Clinician's current thinking (optional context): ${input.clinicianLeaning}`);

  return parts.join('\n\n');
}
