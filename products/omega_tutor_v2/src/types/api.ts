/**
 * OMEGA Tutor v2 — API / Gemini types.
 */

export interface GeminiGenerateRequest {
  contents: Array<{ role: string; parts: Array<{ text: string }> }>;
  generationConfig?: {
    responseMimeType?: string;
    temperature?: number;
  };
}

export interface GeminiGenerateResponse {
  candidates?: Array<{
    content?: {
      parts?: Array<{ text?: string }>;
    };
  }>;
}
