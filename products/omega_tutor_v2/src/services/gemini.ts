/**
 * OMEGA Tutor v2 — Gemini API. Same pattern as Spine Case.
 * Use /api/generate proxy when deployed so API key stays server-side.
 */

const GEMINI_API_URL =
  "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent";

const STORAGE_API_KEY = "omega_tutor_api_key";

function getApiKey(override?: string): string | undefined {
  if (override) return override;
  if (typeof import.meta.env !== "undefined" && (import.meta.env.VITE_GEMINI_API_KEY as string))
    return import.meta.env.VITE_GEMINI_API_KEY as string;
  if (typeof localStorage !== "undefined") {
    const stored = localStorage.getItem(STORAGE_API_KEY);
    if (stored) return stored;
  }
  return undefined;
}

/** Call proxy (Vercel) or direct Gemini. apiKey from override, env, or localStorage. */
export async function generateContent(
  payload: {
    contents: Array<{ role: string; parts: Array<{ text: string }> }>;
    generationConfig?: { responseMimeType?: string; temperature?: number };
  },
  apiKey?: string
): Promise<string> {
  const key = getApiKey(apiKey);
  const useProxy = !key;
  const url = useProxy ? "/api/generate" : `${GEMINI_API_URL}?key=${key}`;

  let response: Response;
  try {
    response = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
  } catch (err) {
    const msg = err instanceof Error ? err.message : "Network error";
    throw new Error(`Request failed: ${msg}. Check your connection.`);
  }

  if (!response.ok) {
    const err = await response.json().catch(() => ({}));
    const msg =
      (err as { error?: string }).error ||
      (err as { message?: string }).message ||
      `Request failed (${response.status})`;
    const hint =
      response.status === 404
        ? " Local dev: set VITE_GEMINI_API_KEY in .env or run 'vercel dev' so /api/generate is available."
        : "";
    throw new Error(msg + hint);
  }

  const data = await response.json();
  const text = data.candidates?.[0]?.content?.parts?.[0]?.text;
  if (text == null) {
    const errMsg =
      data.error?.message || data.candidates?.[0]?.finishReason || "No content in response";
    throw new Error(String(errMsg));
  }
  return text;
}
