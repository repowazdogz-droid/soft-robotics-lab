/**
 * SettingsPage — API key, data management, about.
 */

import { useState, useEffect } from "react";

const STORAGE_API_KEY = "omega_tutor_api_key";

export function SettingsPage() {
  const [apiKey, setApiKey] = useState("");
  const [saved, setSaved] = useState(false);

  useEffect(() => {
    if (typeof localStorage === "undefined") return;
    const stored = localStorage.getItem(STORAGE_API_KEY);
    if (stored) setApiKey(stored);
  }, []);

  const handleSave = () => {
    if (typeof localStorage === "undefined") return;
    localStorage.setItem(STORAGE_API_KEY, apiKey);
    setSaved(true);
    setTimeout(() => setSaved(false), 2000);
  };

  const handleClearData = () => {
    if (
      !confirm(
        "Clear all learning progress? This cannot be undone."
      )
    )
      return;
    if (typeof localStorage === "undefined") return;
    localStorage.removeItem("omega_tutor_learned_topics");
    localStorage.removeItem("omega_tutor_review_schedule");
    localStorage.removeItem("omega_tutor_misconceptions");
    localStorage.removeItem("omega_tutor_curriculum_progress");
    alert("Learning data cleared.");
  };

  return (
    <div className="mx-auto max-w-2xl px-4 py-8">
      <header className="mb-8">
        <h1 className="text-2xl font-semibold text-text-primary">
          Settings
        </h1>
      </header>

      <section className="mb-8 space-y-4">
        <h2 className="text-lg font-medium text-text-primary">
          API Configuration
        </h2>
        <div>
          <label
            htmlFor="settings-api-key"
            className="mb-1 block text-sm text-text-secondary"
          >
            Gemini API Key
          </label>
          <input
            id="settings-api-key"
            type="password"
            value={apiKey}
            onChange={(e) => setApiKey(e.target.value)}
            placeholder="Enter your Gemini API key"
            className="w-full rounded-lg border border-bg-tertiary px-3 py-2 focus:border-accent focus:outline-none focus:ring-2 focus:ring-accent"
            autoComplete="off"
          />
          <p className="mt-1 text-xs text-text-muted">
            Get a key from Google AI Studio. Stored locally in your browser.
          </p>
        </div>
        <button
          type="button"
          onClick={handleSave}
          className="rounded-lg bg-accent px-4 py-2 text-white transition-colors hover:bg-accent/90 focus:outline-none focus:ring-2 focus:ring-accent focus:ring-offset-2"
        >
          {saved ? "Saved!" : "Save"}
        </button>
      </section>

      <section className="mb-8 space-y-4">
        <h2 className="text-lg font-medium text-text-primary">
          Data Management
        </h2>
        <p className="text-sm text-text-secondary">
          All your learning data is stored locally in your browser.
        </p>
        <button
          type="button"
          onClick={handleClearData}
          className="rounded-lg border border-red-300 px-4 py-2 text-red-700 transition-colors hover:bg-red-50 focus:outline-none focus:ring-2 focus:ring-red-500 focus:ring-offset-2"
        >
          Clear All Learning Data
        </button>
      </section>

      <section className="space-y-4">
        <h2 className="text-lg font-medium text-text-primary">About</h2>
        <div className="space-y-2 text-sm text-text-secondary">
          <p>
            <strong className="text-text-primary">OMEGA Tutor</strong> — Cognitive
            Learning System
          </p>
          <p>
            An adaptive teaching environment with explain-back comprehension,
            misconception detection, and spaced repetition.
          </p>
          <p className="text-text-muted">Part of the OMEGA Stack</p>
        </div>
      </section>
    </div>
  );
}
