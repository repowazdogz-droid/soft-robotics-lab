/**
 * ExplainBackPrompt — Prompts user to explain in their own words.
 * Learning exercise, not a test. Calm, non-intimidating.
 */

import { useState, useCallback } from "react";

export interface ExplainBackPromptProps {
  topic: string;
  onSubmit: (explanation: string) => void;
  isLoading: boolean;
}

export function ExplainBackPrompt({
  topic,
  onSubmit,
  isLoading,
}: ExplainBackPromptProps) {
  const [value, setValue] = useState("");

  const handleSubmit = useCallback(
    (e: React.FormEvent) => {
      e.preventDefault();
      const trimmed = value.trim();
      if (trimmed && !isLoading) {
        onSubmit(trimmed);
      }
    },
    [value, isLoading, onSubmit]
  );

  return (
    <section
      className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6"
      aria-labelledby="explain-back-heading"
    >
      <h2 id="explain-back-heading" className="text-lg font-semibold text-text-primary">
        Check your understanding
      </h2>
      <p className="mt-1 text-sm text-text-muted">
        Explain <span className="font-medium text-text-secondary">{topic}</span> in your own words.
        This helps consolidate what you’ve learned.
      </p>
      <form onSubmit={handleSubmit} className="mt-5">
        <label htmlFor="explain-back-textarea" className="sr-only">
          Your explanation
        </label>
        <textarea
          id="explain-back-textarea"
          value={value}
          onChange={(e) => setValue(e.target.value)}
          placeholder="Write what you understood…"
          disabled={isLoading}
          rows={5}
          className="w-full resize-y rounded-lg border border-bg-tertiary bg-bg-primary px-4 py-3 text-base text-text-primary placeholder:text-text-muted focus:border-accent focus:outline-none focus:ring-1 focus:ring-accent disabled:opacity-60"
        />
        <div className="mt-4">
          <button
            type="submit"
            disabled={isLoading || !value.trim()}
            className="rounded-lg border border-bg-tertiary bg-bg-primary px-4 py-2.5 text-sm font-medium text-text-primary transition hover:bg-bg-tertiary disabled:opacity-50"
          >
            {isLoading ? "Checking…" : "Check my understanding"}
          </button>
        </div>
      </form>
    </section>
  );
}
