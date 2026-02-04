/**
 * ExplainBackFeedback — Structured feedback from explain-back evaluation.
 * Mentor-like, precise, constructive. Not gamified.
 */

import type { ExplainBackResult } from "../../types/learning";

function scoreLabel(score: number): string {
  if (score >= 80) return "Strong understanding";
  if (score >= 60) return "Good foundation";
  if (score >= 40) return "Partial understanding";
  return "Needs review";
}

export interface ExplainBackFeedbackProps {
  result: ExplainBackResult;
  onContinue: () => void;
  onRetry: () => void;
}

export function ExplainBackFeedback({
  result,
  onContinue,
  onRetry,
}: ExplainBackFeedbackProps) {
  const label = scoreLabel(result.score);

  return (
    <section
      className="animate-fade-in rounded-lg border border-bg-tertiary bg-bg-secondary p-6"
      aria-labelledby="feedback-heading"
    >
      <h2 id="feedback-heading" className="text-lg font-semibold text-text-primary">
        Feedback
      </h2>
      <p className="mt-1 text-sm text-text-muted" aria-live="polite">
        {label}
      </p>

      <div className="mt-6 space-y-5">
        {result.accurate?.length > 0 && (
          <div>
            <h3 className="text-sm font-medium text-green-700">✓ Accurate</h3>
            <ul className="mt-1.5 list-inside list-disc space-y-1 rounded-md bg-green-50 px-3 py-2 text-sm text-green-800">
              {result.accurate.map((item, i) => (
                <li key={i}>{item}</li>
              ))}
            </ul>
          </div>
        )}

        {result.missing?.length > 0 && (
          <div>
            <h3 className="text-sm font-medium text-amber-700">△ Missing</h3>
            <ul className="mt-1.5 list-inside list-disc space-y-1 rounded-md bg-amber-50 px-3 py-2 text-sm text-amber-800">
              {result.missing.map((item, i) => (
                <li key={i}>{item}</li>
              ))}
            </ul>
          </div>
        )}

        {result.misconceptions?.length > 0 && (
          <div>
            <h3 className="text-sm font-medium text-red-700">✗ Misconceptions</h3>
            <ul className="mt-1.5 list-inside list-disc space-y-1 rounded-md bg-red-50 px-3 py-2 text-sm text-red-800">
              {result.misconceptions.map((item, i) => (
                <li key={i}>{item}</li>
              ))}
            </ul>
          </div>
        )}

        {result.correction?.trim() && (
          <div>
            <h3 className="text-sm font-medium text-text-primary">Correction</h3>
            <p className="mt-1.5 rounded-md bg-bg-primary px-3 py-2 text-sm text-text-secondary">
              {result.correction}
            </p>
          </div>
        )}

        {result.reframing?.trim() && (
          <div>
            <h3 className="text-sm font-medium text-text-primary">Another way to see it</h3>
            <p className="mt-1.5 rounded-md bg-bg-primary px-3 py-2 text-sm text-text-secondary">
              {result.reframing}
            </p>
          </div>
        )}

        {result.deeperQuestion?.trim() && (
          <div className="rounded-md border border-bg-tertiary bg-bg-primary px-3 py-3">
            <h3 className="text-sm font-medium text-text-primary">To go deeper</h3>
            <p className="mt-1.5 text-sm text-text-secondary italic">
              {result.deeperQuestion}
            </p>
          </div>
        )}
      </div>

      <div className="mt-6 flex gap-3">
        <button
          type="button"
          onClick={onContinue}
          className="rounded-lg bg-accent px-4 py-2.5 text-sm font-medium text-white hover:opacity-90"
        >
          Continue learning
        </button>
        <button
          type="button"
          onClick={onRetry}
          className="rounded-lg border border-bg-tertiary bg-bg-primary px-4 py-2.5 text-sm font-medium text-text-primary hover:bg-bg-tertiary"
        >
          Try again
        </button>
      </div>
    </section>
  );
}
