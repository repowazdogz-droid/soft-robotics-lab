/**
 * MisconceptionAlert — Inline alert for common misconceptions.
 * Amber/warning styling (muted). Shows relevant misconception from response or DB.
 */

import { useState } from "react";

export interface MisconceptionAlertProps {
  message: string;
  correctUnderstanding?: string;
  className?: string;
}

export function MisconceptionAlert({
  message,
  correctUnderstanding,
  className = "",
}: MisconceptionAlertProps) {
  const [expanded, setExpanded] = useState(false);

  return (
    <div
      role="alert"
      className={`rounded-lg border border-amber-200 bg-amber-50/80 px-4 py-3 text-sm text-amber-900 ${className}`}
    >
      <p className="font-medium">Common misconception</p>
      <p className="mt-1 text-amber-800">{message}</p>
      {correctUnderstanding && (
        <>
          <button
            type="button"
            onClick={() => setExpanded((e) => !e)}
            className="mt-2 text-xs font-medium text-amber-700 hover:underline"
          >
            {expanded ? "Hide correction" : "Show correct understanding"}
          </button>
          {expanded && (
            <p className="mt-2 border-t border-amber-200 pt-2 text-amber-800">
              {correctUnderstanding}
            </p>
          )}
        </>
      )}
    </div>
  );
}
