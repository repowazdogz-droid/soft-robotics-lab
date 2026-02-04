/**
 * DepthDial — Visual depth selector. Four levels.
 * Horizontal segmented control; current level persists in session.
 */

import { useCallback } from "react";
import type { DepthLevel } from "../../types/learning";

const DEPTH_LEVELS: DepthLevel[] = [
  "intuitive",
  "structured",
  "technical",
  "research",
];

const LABELS: Record<DepthLevel, string> = {
  intuitive: "Intuitive",
  structured: "Structured",
  technical: "Technical",
  research: "Research",
};

const SESSION_KEY = "omega_tutor_depth_level";

export interface DepthDialProps {
  value: DepthLevel;
  onChange: (level: DepthLevel) => void;
  disabled?: boolean;
}

export function DepthDial({ value, onChange, disabled = false }: DepthDialProps) {
  const handleSelect = useCallback(
    (level: DepthLevel) => {
      if (disabled) return;
      onChange(level);
      sessionStorage.setItem(SESSION_KEY, level);
    },
    [onChange, disabled]
  );

  return (
    <div
      className="inline-flex flex-wrap gap-1 rounded-lg border border-bg-tertiary bg-bg-secondary p-1"
      role="group"
      aria-label="Explanation depth"
      aria-busy={disabled}
    >
      {DEPTH_LEVELS.map((level) => (
        <button
          key={level}
          type="button"
          onClick={() => handleSelect(level)}
          disabled={disabled}
          title={
            level === "intuitive"
              ? "Simple language, everyday analogies"
              : level === "structured"
                ? "Clear structure, key terms, practical use"
                : level === "technical"
                  ? "Full technical depth, mechanisms, math"
                  : "Research level: open problems, T1–T4 framing"
          }
          className={`relative rounded-md px-4 py-2 text-sm font-medium transition-colors ${
            value === level
              ? "bg-bg-primary text-text-primary shadow-sm"
              : "text-text-muted hover:bg-bg-tertiary hover:text-text-secondary"
          } ${disabled ? "cursor-not-allowed opacity-60" : ""}`}
        >
          {LABELS[level]}
        </button>
      ))}
    </div>
  );
}
