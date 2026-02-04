/**
 * QuestionInput — Focused question input. Not a chat box.
 * Single text input + submit; calls teaching service on submit.
 */

import { useState, useCallback } from "react";

export interface QuestionInputProps {
  onSubmit: (question: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

export function QuestionInput({
  onSubmit,
  disabled = false,
  placeholder = "What do you want to understand today?",
}: QuestionInputProps) {
  const [value, setValue] = useState("");

  const handleSubmit = useCallback(
    (e: React.FormEvent) => {
      e.preventDefault();
      const trimmed = value.trim();
      if (trimmed && !disabled) {
        onSubmit(trimmed);
        setValue("");
      }
    },
    [value, disabled, onSubmit]
  );

  return (
    <form onSubmit={handleSubmit} className="flex w-full flex-col gap-3 sm:flex-row">
      <input
        type="text"
        value={value}
        onChange={(e) => setValue(e.target.value)}
        placeholder={placeholder}
        disabled={disabled}
        className="min-w-0 flex-1 rounded-lg border border-bg-tertiary bg-bg-primary px-4 py-3 text-base text-text-primary placeholder:text-text-muted focus:border-accent focus:outline-none focus:ring-1 focus:ring-accent disabled:opacity-60 w-full sm:w-auto"
        aria-label="Question"
      />
      <button
        type="submit"
        disabled={disabled || !value.trim()}
        className="shrink-0 rounded-lg bg-accent px-5 py-3 font-medium text-white transition opacity hover:opacity-90 disabled:opacity-50"
      >
        Ask
      </button>
    </form>
  );
}
