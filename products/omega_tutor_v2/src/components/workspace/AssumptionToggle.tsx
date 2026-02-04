/**
 * AssumptionToggle — Toggle to reveal hidden assumptions.
 * "Show Assumptions" button; when active, assumptions display.
 */

import { useState } from "react";

export interface AssumptionToggleProps {
  assumptions: string[];
  label?: string;
  labelWhenOpen?: string;
}

export function AssumptionToggle({
  assumptions,
  label = "Show assumptions",
  labelWhenOpen = "Hide assumptions",
}: AssumptionToggleProps) {
  const [open, setOpen] = useState(false);

  if (!assumptions?.length) return null;

  return (
    <div className="my-4">
      <button
        type="button"
        onClick={() => setOpen((o) => !o)}
        className="text-sm font-medium text-accent hover:underline"
        aria-expanded={open}
      >
        {open ? labelWhenOpen : label}
      </button>
      {open && (
        <ul className="mt-3 list-inside list-disc space-y-1.5 border-l-2 border-bg-tertiary pl-4 text-sm text-text-secondary">
          {assumptions.map((a, i) => (
            <li key={i}>{a}</li>
          ))}
        </ul>
      )}
    </div>
  );
}
