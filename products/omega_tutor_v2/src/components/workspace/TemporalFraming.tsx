/**
 * TemporalFraming — T1–T4 tabs. Visible only when depth = research.
 */

import { useState } from "react";
import type { TemporalFraming as TemporalFramingType } from "../../types/learning";

const TABS = [
  { id: "t1", label: "T1 (0–1y)", desc: "Current relevance" },
  { id: "t2", label: "T2 (1–5y)", desc: "Near-term" },
  { id: "t3", label: "T3 (5–30y)", desc: "Long-term" },
  { id: "t4", label: "T4 (30–200y)", desc: "Civilizational" },
] as const;

export interface TemporalFramingProps {
  data: TemporalFramingType;
}

export function TemporalFraming({ data }: TemporalFramingProps) {
  const [active, setActive] = useState<"t1" | "t2" | "t3" | "t4">("t1");

  const content =
    data[active] ?? "";

  return (
    <section className="my-8 rounded-lg border border-bg-tertiary bg-bg-secondary">
      <h2 className="mb-3 px-4 pt-4 text-sm font-semibold text-text-primary">
        Temporal framing
      </h2>
      <div className="flex gap-1 border-b border-bg-tertiary px-4">
        {TABS.map((tab) => (
          <button
            key={tab.id}
            type="button"
            onClick={() => setActive(tab.id)}
            title={tab.desc}
            className={`border-b-2 px-3 py-2 text-sm font-medium transition-colors ${
              active === tab.id
                ? "border-accent text-accent"
                : "border-transparent text-text-muted hover:text-text-secondary"
            }`}
          >
            {tab.label}
          </button>
        ))}
      </div>
      <div className="p-4 text-sm text-text-secondary">
        {content}
      </div>
    </section>
  );
}
