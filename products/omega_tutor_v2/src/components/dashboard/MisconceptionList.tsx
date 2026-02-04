/**
 * MisconceptionList — "Cognitive Friction" panel. Recurring misunderstandings.
 * Positioned as insight, not failure. Subtle amber styling.
 */

import { Link } from "react-router-dom";
import type { RecordedMisconception } from "../../types/dashboard";

export interface MisconceptionListProps {
  misconceptions: RecordedMisconception[];
}

function cleanTopicName(topic: string): string {
  return topic
    .replace(/^(what is |how does |why do |explain )/i, "")
    .replace(/\?$/, "")
    .replace(/^["']|["']$/g, "")
    .trim();
}

function formatDate(d: Date): string {
  return d.toLocaleDateString(undefined, {
    month: "short",
    day: "numeric",
    year: d.getFullYear() !== new Date().getFullYear() ? "numeric" : undefined,
  });
}

export function MisconceptionList({ misconceptions }: MisconceptionListProps) {
  if (misconceptions.length === 0) {
    return (
      <section className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6">
        <h2 className="text-lg font-semibold text-text-primary">
          Areas for reinforcement
        </h2>
        <p className="mt-2 text-sm text-text-muted">
          No recorded misconceptions. Explain-backs with misconceptions will appear here.
        </p>
      </section>
    );
  }

  return (
    <section
      className="rounded-lg border border-amber-200/60 bg-amber-50/30 p-6"
      aria-labelledby="misconceptions-heading"
    >
      <h2 id="misconceptions-heading" className="text-lg font-semibold text-text-primary">
        Areas for reinforcement
      </h2>
      <p className="mt-1 text-sm text-text-muted">
        Recurring misunderstandings — useful for targeted review.
      </p>
      <ul className="mt-4 space-y-4">
        {misconceptions.map((m, i) => (
          <li
            key={`${m.topic}-${m.misconception}-${i}`}
            className="rounded-lg border border-amber-200/50 bg-bg-primary p-3"
          >
            <p className="text-xs font-medium text-text-muted">
              {cleanTopicName(m.topic)}
            </p>
            <p className="mt-1 text-sm text-amber-800">
              “{m.misconception}”
            </p>
            <p className="mt-1.5 text-sm text-text-secondary">
              → {m.correction}
            </p>
            <div className="mt-2 flex items-center justify-between gap-2">
              <span className="text-xs text-text-muted">
                {m.occurrences}× · {formatDate(m.lastSeen)}
              </span>
              <Link
                to={`/workspace?topic=${encodeURIComponent(m.topic)}`}
                className="text-xs font-medium text-accent hover:underline"
              >
                Review this concept
              </Link>
            </div>
          </li>
        ))}
      </ul>
    </section>
  );
}
