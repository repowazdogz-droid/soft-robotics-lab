/**
 * RetentionChart — Retention strength over time / by topic.
 * Simple list, clear information. Not fancy charts.
 */

import type { RetentionItem } from "../../types/dashboard";

export interface RetentionChartProps {
  items: RetentionItem[];
  sortBy?: "nextReview" | "strength";
}

function strengthLabel(strength: number): string {
  if (strength >= 80) return "Strong";
  if (strength >= 50) return "Fading";
  return "Weak";
}

function daysUntil(d: Date): number {
  const now = new Date();
  now.setHours(0, 0, 0, 0);
  const next = new Date(d);
  next.setHours(0, 0, 0, 0);
  return Math.ceil((next.getTime() - now.getTime()) / (24 * 60 * 60 * 1000));
}

function cleanTopicName(topic: string): string {
  return topic
    .replace(/^(what is |how does |why do |explain )/i, "")
    .replace(/\?$/, "")
    .replace(/^["']|["']$/g, "")
    .trim();
}

export function RetentionChart({
  items,
  sortBy = "nextReview",
}: RetentionChartProps) {
  const sorted = [...items].sort((a, b) => {
    if (sortBy === "nextReview")
      return a.nextReview.getTime() - b.nextReview.getTime();
    return a.strength - b.strength;
  });

  if (sorted.length === 0) {
    return (
      <section className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6">
        <h2 className="text-lg font-semibold text-text-primary">
          Retention
        </h2>
        <p className="mt-2 text-sm text-text-muted">
          No items yet. Complete explain-backs to schedule reviews.
        </p>
      </section>
    );
  }

  return (
    <section
      className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6"
      aria-labelledby="retention-heading"
    >
      <h2 id="retention-heading" className="text-lg font-semibold text-text-primary">
        Retention
      </h2>
      <ul className="mt-4 space-y-3">
        {sorted.map((r) => {
          const label = strengthLabel(r.strength);
          const days = daysUntil(r.nextReview);
          const dueText =
            days < 0
              ? `${Math.abs(days)}d overdue`
              : days === 0
                ? "Today"
                : days === 1
                  ? "Tomorrow"
                  : `in ${days}d`;
          return (
            <li
              key={r.topic}
              className="flex flex-wrap items-center justify-between gap-2 rounded-lg border border-bg-tertiary bg-bg-primary px-3 py-2"
            >
              <div className="min-w-0 flex-1">
                <p className="truncate font-medium text-text-primary">
                  {cleanTopicName(r.topic)}
                </p>
                <p className="text-xs text-text-muted">
                  {label} · Review {dueText}
                </p>
              </div>
              <div
                className="h-2 w-16 shrink-0 overflow-hidden rounded-full bg-bg-tertiary"
                role="progressbar"
                aria-valuenow={r.strength}
                aria-valuemin={0}
                aria-valuemax={100}
              >
                <div
                  className={`h-full rounded-full ${
                    r.strength >= 80
                      ? "bg-green-500/70"
                      : r.strength >= 50
                        ? "bg-amber-500/70"
                        : "bg-red-500/50"
                  }`}
                  style={{ width: `${r.strength}%` }}
                />
              </div>
            </li>
          );
        })}
      </ul>
    </section>
  );
}
