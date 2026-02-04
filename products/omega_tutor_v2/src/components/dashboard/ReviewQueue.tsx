/**
 * ReviewQueue — Topics due for spaced repetition review.
 * Sorted by urgency. Overdue / due today / upcoming.
 */

import type { ReviewItem } from "../../types/dashboard";

export interface ReviewQueueProps {
  items: ReviewItem[];
  onStartReview: (topic: string) => void;
}

function cleanTopicName(topic: string): string {
  return topic
    .replace(/^(what is |how does |why do |explain )/i, "")
    .replace(/\?$/, "")
    .replace(/^["']|["']$/g, "")
    .trim();
}

function formatDueDate(d: Date): string {
  const now = new Date();
  now.setHours(0, 0, 0, 0);
  const due = new Date(d);
  due.setHours(0, 0, 0, 0);
  const days = Math.ceil((due.getTime() - now.getTime()) / (24 * 60 * 60 * 1000));
  if (days < 0) return `${Math.abs(days)}d overdue`;
  if (days === 0) return "Today";
  if (days === 1) return "Tomorrow";
  return due.toLocaleDateString(undefined, { month: "short", day: "numeric" });
}

export function ReviewQueue({ items, onStartReview }: ReviewQueueProps) {
  const handleStartSession = () => {
    const topic = items[0]?.topic;
    if (topic) onStartReview(topic);
  };

  if (items.length === 0) {
    return (
      <section
        className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6"
        aria-labelledby="review-queue-heading"
      >
        <h2 id="review-queue-heading" className="text-lg font-semibold text-text-primary">
          Due for review
        </h2>
        <p className="mt-2 text-sm text-text-muted">
          No topics due for review. Keep learning!
        </p>
      </section>
    );
  }

  return (
    <section
      className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6"
      aria-labelledby="review-queue-heading"
    >
      <div className="flex flex-wrap items-center justify-between gap-2">
        <h2 id="review-queue-heading" className="text-lg font-semibold text-text-primary">
          Due for review
        </h2>
        <span className="text-sm text-text-muted">
          {items.length} {items.length === 1 ? "item" : "items"}
        </span>
      </div>
      <ul className="mt-4 space-y-2">
        {items.map((r) => {
          const isOverdue = r.isOverdue;
          const isToday =
            !isOverdue &&
            r.dueDate.toDateString() === new Date().toDateString();
          const badge =
            isOverdue
              ? "bg-red-100 text-red-800"
              : isToday
                ? "bg-amber-100 text-amber-800"
                : "bg-bg-tertiary text-text-muted";
          return (
            <li
              key={r.topic}
              className="flex flex-wrap items-center justify-between gap-2 rounded-lg border border-bg-tertiary bg-bg-primary px-3 py-2"
            >
              <div className="min-w-0 flex-1">
                <p className="truncate font-medium text-text-primary">
                  {cleanTopicName(r.topic)}
                </p>
                <span
                  className={`inline-block rounded px-1.5 py-0.5 text-xs ${badge}`}
                >
                  {formatDueDate(r.dueDate)}
                </span>
              </div>
              <button
                type="button"
                onClick={() => onStartReview(r.topic)}
                className="shrink-0 rounded border border-bg-tertiary bg-bg-secondary px-2.5 py-1.5 text-sm font-medium text-text-primary hover:bg-bg-tertiary"
              >
                Review
              </button>
            </li>
          );
        })}
      </ul>
      <div className="mt-4">
        <button
          type="button"
          onClick={handleStartSession}
          className="rounded-lg bg-accent px-4 py-2.5 text-sm font-medium text-white hover:opacity-90"
        >
          Start review session
        </button>
      </div>
    </section>
  );
}
