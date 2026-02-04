/**
 * MasteryGrid — Visual breakdown of knowledge by domain.
 * Professional, muted. Not gamified. Shows topics per domain when provided.
 */

import type { DomainMastery } from "../../types/dashboard";
import type { LearnedTopic } from "../../services/dashboard";

export interface MasteryGridProps {
  domains: DomainMastery[];
  learnedTopics?: LearnedTopic[];
}

function cleanTopicName(topic: string): string {
  return topic
    .replace(/^(what is |how does |why do |explain )/i, "")
    .replace(/\?$/, "")
    .replace(/^["']|["']$/g, "")
    .trim();
}

function formatLastActivity(date: Date | null): string {
  if (!date) return "—";
  const now = new Date();
  const d = new Date(date);
  const days = Math.floor(
    (now.getTime() - d.getTime()) / (24 * 60 * 60 * 1000)
  );
  if (days === 0) return "Today";
  if (days === 1) return "Yesterday";
  if (days < 7) return `${days}d ago`;
  if (days < 30) return `${Math.floor(days / 7)}w ago`;
  return d.toLocaleDateString();
}

export function MasteryGrid({ domains, learnedTopics = [] }: MasteryGridProps) {
  if (domains.length === 0) {
    return (
      <section className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6">
        <h2 className="text-lg font-semibold text-text-primary">
          Mastery by domain
        </h2>
        <p className="mt-2 text-sm text-text-muted">
          No domains yet. Learn topics in the workspace to see progress here.
        </p>
      </section>
    );
  }

  return (
    <section
      className="rounded-lg border border-bg-tertiary bg-bg-secondary p-6"
      aria-labelledby="mastery-heading"
    >
      <h2 id="mastery-heading" className="text-lg font-semibold text-text-primary">
        Mastery by domain
      </h2>
      <div className="mt-4 grid grid-cols-1 gap-4 sm:grid-cols-2 lg:grid-cols-3">
        {domains.map((d) => {
          const pct = Math.min(
            100,
            Math.round((d.topicsLearned / d.topicsTotal) * 100)
          );
          const topicsInDomain = learnedTopics.filter(
            (t) => t.domain === d.name
          );
          return (
            <div
              key={d.name}
              className="rounded-lg border border-bg-tertiary bg-bg-primary p-4"
            >
              <h3 className="font-medium text-text-primary">{d.name}</h3>
              <p className="mt-1 text-sm text-text-muted">
                {d.topicsLearned}/{d.topicsTotal} topics
              </p>
              <div
                className="mt-2 h-1.5 w-full overflow-hidden rounded-full bg-bg-tertiary"
                role="progressbar"
                aria-valuenow={pct}
                aria-valuemin={0}
                aria-valuemax={100}
                aria-label={`${d.name} progress`}
              >
                <div
                  className="h-full rounded-full bg-accent/70 transition-all"
                  style={{ width: `${pct}%` }}
                />
              </div>
              <p className="mt-2 text-xs text-text-muted">
                Avg {d.averageScore}% · {formatLastActivity(d.lastActivity)}
              </p>
              {topicsInDomain.length > 0 && (
                <ul className="mt-3 space-y-1 border-t border-bg-tertiary pt-3">
                  {topicsInDomain.map((t, i) => (
                    <li
                      key={`${t.topic}-${t.learnedAt}-${i}`}
                      className="text-xs text-text-muted"
                    >
                      {cleanTopicName(t.topic)}
                    </li>
                  ))}
                </ul>
              )}
            </div>
          );
        })}
      </div>
    </section>
  );
}
