/**
 * LearnedTopicsList — Learning history: topic, domain, score, date.
 * Clean topic names; Review button per row.
 */

export interface LearnedTopic {
  topic: string;
  domain: string;
  score: number;
  learnedAt: string;
}

export interface LearnedTopicsListProps {
  topics: LearnedTopic[];
  onReview: (topic: string) => void;
}

function cleanTopicName(topic: string): string {
  return topic
    .replace(/^(what is |how does |why do |explain )/i, "")
    .replace(/\?$/, "")
    .replace(/^["']|["']$/g, "")
    .trim();
}

function formatDate(iso: string): string {
  const d = new Date(iso);
  return d.toLocaleDateString(undefined, {
    month: "short",
    day: "numeric",
    year: d.getFullYear() !== new Date().getFullYear() ? "numeric" : undefined,
  });
}

export function LearnedTopicsList({ topics, onReview }: LearnedTopicsListProps) {
  if (topics.length === 0) {
    return (
      <p className="text-sm text-text-muted">No topics learned yet.</p>
    );
  }

  const sorted = [...topics].sort(
    (a, b) => new Date(b.learnedAt).getTime() - new Date(a.learnedAt).getTime()
  );

  return (
    <div className="space-y-2">
      {sorted.map((t, i) => (
        <div
          key={`${t.topic}-${t.learnedAt}-${i}`}
          className="flex items-center justify-between rounded-lg bg-bg-secondary p-3"
        >
          <div className="min-w-0 flex-1">
            <p className="font-medium text-text-primary">
              {cleanTopicName(t.topic)}
            </p>
            <p className="text-xs text-text-muted">
              {t.domain} · Score: {t.score}% · {formatDate(t.learnedAt)}
            </p>
          </div>
          <button
            type="button"
            onClick={() => onReview(t.topic)}
            className="shrink-0 text-sm text-accent hover:underline"
          >
            Review
          </button>
        </div>
      ))}
    </div>
  );
}
