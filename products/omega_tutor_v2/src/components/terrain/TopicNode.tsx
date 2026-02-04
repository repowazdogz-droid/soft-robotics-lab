/**
 * TopicNode — Individual topic in the graph.
 * Completed (green), available (blue), locked (grey).
 */

import type { CurriculumTopic } from "../../types/terrain";

export type TopicNodeStatus = "completed" | "available" | "locked";

export interface TopicNodeProps {
  topic: CurriculumTopic;
  status: TopicNodeStatus;
  onSelect: () => void;
  isNextRecommended?: boolean;
}

export function TopicNode({
  topic,
  status,
  onSelect,
  isNextRecommended = false,
}: TopicNodeProps) {
  const isClickable = status === "available";
  const title =
    status === "locked" && topic.prerequisites?.length
      ? `Complete: ${topic.prerequisites.join(", ")}`
      : topic.description;

  const statusStyles = {
    completed:
      "border-green-300 bg-green-50/80 text-green-800 hover:bg-green-50",
    available:
      "border-accent/70 bg-accent/5 text-text-primary hover:bg-accent/10",
    locked:
      "border-bg-tertiary bg-bg-secondary text-text-muted cursor-default opacity-80",
  };

  return (
    <div
      className={`group relative rounded-lg border px-3 py-2 text-sm transition-colors ${statusStyles[status]} ${isNextRecommended ? "ring-2 ring-accent ring-offset-2" : ""}`}
    >
      <button
        type="button"
        onClick={isClickable ? onSelect : undefined}
        disabled={!isClickable}
        title={title}
        className={`w-full text-left ${!isClickable ? "cursor-default" : "cursor-pointer"}`}
      >
        <span className="flex items-center gap-2">
          {status === "completed" && (
            <span className="shrink-0 text-green-600" aria-hidden>
              ✓
            </span>
          )}
          {status === "locked" && (
            <span className="shrink-0 text-text-muted" aria-hidden title={title}>
              🔒
            </span>
          )}
          <span className="truncate font-medium">{topic.name}</span>
        </span>
      </button>
      {isNextRecommended && (
        <span className="absolute -top-1 -right-1 rounded bg-accent px-1.5 py-0.5 text-xs font-medium text-white">
          Next
        </span>
      )}
    </div>
  );
}
