/**
 * ProgressOverlay — Overall progress on the terrain view.
 * Total, completed, available, locked. Next recommended.
 */

import type { Curriculum, TopicProgress } from "../../types/terrain";
import { getTopicStatus, getNextRecommended } from "../../services/terrain";

export interface ProgressOverlayProps {
  curriculum: Curriculum;
  progress: TopicProgress[];
}

export function ProgressOverlay({ curriculum, progress }: ProgressOverlayProps) {
  const completed = progress.filter((p) => p.completed).length;
  const total = curriculum.topics.length;
  let available = 0;
  let locked = 0;
  for (const topic of curriculum.topics) {
    const status = getTopicStatus(topic, progress, curriculum.topics);
    if (status === "available") available += 1;
    else if (status === "locked") locked += 1;
  }

  const next = getNextRecommended(curriculum, progress);

  return (
    <div className="flex flex-wrap items-center gap-4 rounded-lg border border-bg-tertiary bg-bg-secondary px-4 py-3 text-sm">
      <span className="text-text-muted">
        <strong className="text-text-primary">{completed}</strong>/{total}{" "}
        topics
      </span>
      {completed > 0 && (
        <span className="text-green-700">
          {completed} completed
        </span>
      )}
      {available > 0 && (
        <span className="text-accent">
          {available} available
        </span>
      )}
      {locked > 0 && (
        <span className="text-text-muted">
          {locked} locked
        </span>
      )}
      {next && (
        <span className="ml-auto font-medium text-text-primary">
          Next: {next.name}
        </span>
      )}
    </div>
  );
}
