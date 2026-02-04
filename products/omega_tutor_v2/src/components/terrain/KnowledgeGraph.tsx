/**
 * KnowledgeGraph — Visual representation of curriculum as connected nodes.
 * Tree/hierarchy layout. Nodes = topics, edges = prerequisites.
 */

import { useMemo, Fragment } from "react";
import type { Curriculum, CurriculumTopic, TopicProgress } from "../../types/terrain";
import { getTopicStatus, getNextRecommended } from "../../services/terrain";
import { TopicNode } from "./TopicNode";

export interface KnowledgeGraphProps {
  curriculum: Curriculum;
  progress: TopicProgress[];
  onSelectTopic: (topicId: string, question: string) => void;
}

/** Lay out topics in rows by prerequisite depth (level 0 = no prereqs, etc.) */
function layoutByLevel(topics: CurriculumTopic[]): CurriculumTopic[][] {
  const levels: CurriculumTopic[][] = [];
  const assigned = new Set<string>();

  let remaining = [...topics];
  while (remaining.length > 0) {
    const level = remaining.filter((t) =>
      t.prerequisites.every((p) => assigned.has(p))
    );
    if (level.length === 0) break;
    levels.push(level);
    level.forEach((t) => assigned.add(t.id));
    remaining = remaining.filter((t) => !assigned.has(t.id));
  }
  if (remaining.length > 0) levels.push(remaining);
  return levels;
}

export function KnowledgeGraph({
  curriculum,
  progress,
  onSelectTopic,
}: KnowledgeGraphProps) {
  const levels = useMemo(
    () => layoutByLevel(curriculum.topics),
    [curriculum.topics]
  );
  const nextRecommended = getNextRecommended(curriculum, progress);

  return (
    <div
      className="space-y-4 py-4"
      role="graph"
      aria-label={`Knowledge graph: ${curriculum.name}`}
    >
      {levels.map((row, rowIndex) => (
        <Fragment key={rowIndex}>
          {rowIndex > 0 && (
            <hr className="border-bg-tertiary" aria-hidden />
          )}
          <div className="flex flex-wrap gap-3">
            {row.map((topic) => {
              const status = getTopicStatus(
                topic,
                progress,
                curriculum.topics
              );
              const question =
                topic.keyQuestions?.[0] ?? topic.name;
              return (
                <TopicNode
                  key={topic.id}
                  topic={topic}
                  status={status}
                  onSelect={() => onSelectTopic(topic.id, question)}
                  isNextRecommended={nextRecommended?.id === topic.id}
                />
              );
            })}
          </div>
        </Fragment>
      ))}
    </div>
  );
}
