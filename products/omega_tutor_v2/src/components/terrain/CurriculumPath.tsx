/**
 * CurriculumPath — A single curriculum / learning path.
 * Title, description, progress bar, expandable knowledge graph.
 */

import type { Curriculum, TopicProgress } from "../../types/terrain";
import { KnowledgeGraph } from "./KnowledgeGraph";
import { ProgressOverlay } from "./ProgressOverlay";

export interface CurriculumPathProps {
  curriculum: Curriculum;
  progress: TopicProgress[];
  onSelectTopic: (topicId: string, question: string) => void;
  isExpanded: boolean;
  onToggle: () => void;
}

export function CurriculumPath({
  curriculum,
  progress,
  onSelectTopic,
  isExpanded,
  onToggle,
}: CurriculumPathProps) {
  const completed = progress.filter((p) => p.completed).length;
  const total = curriculum.topics.length;

  return (
    <div className="overflow-hidden rounded-lg border border-bg-tertiary bg-bg-secondary">
      <button
        type="button"
        onClick={onToggle}
        className="w-full p-5 text-left transition-colors hover:bg-bg-tertiary"
        aria-expanded={isExpanded}
        aria-controls={`curriculum-${curriculum.id}-graph`}
      >
        <div className="flex items-start justify-between">
          <div className="flex-1">
            <h3 className="text-lg font-medium text-text-primary">
              {curriculum.name}
            </h3>
            <p className="mt-1 text-sm text-text-muted">
              {curriculum.description}
            </p>

            <div className="mt-3 flex items-center gap-3">
              <div className="h-2 flex-1 overflow-hidden rounded-full bg-bg-tertiary">
                <div
                  className="h-2 rounded-full bg-accent transition-all"
                  style={{
                    width: `${total > 0 ? (completed / total) * 100 : 0}%`,
                  }}
                />
              </div>
              <span className="text-sm text-text-muted">
                {completed}/{total} topics
              </span>
            </div>
          </div>

          <div className="ml-4 flex items-center gap-2">
            {curriculum.estimatedHours != null && (
              <span className="text-xs text-text-muted">
                ~{curriculum.estimatedHours}h
              </span>
            )}
            <span className="text-text-muted" aria-hidden>
              {isExpanded ? "▲" : "▼"}
            </span>
          </div>
        </div>
      </button>

      {isExpanded && (
        <div
          id={`curriculum-${curriculum.id}-graph`}
          className="animate-fade-in border-t border-bg-tertiary p-5"
        >
          <ProgressOverlay curriculum={curriculum} progress={progress} />
          <KnowledgeGraph
            curriculum={curriculum}
            progress={progress}
            onSelectTopic={onSelectTopic}
          />
        </div>
      )}
    </div>
  );
}
