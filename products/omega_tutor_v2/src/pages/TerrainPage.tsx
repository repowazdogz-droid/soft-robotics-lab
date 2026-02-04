/**
 * TerrainPage — Knowledge terrain. Navigable curriculum graph.
 * CurriculumPath, KnowledgeGraph, TopicNode, ProgressOverlay.
 */

import { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { CurriculumPath } from "../components/terrain/CurriculumPath";
import { getAllCurricula, getProgress, getTopicStatus } from "../services/terrain";
import type { Curriculum, TopicProgress } from "../types/terrain";

export function TerrainPage() {
  const navigate = useNavigate();
  const [curricula, setCurricula] = useState<Curriculum[]>([]);
  const [progressMap, setProgressMap] = useState<Record<string, TopicProgress[]>>(
    {}
  );
  const [expandedId, setExpandedId] = useState<string | null>(null);

  useEffect(() => {
    const allCurricula = getAllCurricula();
    setCurricula(allCurricula);
    const progress: Record<string, TopicProgress[]> = {};
    allCurricula.forEach((c) => {
      progress[c.id] = getProgress(c.id);
    });
    setProgressMap(progress);
  }, []);

  const totalCompleted = Object.values(progressMap)
    .flat()
    .filter((p) => p.completed).length;
  const totalAvailable = curricula.reduce((sum, c) => {
    const progress = progressMap[c.id] ?? [];
    return (
      sum +
      c.topics.filter(
        (t) =>
          getTopicStatus(t, progress, c.topics) === "available"
      ).length
    );
  }, 0);
  const totalLocked = curricula.reduce((sum, c) => {
    const progress = progressMap[c.id] ?? [];
    return (
      sum +
      c.topics.filter(
        (t) => getTopicStatus(t, progress, c.topics) === "locked"
      ).length
    );
  }, 0);

  const handleSelectTopic = (
    curriculumId: string,
    topicId: string,
    question: string
  ) => {
    navigate("/workspace", {
      state: {
        initialQuestion: question,
        fromCurriculum: curriculumId,
        topicId,
      },
    });
  };

  const handleToggle = (id: string) => {
    setExpandedId((prev) => (prev === id ? null : id));
  };

  return (
    <div className="mx-auto max-w-4xl space-y-8 p-6">
      <header className="border-b border-bg-tertiary pb-6">
        <h1 className="text-2xl font-semibold text-text-primary">
          Knowledge Terrain
        </h1>
        <p className="mt-2 max-w-2xl text-text-secondary">
          Structured learning paths with prerequisites and progression. Each path
          builds understanding systematically — complete topics to unlock the
          next.
        </p>
      </header>

      <div className="flex items-start gap-4 rounded-lg bg-bg-secondary p-4">
        <div className="text-2xl">🗺️</div>
        <div className="text-sm">
          <p className="font-medium text-text-primary">
            How learning paths work
          </p>
          <p className="mt-1 text-text-muted">
            Each path contains topics in sequence. Complete a topic by learning
            it and explaining it back. Prerequisites unlock automatically. Your
            progress is saved.
          </p>
        </div>
      </div>

      <div className="grid grid-cols-3 gap-4">
        <div className="rounded-lg bg-bg-secondary p-4 text-center">
          <div className="text-2xl font-semibold text-text-primary">
            {totalCompleted}
          </div>
          <div className="text-sm text-text-muted">Topics completed</div>
        </div>
        <div className="rounded-lg bg-bg-secondary p-4 text-center">
          <div className="text-2xl font-semibold text-accent">
            {totalAvailable}
          </div>
          <div className="text-sm text-text-muted">Available now</div>
        </div>
        <div className="rounded-lg bg-bg-secondary p-4 text-center">
          <div className="text-2xl font-semibold text-text-muted">
            {totalLocked}
          </div>
          <div className="text-sm text-text-muted">Locked</div>
        </div>
      </div>

      <section className="space-y-4">
        <h2 className="text-lg font-medium text-text-primary">
          Learning Paths
        </h2>

        <div className="space-y-4">
          {curricula.map((curriculum) => (
            <CurriculumPath
              key={curriculum.id}
              curriculum={curriculum}
              progress={progressMap[curriculum.id] ?? []}
              onSelectTopic={(topicId, question) =>
                handleSelectTopic(curriculum.id, topicId, question)
              }
              isExpanded={expandedId === curriculum.id}
              onToggle={() => handleToggle(curriculum.id)}
            />
          ))}
        </div>
      </section>

      <section className="border-t border-bg-tertiary pt-6">
        <div className="rounded-lg bg-bg-secondary p-4 text-center">
          <p className="text-sm text-text-muted">
            More paths coming soon. Want a custom learning path?{" "}
            <button
              type="button"
              className="ml-1 text-accent hover:underline"
            >
              Request one
            </button>
          </p>
        </div>
      </section>
    </div>
  );
}
