/**
 * OMEGA Tutor v2 — Terrain service. Curriculum progress in localStorage.
 */

import type { Curriculum, CurriculumTopic, TopicProgress } from "../types/terrain";
import { SOFT_ROBOTICS_101 } from "../data/curricula/softRobotics101";
import { MACHINE_LEARNING_101 } from "../data/curricula/machineLearning101";
import { SYNTHETIC_BIOLOGY_101 } from "../data/curricula/syntheticBiology101";

const STORAGE_KEY = "omega_tutor_curriculum_progress";

export function getAllCurricula(): Curriculum[] {
  return [SOFT_ROBOTICS_101, MACHINE_LEARNING_101, SYNTHETIC_BIOLOGY_101];
}

export function getCurriculum(id: string): Curriculum | undefined {
  return getAllCurricula().find((c) => c.id === id);
}

export function getProgress(curriculumId: string): TopicProgress[] {
  if (typeof localStorage === "undefined") return [];
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    const all = raw ? (JSON.parse(raw) as Record<string, TopicProgress[]>) : {};
    return all[curriculumId] ?? [];
  } catch {
    return [];
  }
}

export function markTopicComplete(
  curriculumId: string,
  topicId: string,
  score: number
): void {
  if (typeof localStorage === "undefined") return;
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    const all = raw ? (JSON.parse(raw) as Record<string, TopicProgress[]>) : {};
    const progress = all[curriculumId] ?? [];
    const existing = progress.find((p) => p.topicId === topicId);
    if (existing) {
      existing.completed = true;
      existing.lastScore = score;
      existing.lastAttempt = new Date().toISOString();
    } else {
      progress.push({
        topicId,
        completed: true,
        lastScore: score,
        lastAttempt: new Date().toISOString(),
      });
    }
    all[curriculumId] = progress;
    localStorage.setItem(STORAGE_KEY, JSON.stringify(all));
  } catch {
    // ignore
  }
}

export function getTopicStatus(
  topic: CurriculumTopic,
  progress: TopicProgress[],
  _allTopics: CurriculumTopic[]
): "completed" | "available" | "locked" {
  const topicProgress = progress.find((p) => p.topicId === topic.id);
  if (topicProgress?.completed) return "completed";

  const prereqsMet = topic.prerequisites.every((prereqId) => {
    const prereqProgress = progress.find((p) => p.topicId === prereqId);
    return prereqProgress?.completed;
  });

  return prereqsMet ? "available" : "locked";
}

export function getNextRecommended(
  curriculum: Curriculum,
  progress: TopicProgress[]
): CurriculumTopic | null {
  for (const topic of curriculum.topics) {
    const status = getTopicStatus(topic, progress, curriculum.topics);
    if (status === "available") return topic;
  }
  return null;
}
