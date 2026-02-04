/**
 * OMEGA Tutor v2 — Knowledge terrain types.
 * Curriculum graph, prerequisites, progress.
 */

export type DepthLevel = "intuitive" | "structured" | "technical" | "research";

export interface CurriculumTopic {
  id: string;
  name: string;
  description: string;
  prerequisites: string[];
  depth: DepthLevel;
  estimatedMinutes?: number;
  keyQuestions?: string[];
}

export interface Curriculum {
  id: string;
  name: string;
  description: string;
  topics: CurriculumTopic[];
  estimatedHours?: number;
}

export type TopicStatus = "completed" | "available" | "locked";

export interface TopicProgress {
  topicId: string;
  completed: boolean;
  lastScore?: number;
  lastAttempt?: string;
}
