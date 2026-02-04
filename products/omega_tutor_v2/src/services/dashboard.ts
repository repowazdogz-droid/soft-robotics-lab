/**
 * OMEGA Tutor v2 — Dashboard data service.
 * Persists learning, review schedule, misconceptions in localStorage.
 */

import { updateCard } from "../utils/sm2";
import type {
  DomainMastery,
  RetentionItem,
  RecordedMisconception,
  ReviewItem,
} from "../types/dashboard";

const STORAGE_KEYS = {
  LEARNED_TOPICS: "omega_tutor_learned_topics",
  REVIEW_SCHEDULE: "omega_tutor_review_schedule",
  MISCONCEPTIONS: "omega_tutor_misconceptions",
};

interface LearnedTopicRecord {
  topic: string;
  domain: string;
  score: number;
  learnedAt: string;
}

interface ReviewScheduleRecord {
  topic: string;
  domain: string;
  repetitions: number;
  easiness: number;
  interval: number;
  lastReviewDate: string;
  nextReviewDate: string;
}

interface StoredMisconception {
  topic: string;
  misconception: string;
  correction: string;
  occurrences: number;
  lastSeen: string;
}

function getStorage<T>(key: string, defaultVal: T): T {
  if (typeof localStorage === "undefined") return defaultVal;
  try {
    const raw = localStorage.getItem(key);
    return raw ? (JSON.parse(raw) as T) : defaultVal;
  } catch {
    return defaultVal;
  }
}

function setStorage(key: string, value: unknown): void {
  if (typeof localStorage === "undefined") return;
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch {
    // ignore
  }
}

/** Map explain-back score 0–100 to SM-2 quality 0–5 */
function scoreToQuality(score: number): number {
  const q = Math.round((score / 100) * 5);
  return Math.max(0, Math.min(5, q));
}

/** Derive display strength 0–100 from SM-2 state */
function strengthFromSM2(repetitions: number, easiness: number): number {
  const base = 30 + repetitions * 12 + (easiness - 1.3) * 25;
  return Math.round(Math.min(100, Math.max(0, base)));
}

function detectDomain(topic: string): string {
  const topicLower = topic.toLowerCase();
  if (
    topicLower.includes("robot") ||
    topicLower.includes("actuator") ||
    topicLower.includes("gripper") ||
    topicLower.includes("morphological")
  ) {
    return "Robotics";
  }
  if (
    topicLower.includes("neural") ||
    topicLower.includes("machine learning") ||
    topicLower.includes(" ai ") ||
    topicLower.includes("deep learning")
  ) {
    return "Machine Learning";
  }
  if (
    topicLower.includes("trauma") ||
    topicLower.includes("attachment") ||
    topicLower.includes("regulation") ||
    topicLower.includes("psychology")
  ) {
    return "Psychology";
  }
  if (
    topicLower.includes("biology") ||
    topicLower.includes("gene") ||
    topicLower.includes("cell") ||
    topicLower.includes("synthetic")
  ) {
    return "Biology";
  }
  if (
    topicLower.includes("physics") ||
    topicLower.includes("quantum") ||
    topicLower.includes("mechanics")
  ) {
    return "Physics";
  }
  if (
    topicLower.includes("spaced repetition") ||
    topicLower.includes("memory") ||
    topicLower.includes("retention") ||
    topicLower.includes("learning science")
  ) {
    return "Learning Science";
  }
  if (
    topicLower.includes("cognitive") ||
    topicLower.includes("type 1") ||
    topicLower.includes("type 2") ||
    topicLower.includes("thinking")
  ) {
    return "Cognitive Science";
  }
  if (
    topicLower.includes("education") ||
    topicLower.includes("teaching") ||
    topicLower.includes("practice")
  ) {
    return "Education";
  }
  return "General Knowledge";
}

export interface LearnedTopic {
  topic: string;
  domain: string;
  score: number;
  learnedAt: string;
}

export function getLearnedTopics(): LearnedTopic[] {
  const stored = getStorage<LearnedTopicRecord[]>(
    STORAGE_KEYS.LEARNED_TOPICS,
    []
  );
  return stored.map((r) => ({
    topic: r.topic,
    domain: r.domain,
    score: r.score,
    learnedAt: r.learnedAt,
  }));
}

export function recordLearning(
  topic: string,
  domain: string | undefined,
  score: number
): void {
  const actualDomain =
    domain && domain !== "general" ? domain : detectDomain(topic);
  const learned: LearnedTopicRecord[] = getStorage(
    STORAGE_KEYS.LEARNED_TOPICS,
    []
  );
  learned.push({
    topic,
    domain: actualDomain,
    score,
    learnedAt: new Date().toISOString(),
  });
  setStorage(STORAGE_KEYS.LEARNED_TOPICS, learned);
  scheduleReview(topic, actualDomain, score);
}

export function scheduleReview(topic: string, domain: string, qualityOrScore: number): void {
  const quality = qualityOrScore <= 5 ? qualityOrScore : scoreToQuality(qualityOrScore);
  const schedule: ReviewScheduleRecord[] = getStorage(STORAGE_KEYS.REVIEW_SCHEDULE, []);
  const existing = schedule.find((r) => r.topic === topic);
  const now = new Date().toISOString().slice(0, 10);
  const prev = existing
    ? {
        repetitions: existing.repetitions,
        easiness: existing.easiness,
        previousInterval: existing.interval,
      }
    : { repetitions: 0, easiness: 2.5, previousInterval: 0 };

  const result = updateCard(
    quality >= 3,
    quality,
    prev.repetitions,
    prev.easiness,
    prev.previousInterval
  );

  const next: ReviewScheduleRecord = {
    topic,
    domain,
    repetitions: result.repetitions,
    easiness: result.easiness,
    interval: result.interval,
    lastReviewDate: now,
    nextReviewDate: result.nextReviewDate,
  };

  const updated = schedule.filter((r) => r.topic !== topic);
  updated.push(next);
  setStorage(STORAGE_KEYS.REVIEW_SCHEDULE, updated);
}

export function recordMisconception(
  topic: string,
  misconception: string,
  correction: string
): void {
  const list: StoredMisconception[] = getStorage(STORAGE_KEYS.MISCONCEPTIONS, []);
  const match = list.find(
    (m) => m.topic === topic && m.misconception === misconception
  );
  const now = new Date().toISOString();
  if (match) {
    match.occurrences += 1;
    match.lastSeen = now;
    match.correction = correction;
  } else {
    list.push({
      topic,
      misconception,
      correction,
      occurrences: 1,
      lastSeen: now,
    });
  }
  setStorage(STORAGE_KEYS.MISCONCEPTIONS, list);
}

export function getDomainMastery(): DomainMastery[] {
  const learned: LearnedTopicRecord[] = getStorage(STORAGE_KEYS.LEARNED_TOPICS, []);
  const byDomain = new Map<
    string,
    { scores: number[]; dates: string[] }
  >();
  for (const r of learned) {
    const d = byDomain.get(r.domain) ?? { scores: [], dates: [] };
    d.scores.push(r.score);
    d.dates.push(r.learnedAt);
    byDomain.set(r.domain, d);
  }
  return Array.from(byDomain.entries()).map(([name, data]) => {
    const topicsLearned = data.scores.length;
    const averageScore =
      topicsLearned > 0
        ? Math.round(
            data.scores.reduce((a, b) => a + b, 0) / topicsLearned
          )
        : 0;
    const lastActivity =
      data.dates.length > 0
        ? new Date(data.dates.sort().reverse()[0])
        : null;
    return {
      name,
      topicsLearned,
      topicsTotal: Math.max(topicsLearned, 1),
      averageScore,
      lastActivity,
    };
  });
}

export function getRetentionItems(): RetentionItem[] {
  const schedule: ReviewScheduleRecord[] = getStorage(
    STORAGE_KEYS.REVIEW_SCHEDULE,
    []
  );
  return schedule.map((r) => ({
    topic: r.topic,
    domain: r.domain,
    strength: strengthFromSM2(r.repetitions, r.easiness),
    lastReview: new Date(r.lastReviewDate),
    nextReview: new Date(r.nextReviewDate),
    easeFactor: r.easiness,
    interval: r.interval,
    repetitions: r.repetitions,
  }));
}

export function getRecordedMisconceptions(): RecordedMisconception[] {
  const list: StoredMisconception[] = getStorage(STORAGE_KEYS.MISCONCEPTIONS, []);
  return list
    .map((m) => ({
      topic: m.topic,
      misconception: m.misconception,
      correction: m.correction,
      occurrences: m.occurrences,
      lastSeen: new Date(m.lastSeen),
    }))
    .sort((a, b) => b.lastSeen.getTime() - a.lastSeen.getTime());
}

export function getDueReviews(): ReviewItem[] {
  const items = getRetentionItems();
  const now = new Date();
  now.setHours(0, 0, 0, 0);
  const due = items
    .filter((r) => {
      const next = new Date(r.nextReview);
      next.setHours(0, 0, 0, 0);
      return next.getTime() <= now.getTime();
    })
    .map((r) => ({
      topic: r.topic,
      domain: r.domain,
      dueDate: r.nextReview,
      isOverdue: r.nextReview.getTime() < now.getTime(),
      strength: r.strength,
    }))
    .sort((a, b) => a.dueDate.getTime() - b.dueDate.getTime());
  return due;
}
