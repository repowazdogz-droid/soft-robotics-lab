/**
 * DashboardPage — Cognitive dashboard. Professional instrumentation.
 * MasteryGrid, RetentionChart, MisconceptionList, ReviewQueue.
 */

import { useState, useEffect } from "react";
import { useNavigate, Link } from "react-router-dom";
import { StatCard } from "../components/dashboard/StatCard";
import { MasteryGrid } from "../components/dashboard/MasteryGrid";
import { RetentionChart } from "../components/dashboard/RetentionChart";
import { MisconceptionList } from "../components/dashboard/MisconceptionList";
import { ReviewQueue } from "../components/dashboard/ReviewQueue";
import { LearnedTopicsList } from "../components/dashboard/LearnedTopicsList";
import {
  getDomainMastery,
  getRetentionItems,
  getRecordedMisconceptions,
  getDueReviews,
  getLearnedTopics,
} from "../services/dashboard";
import type { LearnedTopic } from "../services/dashboard";
import type {
  DomainMastery,
  RetentionItem,
  RecordedMisconception,
  ReviewItem,
} from "../types/dashboard";

export function DashboardPage() {
  const [domains, setDomains] = useState<DomainMastery[]>([]);
  const [retention, setRetention] = useState<RetentionItem[]>([]);
  const [misconceptions, setMisconceptions] = useState<RecordedMisconception[]>(
    []
  );
  const [dueReviews, setDueReviews] = useState<ReviewItem[]>([]);
  const [learnedTopics, setLearnedTopics] = useState<LearnedTopic[]>([]);
  const navigate = useNavigate();

  const refreshData = () => {
    setDomains(getDomainMastery());
    setRetention(getRetentionItems());
    setMisconceptions(getRecordedMisconceptions());
    setDueReviews(getDueReviews());
    setLearnedTopics(getLearnedTopics());
  };

  useEffect(() => {
    refreshData();
  }, []);

  const totalTopics = domains.reduce((sum, d) => sum + d.topicsLearned, 0);
  const dueCount = dueReviews.length;
  const misconceptionCount = misconceptions.length;
  const avgRetention =
    retention.length > 0
      ? Math.round(
          retention.reduce((sum, r) => sum + r.strength, 0) / retention.length
        )
      : 0;

  const handleStartReview = (topic: string) => {
    navigate("/workspace", { state: { reviewTopic: topic } });
  };

  const handleClearData = () => {
    if (
      confirm(
        "Clear all learning data? This cannot be undone."
      )
    ) {
      if (typeof localStorage !== "undefined") {
        localStorage.removeItem("omega_tutor_learned_topics");
        localStorage.removeItem("omega_tutor_review_schedule");
        localStorage.removeItem("omega_tutor_misconceptions");
        localStorage.removeItem("omega_tutor_curriculum_progress");
      }
      refreshData();
    }
  };

  return (
    <div className="mx-auto max-w-5xl space-y-8 p-6">
      <header className="border-b border-bg-tertiary pb-6">
        <h1 className="text-2xl font-semibold text-text-primary">
          Cognitive Dashboard
        </h1>
        <p className="mt-2 max-w-2xl text-text-secondary">
          Track what you've learned, what's fading, and where to focus next. This
          is your knowledge instrumentation panel — not gamification, just
          clarity.
        </p>
      </header>

      <div className="grid grid-cols-2 gap-4 sm:grid-cols-4">
        <StatCard label="Topics Learned" value={totalTopics} />
        <StatCard
          label="Due for Review"
          value={dueCount}
          highlight={dueCount > 0}
        />
        <StatCard label="Misconceptions" value={misconceptionCount} />
        <StatCard label="Avg Retention" value={`${avgRetention}%`} />
      </div>

      <section className="space-y-4">
        <div>
          <h2 className="text-lg font-medium text-text-primary">
            What You've Learned
          </h2>
          <p className="mt-1 text-sm text-text-muted">
            Your learning history
          </p>
        </div>
        <LearnedTopicsList
          topics={learnedTopics}
          onReview={(topic) =>
            navigate("/workspace", { state: { reviewTopic: topic } })
          }
        />
      </section>

      {dueReviews.length > 0 ? (
        <section className="rounded-lg border border-amber-200 bg-amber-50 p-6">
          <h2 className="text-lg font-medium text-amber-900">
            Topics need review
          </h2>
          <p className="mt-1 text-sm text-amber-700">
            Spaced repetition works best when you review before forgetting.
          </p>
          <div className="mt-4">
            <ReviewQueue items={dueReviews} onStartReview={handleStartReview} />
          </div>
        </section>
      ) : (
        <section className="rounded-lg border border-green-200 bg-green-50 p-4">
          <p className="text-green-800">
            ✓ Nothing due for review. You're on track.
          </p>
        </section>
      )}

      {domains.length > 0 && (
        <div className="grid gap-6 lg:grid-cols-2">
          <section className="space-y-4">
            <div>
              <h2 className="text-lg font-medium text-text-primary">
                Mastery by Domain
              </h2>
              <p className="mt-1 text-sm text-text-muted">
                Topics you've studied, grouped by area
              </p>
            </div>
            <MasteryGrid domains={domains} learnedTopics={learnedTopics} />
          </section>

          <section className="space-y-4">
            <div>
              <h2 className="text-lg font-medium text-text-primary">
                Retention Strength
              </h2>
              <p className="mt-1 text-sm text-text-muted">
                How well you're holding onto what you've learned
              </p>
            </div>
            <RetentionChart items={retention} sortBy="nextReview" />
          </section>
        </div>
      )}

      <section className="space-y-4">
        <div>
          <h2 className="text-lg font-medium text-text-primary">
            Areas for Reinforcement
          </h2>
          <p className="mt-1 text-sm text-text-muted">
            Misconceptions surface here so you can address them. This is insight,
            not failure.
          </p>
        </div>
        <MisconceptionList misconceptions={misconceptions} />
      </section>

      {domains.length === 0 && (
        <div className="rounded-lg bg-bg-secondary py-12 text-center">
          <h2 className="text-lg font-medium text-text-primary">
            Your dashboard is empty
          </h2>
          <p className="mx-auto mt-2 max-w-md text-text-muted">
            Start learning in the Workspace. As you study topics and complete
            explain-backs, your knowledge landscape will appear here.
          </p>
          <Link
            to="/workspace"
            className="mt-4 inline-block rounded-lg bg-accent px-4 py-2 text-white hover:bg-accent/90"
          >
            Start Learning
          </Link>
        </div>
      )}

      <section className="mt-8 border-t border-bg-tertiary pt-6">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="text-sm font-medium text-text-primary">
              Data Management
            </h3>
            <p className="text-xs text-text-muted">
              All data is stored locally in your browser
            </p>
          </div>
          <button
            type="button"
            onClick={handleClearData}
            className="text-sm text-red-600 hover:underline hover:text-red-700"
          >
            Clear all learning data
          </button>
        </div>
      </section>
    </div>
  );
}
