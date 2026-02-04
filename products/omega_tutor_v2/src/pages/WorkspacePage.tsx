/**
 * WorkspacePage — Learning workspace. Not chat.
 * QuestionInput first; DepthDial lives with explanation (depth as live control).
 */

import { useState, useCallback, useEffect, useRef } from "react";
import { useLocation, useNavigate } from "react-router-dom";
import { QuestionInput } from "../components/workspace/QuestionInput";
import { DepthDial } from "../components/workspace/DepthDial";
import { TopicHeader } from "../components/workspace/TopicHeader";
import { ExplanationBlock } from "../components/workspace/ExplanationBlock";
import { TemporalFraming } from "../components/workspace/TemporalFraming";
import { MisconceptionAlert } from "../components/workspace/MisconceptionAlert";
import { ExplainBackPrompt } from "../components/workspace/ExplainBackPrompt";
import { ExplainBackFeedback } from "../components/workspace/ExplainBackFeedback";
import { LoadingState } from "../components/common/LoadingState";
import { ErrorMessage } from "../components/common/ErrorMessage";
import { teach, evaluateExplanation } from "../services/learning";
import {
  recordLearning,
  recordMisconception,
} from "../services/dashboard";
import { markTopicComplete } from "../services/terrain";
import type { DepthLevel, TeachingResponse, ExplainBackResult } from "../types/learning";

const SESSION_DEPTH_KEY = "omega_tutor_depth_level";

const EXAMPLE_QUESTIONS = [
  {
    question: "What is morphological computation in soft robotics?",
    category: "Robotics",
  },
  {
    question: "How does spaced repetition improve memory retention?",
    category: "Learning Science",
  },
  {
    question: "What are the key principles of trauma-informed practice?",
    category: "Education",
  },
  {
    question: "How do neural networks learn from data?",
    category: "Machine Learning",
  },
  {
    question: "What is the difference between Type 1 and Type 2 thinking?",
    category: "Cognitive Science",
  },
];

function getInitialDepth(): DepthLevel {
  if (typeof sessionStorage === "undefined") return "structured";
  const s = sessionStorage.getItem(SESSION_DEPTH_KEY);
  const levels: DepthLevel[] = ["intuitive", "structured", "technical", "research"];
  return s && levels.includes(s as DepthLevel) ? (s as DepthLevel) : "structured";
}

export function WorkspacePage() {
  const location = useLocation();
  const navigate = useNavigate();
  const handledReviewRef = useRef(false);

  const [currentTopic, setCurrentTopic] = useState<string | null>(null);
  const [depthLevel, setDepthLevel] = useState<DepthLevel>("structured");
  const [teachingResponse, setTeachingResponse] = useState<TeachingResponse | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const [showExplainBack, setShowExplainBack] = useState(false);
  const [explainBackResult, setExplainBackResult] = useState<ExplainBackResult | null>(null);
  const [isEvaluating, setIsEvaluating] = useState(false);
  const [evaluateError, setEvaluateError] = useState<string | null>(null);

  const [fromCurriculumId, setFromCurriculumId] = useState<string | null>(null);
  const [terrainTopicId, setTerrainTopicId] = useState<string | null>(null);

  useEffect(() => {
    const state = location.state as {
      reviewTopic?: string;
      initialQuestion?: string;
      fromCurriculum?: string;
      topicId?: string;
    } | null;
    const question = state?.reviewTopic ?? state?.initialQuestion;
    if (!question || typeof question !== "string" || handledReviewRef.current)
      return;
    handledReviewRef.current = true;
    if (state?.initialQuestion && state?.fromCurriculum && state?.topicId) {
      setFromCurriculumId(state.fromCurriculum);
      setTerrainTopicId(state.topicId);
    }
    setError(null);
    setCurrentTopic(question);
    setTeachingResponse(null);
    setShowExplainBack(false);
    setExplainBackResult(null);
    const depth = getInitialDepth();
    setDepthLevel(depth);
    if (typeof sessionStorage !== "undefined")
      sessionStorage.setItem(SESSION_DEPTH_KEY, depth);
    setIsLoading(true);
    teach(question, depth)
      .then(setTeachingResponse)
      .catch((e) =>
        setError(e instanceof Error ? e.message : "Something went wrong")
      )
      .finally(() => setIsLoading(false));
    navigate(".", { replace: true, state: {} });
  }, [location.state, navigate]);

  const handleDepthChange = useCallback(
    async (newDepth: DepthLevel) => {
      setDepthLevel(newDepth);
      if (typeof sessionStorage !== "undefined")
        sessionStorage.setItem(SESSION_DEPTH_KEY, newDepth);
      setShowExplainBack(false);
      setExplainBackResult(null);
      if (!currentTopic) return;
      setError(null);
      setIsLoading(true);
      try {
        const response = await teach(currentTopic, newDepth);
        setTeachingResponse(response);
      } catch (e) {
        setError(e instanceof Error ? e.message : "Failed to generate");
      } finally {
        setIsLoading(false);
      }
    },
    [currentTopic]
  );

  const handleSubmit = useCallback(
    async (question: string) => {
      setError(null);
      setCurrentTopic(question);
      setTeachingResponse(null);
      setShowExplainBack(false);
      setExplainBackResult(null);
      const depth = getInitialDepth();
      setDepthLevel(depth);
      if (typeof sessionStorage !== "undefined")
        sessionStorage.setItem(SESSION_DEPTH_KEY, depth);
      setIsLoading(true);
      try {
        const response = await teach(question, depth);
        setTeachingResponse(response);
      } catch (e) {
        setError(e instanceof Error ? e.message : "Something went wrong");
      } finally {
        setIsLoading(false);
      }
    },
    []
  );

  const handleExplainBackSubmit = useCallback(
    async (explanation: string) => {
      if (!currentTopic || !teachingResponse) return;
      setEvaluateError(null);
      setIsEvaluating(true);
      try {
        const result = await evaluateExplanation(
          explanation,
          currentTopic,
          teachingResponse
        );
        setExplainBackResult(result);
        recordLearning(currentTopic, "general", result.score);
        if (fromCurriculumId && terrainTopicId) {
          markTopicComplete(fromCurriculumId, terrainTopicId, result.score);
          setFromCurriculumId(null);
          setTerrainTopicId(null);
        }
        if (result.misconceptions?.length && result.correction) {
          result.misconceptions.forEach((m) =>
            recordMisconception(currentTopic, m, result.correction)
          );
        }
      } catch (e) {
        setEvaluateError(e instanceof Error ? e.message : "Evaluation failed");
      } finally {
        setIsEvaluating(false);
      }
    },
    [currentTopic, teachingResponse, fromCurriculumId, terrainTopicId]
  );

  const handleExplainBackContinue = useCallback(() => {
    setShowExplainBack(false);
    setExplainBackResult(null);
  }, []);

  const handleExplainBackRetry = useCallback(() => {
    setExplainBackResult(null);
  }, []);

  const handleExampleClick = useCallback(
    (question: string) => {
      handleSubmit(question);
    },
    [handleSubmit]
  );

  return (
    <div className="mx-auto max-w-3xl px-4 py-8">
      <div className="mb-8">
        <QuestionInput
          onSubmit={handleSubmit}
          disabled={isLoading}
          placeholder="What do you want to understand today?"
        />
      </div>

      {error && (
        <div className="mb-6">
          <ErrorMessage
            message={error}
            onRetry={() => {
              setError(null);
              if (currentTopic) handleSubmit(currentTopic);
            }}
          />
        </div>
      )}

      {currentTopic && (teachingResponse || isLoading) && (
        <div className="animate-fade-in">
          <TopicHeader
            topic={currentTopic}
            depthLevel={depthLevel}
            breadcrumb={undefined}
          />

          <div className="mb-6 flex flex-wrap items-center gap-3">
            <DepthDial
              value={depthLevel}
              onChange={handleDepthChange}
              disabled={isLoading}
            />
            {isLoading && teachingResponse && (
              <span className="text-sm text-text-muted">Regenerating…</span>
            )}
          </div>

          {isLoading ? (
            <div className="py-8">
              <LoadingState message="Thinking…" />
            </div>
          ) : teachingResponse ? (
            <>
          {teachingResponse.commonMisconceptions?.length > 0 && (
            <div className="mb-6 space-y-3">
              {teachingResponse.commonMisconceptions.slice(0, 2).map((msg, i) => (
                <MisconceptionAlert key={i} message={msg} />
              ))}
            </div>
          )}

          <ExplanationBlock data={teachingResponse} />

          {depthLevel === "research" &&
            teachingResponse.temporalFraming &&
            (teachingResponse.temporalFraming.t1 ||
              teachingResponse.temporalFraming.t2 ||
              teachingResponse.temporalFraming.t3 ||
              teachingResponse.temporalFraming.t4) && (
              <TemporalFraming data={teachingResponse.temporalFraming} />
            )}

          {!showExplainBack && !explainBackResult && (
            <div className="mt-8">
              <button
                type="button"
                onClick={() => setShowExplainBack(true)}
                className="rounded-lg border border-bg-tertiary bg-bg-secondary px-4 py-2.5 text-sm font-medium text-text-primary hover:bg-bg-tertiary"
              >
                Check my understanding
              </button>
            </div>
          )}

          {showExplainBack && !explainBackResult && (
            <div className="mt-8 animate-fade-in">
              {evaluateError && (
                <div className="mb-4">
                  <ErrorMessage message={evaluateError} />
                </div>
              )}
              <ExplainBackPrompt
                topic={currentTopic}
                onSubmit={handleExplainBackSubmit}
                isLoading={isEvaluating}
              />
            </div>
          )}

          {explainBackResult && (
            <div className="mt-8">
              <ExplainBackFeedback
                result={explainBackResult}
                onContinue={handleExplainBackContinue}
                onRetry={handleExplainBackRetry}
              />
            </div>
          )}
            </>
          ) : null}
        </div>
      )}

      {!teachingResponse && !isLoading && (
        <div className="space-y-8">
          <div className="text-center py-6">
            <h2 className="text-xl font-medium text-text-primary">
              What do you want to understand?
            </h2>
            <p className="text-text-muted mt-2">
              Ask any question. Get a structured explanation at your level.
            </p>
          </div>

          <div className="space-y-3">
            <h3 className="text-sm font-medium text-text-secondary">
              Try one of these
            </h3>
            <div className="grid gap-2">
              {EXAMPLE_QUESTIONS.map((q, i) => (
                <button
                  key={i}
                  onClick={() => handleExampleClick(q.question)}
                  className="text-left p-4 bg-bg-secondary hover:bg-bg-tertiary rounded-lg transition-colors"
                >
                  <span className="font-medium text-text-primary">
                    {q.question}
                  </span>
                  <span className="block text-sm text-text-muted mt-1">
                    {q.category}
                  </span>
                </button>
              ))}
            </div>
          </div>

          <div className="border-t border-bg-tertiary pt-6">
            <h3 className="text-sm font-medium text-text-secondary mb-3">
              How it works
            </h3>
            <div className="grid sm:grid-cols-3 gap-4 text-sm">
              <div className="p-3 bg-bg-secondary rounded-lg">
                <span className="font-medium text-text-primary">1. Ask</span>
                <p className="text-text-muted mt-1">
                  Enter any question you want to understand deeply
                </p>
              </div>
              <div className="p-3 bg-bg-secondary rounded-lg">
                <span className="font-medium text-text-primary">
                  2. Explore
                </span>
                <p className="text-text-muted mt-1">
                  Read the explanation, expand depth layers, switch levels
                </p>
              </div>
              <div className="p-3 bg-bg-secondary rounded-lg">
                <span className="font-medium text-text-primary">3. Check</span>
                <p className="text-text-muted mt-1">
                  Explain it back to test and consolidate understanding
                </p>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
