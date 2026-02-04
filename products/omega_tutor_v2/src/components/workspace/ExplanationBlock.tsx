/**
 * ExplanationBlock — Main content display. Not chat bubbles.
 * Core explanation, collapsible Assumptions / Competing Models, expandable DepthLayers.
 */

import { useState } from "react";
import type { TeachingResponse } from "../../types/learning";
import { AssumptionToggle } from "./AssumptionToggle";
import { DepthLayer } from "./DepthLayer";

export interface ExplanationBlockProps {
  data: TeachingResponse;
  showAssumptions?: boolean;
  showCompetingModels?: boolean;
}

export function ExplanationBlock({
  data,
  showAssumptions = true,
  showCompetingModels = true,
}: ExplanationBlockProps) {
  const [competingOpen, setCompetingOpen] = useState(false);

  return (
    <article className="animate-fade-in space-y-8">
      <section>
        <h2 className="sr-only">Core explanation</h2>
        <div className="prose prose-lg max-w-none text-text-primary whitespace-pre-wrap">
          {data.coreExplanation}
        </div>
      </section>

      {showAssumptions && data.assumptions?.length > 0 && (
        <AssumptionToggle
          assumptions={data.assumptions}
          label="Show assumptions"
          labelWhenOpen="Hide assumptions"
        />
      )}

      {showCompetingModels && data.competingModels?.length > 0 && (
        <section className="rounded-lg border border-bg-tertiary bg-bg-secondary p-4">
          <button
            type="button"
            onClick={() => setCompetingOpen((o) => !o)}
            className="text-sm font-medium text-accent hover:underline"
            aria-expanded={competingOpen}
          >
            {competingOpen ? "Hide competing models" : "Show competing models"}
          </button>
          {competingOpen && (
            <ul className="mt-4 space-y-4">
              {data.competingModels.map((m, i) => (
                <li key={i} className="border-l-2 border-bg-tertiary pl-4">
                  <strong className="text-text-primary">{m.name}</strong>
                  <p className="mt-1 text-sm text-text-secondary">
                    {m.explanation}
                  </p>
                  <p className="mt-1 text-xs text-text-muted">
                    Strengths: {m.strengths} — Limitations: {m.limitations}
                  </p>
                </li>
              ))}
            </ul>
          )}
        </section>
      )}

      {data.depthLayers?.length > 0 && (
        <section>
          <h2 className="mb-3 text-sm font-semibold text-text-primary">
            Depth layers
          </h2>
          <div className="space-y-2">
            {data.depthLayers.map((layer, i) => (
              <DepthLayer
                key={i}
                layer={layer}
                depthIndex={i}
                defaultExpanded={false}
              />
            ))}
          </div>
        </section>
      )}

      {data.reflectionPrompt && (
        <section className="rounded-lg border border-bg-tertiary bg-bg-secondary p-4">
          <h2 className="text-sm font-semibold text-text-primary">
            Reflect
          </h2>
          <p className="mt-2 text-text-secondary">{data.reflectionPrompt}</p>
        </section>
      )}
    </article>
  );
}
