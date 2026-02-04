/**
 * TopicHeader — Current topic and depth.
 * Topic title, depth badge, optional breadcrumb for curriculum.
 */

import type { DepthLevel } from "../../types/learning";

const DEPTH_LABELS: Record<DepthLevel, string> = {
  intuitive: "Intuitive",
  structured: "Structured",
  technical: "Technical",
  research: "Research",
};

export interface TopicHeaderProps {
  topic: string;
  depthLevel: DepthLevel;
  breadcrumb?: string[];
}

export function TopicHeader({
  topic,
  depthLevel,
  breadcrumb,
}: TopicHeaderProps) {
  return (
    <header className="mb-8">
      {breadcrumb && breadcrumb.length > 0 && (
        <nav
          className="mb-2 text-sm text-text-muted"
          aria-label="Breadcrumb"
        >
          {breadcrumb.map((item, i) => (
            <span key={item}>
              {i > 0 && <span className="mx-1.5">/</span>}
              <span>{item}</span>
            </span>
          ))}
        </nav>
      )}
      <h1 className="text-2xl font-semibold tracking-tight text-text-primary">
        {topic}
      </h1>
      <span
        className="mt-2 inline-block rounded-md border border-bg-tertiary bg-bg-secondary px-2.5 py-1 text-xs font-medium text-text-secondary"
        aria-label="Depth level"
      >
        {DEPTH_LABELS[depthLevel]}
      </span>
    </header>
  );
}
