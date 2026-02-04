/**
 * DepthLayer — Expandable depth section. "Go Deeper" trigger, smooth expand/collapse.
 */

import { useState } from "react";
import type { DepthLayer as DepthLayerType } from "../../types/learning";

export interface DepthLayerProps {
  layer: DepthLayerType;
  depthIndex?: number;
  defaultExpanded?: boolean;
}

export function DepthLayer({
  layer,
  depthIndex = 0,
  defaultExpanded = false,
}: DepthLayerProps) {
  const [expanded, setExpanded] = useState(defaultExpanded);

  return (
    <div
      className="border-l-2 border-bg-tertiary pl-4"
      data-depth={depthIndex}
      style={{ marginLeft: `${depthIndex * 8}px` }}
    >
      <button
        type="button"
        onClick={() => setExpanded((e) => !e)}
        className="mb-1 flex w-full items-center gap-2 text-left text-sm font-medium text-accent hover:underline"
        aria-expanded={expanded}
      >
        <span
          className={`inline-block transition-transform ${expanded ? "rotate-90" : ""}`}
          aria-hidden
        >
          ▶
        </span>
        Go deeper — {layer.level}
      </button>
      {expanded && (
        <div className="animate-expand overflow-hidden text-text-secondary">
          <div className="prose prose-sm max-w-none pb-4 pt-1 text-text-secondary">
            {layer.content}
          </div>
        </div>
      )}
    </div>
  );
}
