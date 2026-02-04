/**
 * Sidebar — Contextual information. Collapsible.
 * High information density, no clutter.
 */

import type { ReactNode } from "react";

interface SidebarProps {
  children: ReactNode;
  open?: boolean;
  onToggle?: () => void;
}

export function Sidebar({ children, open = true, onToggle }: SidebarProps) {
  return (
    <aside
      className={`border-l border-bg-tertiary bg-bg-secondary transition-all ${
        open ? "w-64 min-w-[16rem]" : "w-0 min-w-0 overflow-hidden"
      }`}
      aria-label="Context panel"
    >
      {onToggle && (
        <button
          type="button"
          onClick={onToggle}
          className="absolute left-0 top-4 -translate-x-full rounded-l border border-r-0 border-bg-tertiary bg-bg-secondary px-2 py-1 text-xs text-text-muted hover:text-text-secondary"
          aria-label={open ? "Collapse sidebar" : "Expand sidebar"}
        >
          {open ? "◀" : "▶"}
        </button>
      )}
      {open && <div className="p-4">{children}</div>}
    </aside>
  );
}
