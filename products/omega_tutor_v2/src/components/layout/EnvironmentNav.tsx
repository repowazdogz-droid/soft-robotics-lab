/**
 * EnvironmentNav — Switch between Workspace / Dashboard / Terrain.
 * Calm, precise; no decorative elements.
 */

import { NavLink } from "react-router-dom";

const envs = [
  { path: "/workspace", label: "Learning Workspace" },
  { path: "/dashboard", label: "Cognitive Dashboard" },
  { path: "/terrain", label: "Knowledge Terrain" },
] as const;

export function EnvironmentNav() {
  return (
    <nav
      className="flex flex-wrap items-center gap-1 border-b border-bg-tertiary bg-bg-secondary px-4 py-2"
      aria-label="Primary environments"
    >
      {envs.map(({ path, label }) => (
        <NavLink
          key={path}
          to={path}
          className={({ isActive }) =>
            `px-3 py-2 text-sm font-medium no-underline transition-colors rounded focus:outline-none focus:ring-2 focus:ring-accent focus:ring-offset-1 ${
              isActive
                ? "text-accent border-b-2 border-accent -mb-[2px]"
                : "text-text-muted hover:text-text-secondary"
            }`
          }
        >
          {label}
        </NavLink>
      ))}
    </nav>
  );
}
