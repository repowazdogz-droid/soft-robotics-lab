/**
 * AppShell — Main layout: header, environment nav, content area.
 * No chat bubbles, no assistant personality. Scientific software aesthetic.
 */

import { Outlet } from "react-router-dom";
import { EnvironmentNav } from "./EnvironmentNav";
import { Link } from "react-router-dom";

export function AppShell() {
  return (
    <div className="flex h-full min-h-screen flex-col bg-bg-primary">
      <header className="border-b border-bg-tertiary bg-bg-secondary px-4 py-3">
        <div className="mx-auto flex max-w-6xl items-center justify-between">
          <Link
            to="/"
            className="text-lg font-semibold text-text-primary no-underline focus:outline-none focus:ring-2 focus:ring-accent focus:ring-offset-2 rounded"
          >
            OMEGA Tutor
          </Link>
          <Link
            to="/settings"
            className="text-sm text-text-muted no-underline hover:text-text-secondary focus:outline-none focus:ring-2 focus:ring-accent focus:ring-offset-2 rounded"
          >
            Settings
          </Link>
        </div>
      </header>
      <EnvironmentNav />
      <main className="flex-1 overflow-auto">
        <Outlet />
      </main>
    </div>
  );
}
