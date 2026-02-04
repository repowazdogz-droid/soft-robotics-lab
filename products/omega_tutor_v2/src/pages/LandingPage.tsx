/**
 * LandingPage — Entry to environments. Hero, three cards, features.
 */

import { Link } from "react-router-dom";

function EnvironmentCard({
  title,
  description,
  href,
  icon,
}: {
  title: string;
  description: string;
  href: string;
  icon: string;
}) {
  return (
    <Link
      to={href}
      className="block rounded-lg bg-bg-secondary p-6 transition-colors hover:bg-bg-tertiary focus:outline-none focus:ring-2 focus:ring-accent focus:ring-offset-2"
    >
      <div className="mb-3 text-2xl" aria-hidden>
        {icon}
      </div>
      <h3 className="font-medium text-text-primary">{title}</h3>
      <p className="mt-2 text-sm text-text-muted">{description}</p>
    </Link>
  );
}

function Feature({
  title,
  description,
}: {
  title: string;
  description: string;
}) {
  return (
    <div className="rounded-lg bg-bg-secondary p-4">
      <h4 className="font-medium text-text-primary">{title}</h4>
      <p className="mt-1 text-text-muted">{description}</p>
    </div>
  );
}

export function LandingPage() {
  return (
    <div className="mx-auto max-w-4xl px-4 py-6 md:px-6">
      <header className="py-12 text-center">
        <h1 className="text-3xl font-semibold text-text-primary">
          OMEGA Tutor
        </h1>
        <p className="mt-2 text-lg text-text-secondary">
          A cognitive learning system for deep understanding
        </p>
      </header>

      <div className="mt-8 grid gap-6 md:grid-cols-3">
        <EnvironmentCard
          title="Learning Workspace"
          description="Explore concepts with adaptive explanations, multiple depth levels, and comprehension checkpoints."
          href="/workspace"
          icon="📚"
        />
        <EnvironmentCard
          title="Cognitive Dashboard"
          description="Track your knowledge landscape: mastery, retention, misconceptions, and review schedule."
          href="/dashboard"
          icon="📊"
        />
        <EnvironmentCard
          title="Knowledge Terrain"
          description="Navigate structured learning paths with prerequisites and progress tracking."
          href="/terrain"
          icon="🗺️"
        />
      </div>

      <section className="mt-16 space-y-4">
        <h2 className="text-center text-lg font-medium text-text-primary">
          How it works
        </h2>
        <div className="grid gap-4 text-sm text-text-secondary md:grid-cols-2">
          <Feature
            title="Adaptive Depth"
            description="From intuitive to research-level explanations"
          />
          <Feature
            title="Explain-Back"
            description="Consolidate understanding through active recall"
          />
          <Feature
            title="Misconception Detection"
            description="Surface and correct misunderstandings"
          />
          <Feature
            title="Spaced Repetition"
            description="Retain knowledge with SM-2 scheduling"
          />
        </div>
      </section>
    </div>
  );
}
