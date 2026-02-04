/**
 * LoadingState — Full-page-style loading with message.
 */

import { LoadingSpinner } from "./LoadingSpinner";

export interface LoadingStateProps {
  message?: string;
}

export function LoadingState({
  message = "Loading…",
}: LoadingStateProps) {
  return (
    <div className="flex min-h-[200px] items-center justify-center">
      <LoadingSpinner message={message} size="md" />
    </div>
  );
}
