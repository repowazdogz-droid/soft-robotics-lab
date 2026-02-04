/**
 * LoadingSpinner — Reusable loading indicator.
 */

export interface LoadingSpinnerProps {
  size?: "sm" | "md" | "lg";
  message?: string;
}

const sizeClasses = {
  sm: "h-4 w-4",
  md: "h-8 w-8",
  lg: "h-12 w-12",
};

export function LoadingSpinner({
  size = "md",
  message,
}: LoadingSpinnerProps) {
  return (
    <div className="flex flex-col items-center justify-center gap-3">
      <div
        className={`${sizeClasses[size]} shrink-0 rounded-full border-2 border-bg-tertiary border-t-accent animate-spin`}
        aria-hidden
      />
      {message && (
        <p className="text-sm text-text-muted">{message}</p>
      )}
    </div>
  );
}
