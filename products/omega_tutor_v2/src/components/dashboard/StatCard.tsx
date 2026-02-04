/**
 * StatCard — Single stat for dashboard (e.g. Topics Learned, Due for Review).
 */

export interface StatCardProps {
  label: string;
  value: string | number;
  highlight?: boolean;
}

export function StatCard({ label, value, highlight }: StatCardProps) {
  return (
    <div
      className={`rounded-lg p-4 ${
        highlight
          ? "border border-amber-200 bg-amber-50"
          : "border border-transparent bg-bg-secondary"
      }`}
    >
      <div
        className={`text-2xl font-semibold ${
          highlight ? "text-amber-700" : "text-text-primary"
        }`}
      >
        {value}
      </div>
      <div className="mt-1 text-sm text-text-muted">{label}</div>
    </div>
  );
}
