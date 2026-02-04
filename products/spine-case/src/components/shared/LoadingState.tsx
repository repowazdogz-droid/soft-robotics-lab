export function LoadingState() {
  return (
    <div className="flex flex-col items-center justify-center py-16">
      <div className="w-12 h-12 border-4 border-slate-200 border-t-slate-800 rounded-full animate-spin mb-4" />
      <p className="text-slate-600 font-medium">Generating case synthesis...</p>
      <p className="text-sm text-slate-500 mt-2">This typically takes 10-20 seconds</p>
    </div>
  );
}
