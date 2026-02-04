/**
 * OMEGA Tutor v2 — Misconceptions hook. Phase 3.
 */

export function useMisconceptions(_topic: string) {
  return {
    list: [],
    preemptiveWarning: null as string | null,
    checkExplanation: (_text: string) => ({ hasMisconceptions: false, corrections: [] as string[] }),
  };
}
