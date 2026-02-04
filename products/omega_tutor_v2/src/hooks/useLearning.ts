/**
 * OMEGA Tutor v2 — Learning hook. Phase 2: teach + explain-back state.
 */

export function useLearning() {
  return {
    question: null as string | null,
    response: null,
    explainBackResult: null,
    isLoading: false,
    ask: async (_q: string, _depth: string) => {},
    submitExplainBack: async (_topic: string, _text: string) => {},
  };
}
