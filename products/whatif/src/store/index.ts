import { create } from 'zustand';
import type { AppMode, ScenarioId, ChallengeId, ObjectType } from '../types';

interface AppState {
  mode: AppMode;
  setMode: (m: AppMode) => void;

  selectedScenarioId: ScenarioId | null;
  setSelectedScenarioId: (id: ScenarioId | null) => void;

  selectedChallengeId: ChallengeId | null;
  setSelectedChallengeId: (id: ChallengeId | null) => void;

  isPaused: boolean;
  setPaused: (p: boolean) => void;

  gravity: number;
  friction: number;
  timeScale: number;
  setGravity: (g: number) => void;
  setFriction: (f: number) => void;
  setTimeScale: (t: number) => void;
  resetDials: () => void;

  selectedObjectType: ObjectType | null;
  setSelectedObjectType: (t: ObjectType | null) => void;

  whyPanelOpen: boolean;
  setWhyPanelOpen: (o: boolean) => void;

  resultsOverlayOpen: boolean;
  setResultsOverlayOpen: (o: boolean) => void;
  resultsExpected: boolean | null;
  setResultsExpected: (e: boolean | null) => void;
  resultsNote: string;
  setResultsNote: (n: string) => void;

  placedCount: number;
  incrementPlacedCount: () => void;
  resetPlacedCount: () => void;
}

const defaultGravity = 1;
const defaultFriction = 0.5;
const defaultTimeScale = 1;

export const useStore = create<AppState>((set) => ({
  mode: 'scenarios',
  setMode: (mode) => set({ mode }),

  selectedScenarioId: null,
  setSelectedScenarioId: (selectedScenarioId) => set({ selectedScenarioId }),

  selectedChallengeId: null,
  setSelectedChallengeId: (selectedChallengeId) => set({ selectedChallengeId }),

  isPaused: false,
  setPaused: (isPaused) => set({ isPaused }),

  gravity: defaultGravity,
  friction: defaultFriction,
  timeScale: defaultTimeScale,
  setGravity: (gravity) => set({ gravity }),
  setFriction: (friction) => set({ friction }),
  setTimeScale: (timeScale) => set({ timeScale }),
  resetDials: () =>
    set({
      gravity: defaultGravity,
      friction: defaultFriction,
      timeScale: defaultTimeScale,
    }),

  selectedObjectType: null,
  setSelectedObjectType: (selectedObjectType) => set({ selectedObjectType }),

  whyPanelOpen: false,
  setWhyPanelOpen: (whyPanelOpen) => set({ whyPanelOpen }),

  resultsOverlayOpen: false,
  setResultsOverlayOpen: (resultsOverlayOpen) => set({ resultsOverlayOpen }),
  resultsExpected: null,
  setResultsExpected: (resultsExpected) => set({ resultsExpected }),
  resultsNote: '',
  setResultsNote: (resultsNote) => set({ resultsNote }),

  placedCount: 0,
  incrementPlacedCount: () =>
    set((s) => ({ placedCount: s.placedCount + 1 })),
  resetPlacedCount: () => set({ placedCount: 0 }),
}));
