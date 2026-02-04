import { useCallback } from 'react';
import { useStore } from '../store';
import { SCENARIOS } from '../scenarios/scenarios';
import { CHALLENGES } from '../scenarios/challenges';
import type { ScenarioId, ChallengeId } from '../types';
import type { ScenarioSetup } from '../types';

export function useScenario() {
  const setGravity = useStore((s) => s.setGravity);
  const setFriction = useStore((s) => s.setFriction);
  const setTimeScale = useStore((s) => s.setTimeScale);
  const setSelectedScenarioId = useStore((s) => s.setSelectedScenarioId);
  const setSelectedChallengeId = useStore((s) => s.setSelectedChallengeId);
  const resetPlacedCount = useStore((s) => s.resetPlacedCount);

  const loadSetup = useCallback(
    (setup: ScenarioSetup) => {
      setGravity(setup.gravity);
      setFriction(setup.friction);
      setTimeScale(1);
      resetPlacedCount();
    },
    [setGravity, setFriction, setTimeScale, resetPlacedCount]
  );

  const loadScenario = useCallback(
    (id: ScenarioId) => {
      const scenario = SCENARIOS.find((s) => s.id === id);
      if (scenario) {
        setSelectedScenarioId(id);
        setSelectedChallengeId(null);
        loadSetup(scenario.setup);
        return scenario.setup;
      }
      return null;
    },
    [loadSetup, setSelectedScenarioId, setSelectedChallengeId]
  );

  const loadChallenge = useCallback(
    (id: ChallengeId) => {
      const challenge = CHALLENGES.find((c) => c.id === id);
      if (challenge) {
        setSelectedChallengeId(id);
        setSelectedScenarioId(null);
        const { gravity, friction } = challenge.constraints;
        setGravity(gravity);
        setFriction(friction ?? 0.5);
        setTimeScale(1);
        resetPlacedCount();
        return challenge.setup;
      }
      return null;
    },
    [setGravity, setFriction, setTimeScale, setSelectedChallengeId, setSelectedScenarioId, resetPlacedCount]
  );

  const getScenarioSetup = useCallback((id: ScenarioId) => {
    return SCENARIOS.find((s) => s.id === id)?.setup ?? null;
  }, []);

  const getChallengeSetup = useCallback((id: ChallengeId) => {
    return CHALLENGES.find((c) => c.id === id)?.setup ?? null;
  }, []);

  const getChallengeConstraints = useCallback((id: ChallengeId) => {
    return CHALLENGES.find((c) => c.id === id)?.constraints ?? null;
  }, []);

  return {
    loadScenario,
    loadChallenge,
    loadSetup,
    getScenarioSetup,
    getChallengeSetup,
    getChallengeConstraints,
  };
}
