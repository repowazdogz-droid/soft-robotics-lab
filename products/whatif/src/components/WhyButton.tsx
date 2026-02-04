import { useStore } from '../store';
import { EXPLANATIONS } from '../scenarios/explanations';
import type { ScenarioId } from '../types';
import styles from './WhyButton.module.css';

const SCENARIO_IDS: ScenarioId[] = ['moon-drop', 'no-friction', 'reverse-gravity'];

export function WhyButton() {
  const whyPanelOpen = useStore((s) => s.whyPanelOpen);
  const setWhyPanelOpen = useStore((s) => s.setWhyPanelOpen);
  const selectedScenarioId = useStore((s) => s.selectedScenarioId);

  const explanation =
    selectedScenarioId && SCENARIO_IDS.includes(selectedScenarioId)
      ? EXPLANATIONS[selectedScenarioId]
      : null;

  return (
    <>
      <button
        type="button"
        className={styles.trigger}
        onClick={() => setWhyPanelOpen(!whyPanelOpen)}
        title="Why?"
        aria-expanded={whyPanelOpen}
      >
        Why?
      </button>
      {whyPanelOpen && explanation && (
        <div className={styles.panel} role="dialog" aria-label="Explanation">
          <div className={styles.panelInner}>
            <h3 className={styles.panelTitle}>{explanation.title}</h3>
            <p className={styles.panelText}>{explanation.text}</p>
            <button
              type="button"
              className={styles.close}
              onClick={() => setWhyPanelOpen(false)}
              aria-label="Close"
            >
              Close
            </button>
          </div>
        </div>
      )}
    </>
  );
}
