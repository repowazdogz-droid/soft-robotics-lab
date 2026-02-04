import { useStore } from '../store';
import { useScenario } from '../hooks/useScenario';
import { SCENARIOS } from '../scenarios/scenarios';
import type { ScenarioId } from '../types';
import styles from './ScenarioSelector.module.css';

export function ScenarioSelector() {
  const setMode = useStore((s) => s.setMode);
  const setSelectedScenarioId = useStore((s) => s.setSelectedScenarioId);
  const { loadScenario } = useScenario();

  const handleSelect = (id: ScenarioId) => {
    loadScenario(id);
    setSelectedScenarioId(id);
    setMode('scenarios');
  };

  return (
    <div className={styles.wrap}>
      <h1 className={styles.heading}>What If?</h1>
      <p className={styles.sub}>Change an assumption. See what happens.</p>
      <div className={styles.grid}>
        {SCENARIOS.map((s) => (
          <button
            key={s.id}
            type="button"
            className={styles.card}
            onClick={() => handleSelect(s.id)}
          >
            <h2 className={styles.cardTitle}>{s.title}</h2>
            <p className={styles.cardDesc}>{s.description}</p>
            <span className={styles.cardHint}>{s.hint}</span>
          </button>
        ))}
      </div>
    </div>
  );
}
