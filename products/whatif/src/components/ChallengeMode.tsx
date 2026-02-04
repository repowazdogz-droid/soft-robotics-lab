import { useStore } from '../store';
import { useScenario } from '../hooks/useScenario';
import { CHALLENGES } from '../scenarios/challenges';
import type { ChallengeId } from '../types';
import styles from './ChallengeMode.module.css';

export function ChallengeMode() {
  const setMode = useStore((s) => s.setMode);
  const setSelectedChallengeId = useStore((s) => s.setSelectedChallengeId);
  const { loadChallenge } = useScenario();

  const handleSelect = (id: ChallengeId) => {
    loadChallenge(id);
    setSelectedChallengeId(id);
    setMode('challenges');
  };

  return (
    <div className={styles.wrap}>
      <h1 className={styles.heading}>Challenges</h1>
      <p className={styles.sub}>Constrained puzzles. Can you win?</p>
      <div className={styles.grid}>
        {CHALLENGES.map((c) => (
          <button
            key={c.id}
            type="button"
            className={styles.card}
            onClick={() => handleSelect(c.id)}
          >
            <h2 className={styles.cardTitle}>{c.title}</h2>
            <p className={styles.cardDesc}>{c.description}</p>
            <span className={styles.cardWin}>{c.winCondition}</span>
          </button>
        ))}
      </div>
    </div>
  );
}
