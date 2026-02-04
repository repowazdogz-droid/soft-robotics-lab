import { useStore } from '../store';
import styles from './ResultsOverlay.module.css';

export function ResultsOverlay() {
  const resultsOverlayOpen = useStore((s) => s.resultsOverlayOpen);
  const setResultsOverlayOpen = useStore((s) => s.setResultsOverlayOpen);
  const resultsExpected = useStore((s) => s.resultsExpected);
  const setResultsExpected = useStore((s) => s.setResultsExpected);
  const resultsNote = useStore((s) => s.resultsNote);
  const setResultsNote = useStore((s) => s.setResultsNote);

  if (!resultsOverlayOpen) return null;

  const handleExpected = (expected: boolean) => {
    setResultsExpected(expected);
    if (!expected) return;
    setResultsOverlayOpen(false);
    setResultsNote('');
  };

  const handleSkip = () => {
    setResultsOverlayOpen(false);
    setResultsExpected(null);
    setResultsNote('');
  };

  const handleSubmitNote = () => {
    setResultsOverlayOpen(false);
    setResultsNote('');
  };

  return (
    <div className={styles.overlay} role="dialog" aria-label="What did you expect?">
      <div className={styles.card}>
        <h3 className={styles.title}>What did you expect to happen?</h3>
        <div className={styles.buttons}>
          <button
            type="button"
            className={styles.btn}
            onClick={() => handleExpected(true)}
          >
            It did what I thought
          </button>
          <button
            type="button"
            className={styles.btn}
            onClick={() => handleExpected(false)}
          >
            I was surprised
          </button>
        </div>
        {resultsExpected === false && (
          <div className={styles.follow}>
            <label className={styles.label}>What surprised you?</label>
            <textarea
              className={styles.input}
              value={resultsNote}
              onChange={(e) => setResultsNote(e.target.value)}
              placeholder="Optional..."
              rows={3}
            />
            <div className={styles.followActions}>
              <button type="button" className={styles.btnSecondary} onClick={handleSubmitNote}>
                Done
              </button>
              <button type="button" className={styles.btnSecondary} onClick={handleSkip}>
                Skip
              </button>
            </div>
          </div>
        )}
        {resultsExpected !== false && (
          <button
            type="button"
            className={styles.skip}
            onClick={handleSkip}
          >
            Skip
          </button>
        )}
      </div>
    </div>
  );
}
