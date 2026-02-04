import { useEffect } from 'react';
import { useStore } from '../store';
import styles from './Controls.module.css';

export function Controls() {
  const isPaused = useStore((s) => s.isPaused);
  const setPaused = useStore((s) => s.setPaused);
  const timeScale = useStore((s) => s.timeScale);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.code === 'Space') {
        e.preventDefault();
        setPaused(!useStore.getState().isPaused);
      }
      if (e.code === 'KeyR' && !e.ctrlKey && !e.metaKey) {
        e.preventDefault();
        window.dispatchEvent(new CustomEvent('whatif-reset'));
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [setPaused]);

  const handleReset = () => {
    window.dispatchEvent(new CustomEvent('whatif-reset'));
  };

  const handleClear = () => {
    window.dispatchEvent(new CustomEvent('whatif-clear'));
  };

  return (
    <div className={styles.bar}>
      <div className={styles.left}>
        <button
          type="button"
          className={styles.btn}
          onClick={() => setPaused(!isPaused)}
          title={isPaused ? 'Play (Space)' : 'Pause (Space)'}
        >
          {isPaused ? 'Play' : 'Pause'}
        </button>
        <button
          type="button"
          className={styles.btn}
          onClick={handleReset}
          title="Reset scenario (R)"
        >
          Reset
        </button>
        <button
          type="button"
          className={styles.btn}
          onClick={handleClear}
          title="Clear all objects"
        >
          Clear
        </button>
      </div>
      <div className={styles.right}>
        <span className={styles.speed}>{timeScale.toFixed(1)}×</span>
      </div>
    </div>
  );
}
