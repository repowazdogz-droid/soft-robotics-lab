import { DIALS } from '../constants';
import { useStore } from '../store';
import styles from './ControlPanel.module.css';

const DIAL_DEFAULTS: Record<string, number> = {
  gravity: 1,
  friction: 0.5,
  timeScale: 1,
};

function isDangerous(id: string, value: number): boolean {
  if (id === 'gravity') return value < -0.5 || value > 3;
  if (id === 'friction') return value < 0.1;
  if (id === 'timeScale') return value < 0.3 || value > 2;
  return false;
}

export function ControlPanel() {
  const gravity = useStore((s) => s.gravity);
  const friction = useStore((s) => s.friction);
  const timeScale = useStore((s) => s.timeScale);
  const setGravity = useStore((s) => s.setGravity);
  const setFriction = useStore((s) => s.setFriction);
  const setTimeScale = useStore((s) => s.setTimeScale);
  const resetDials = useStore((s) => s.resetDials);

  const values: Record<string, number> = { gravity, friction, timeScale };
  const setters: Record<string, (v: number) => void> = {
    gravity: setGravity,
    friction: setFriction,
    timeScale: setTimeScale,
  };

  return (
    <div className={styles.panel}>
      <h3 className={styles.title}>Reality dials</h3>
      {DIALS.map((dial) => {
        const value = values[dial.id];
        const setValue = setters[dial.id];
        const dangerous = isDangerous(dial.id, value);
        const pct =
          ((value - dial.min) / (dial.max - dial.min)) * 100;
        return (
          <div
            key={dial.id}
            className={`${styles.dialWrap} ${dangerous ? styles.dangerous : ''}`}
          >
            <div className={styles.dialHeader}>
              <label className={styles.label}>{dial.label}</label>
              <span className={styles.value}>
                {value.toFixed(dial.unit ? 2 : 2)}
                {dial.unit}
              </span>
            </div>
            <input
              type="range"
              min={dial.min}
              max={dial.max}
              step={dial.id === 'friction' ? 0.05 : 0.1}
              value={value}
              onChange={(e) => setValue(Number(e.target.value))}
              className={styles.slider}
              style={
                {
                  '--pct': `${pct}%`,
                } as React.CSSProperties
              }
            />
            {dangerous && (
              <span className={styles.warning}>{dial.warning}</span>
            )}
            <button
              type="button"
              className={styles.resetBtn}
              onClick={() => setValue(DIAL_DEFAULTS[dial.id] ?? dial.default)}
              aria-label={`Reset ${dial.label}`}
            >
              Reset
            </button>
          </div>
        );
      })}
      <button
        type="button"
        className={styles.resetAll}
        onClick={resetDials}
      >
        Reset all dials
      </button>
    </div>
  );
}
