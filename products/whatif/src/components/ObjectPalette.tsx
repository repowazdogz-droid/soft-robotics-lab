import { OBJECTS } from '../constants';
import { useStore } from '../store';
import { useScenario } from '../hooks/useScenario';
import styles from './ObjectPalette.module.css';

export function ObjectPalette() {
  const selectedObjectType = useStore((s) => s.selectedObjectType);
  const setSelectedObjectType = useStore((s) => s.setSelectedObjectType);
  const mode = useStore((s) => s.mode);
  const selectedChallengeId = useStore((s) => s.selectedChallengeId);
  const placedCount = useStore((s) => s.placedCount);
  const { getChallengeConstraints } = useScenario();

  // In challenge mode, filter by constraints
  const constraints =
    mode === 'challenges' && selectedChallengeId
      ? getChallengeConstraints(selectedChallengeId)
      : null;
  const allowedTypes = constraints?.availableObjects ?? null;
  const maxObjects = constraints?.maxObjects ?? Infinity;
  const canPlace = placedCount < maxObjects;

  const items = allowedTypes
    ? OBJECTS.filter((o) => allowedTypes.includes(o.type))
    : OBJECTS;

  return (
    <div className={styles.palette}>
      <h3 className={styles.title}>Objects</h3>
      <div className={styles.list}>
        {items.map((obj) => {
          const isSelected = selectedObjectType === obj.type;
          const disabled =
            canPlace === false && !isSelected && allowedTypes !== null;
          return (
            <button
              key={obj.type}
              type="button"
              className={`${styles.item} ${isSelected ? styles.selected : ''} ${disabled ? styles.disabled : ''}`}
              onClick={() =>
                setSelectedObjectType(disabled ? null : isSelected ? null : obj.type)
              }
              disabled={disabled}
              title={obj.label}
            >
              <span className={styles.icon}>{obj.icon}</span>
              <span className={styles.label}>{obj.label}</span>
            </button>
          );
        })}
      </div>
      {allowedTypes && (
        <p className={styles.limit}>
          {placedCount} / {maxObjects} placed
        </p>
      )}
    </div>
  );
}
