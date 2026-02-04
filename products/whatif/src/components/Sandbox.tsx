import { useRef, useEffect, useCallback } from 'react';
import { usePhysicsWorld } from '../hooks/usePhysicsWorld';
import { useScenario } from '../hooks/useScenario';
import { useStore } from '../store';
import styles from './Sandbox.module.css';

export function Sandbox() {
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  const { addObject, clear } = usePhysicsWorld(canvasRef, containerRef);

  const selectedScenarioId = useStore((s) => s.selectedScenarioId);
  const selectedChallengeId = useStore((s) => s.selectedChallengeId);
  const selectedObjectType = useStore((s) => s.selectedObjectType);
  const setSelectedObjectType = useStore((s) => s.setSelectedObjectType);
  const incrementPlacedCount = useStore((s) => s.incrementPlacedCount);

  const { getScenarioSetup, getChallengeSetup } = useScenario();

  const loadCurrentSetup = useCallback(() => {
    clear();
    if (selectedScenarioId) {
      const setup = getScenarioSetup(selectedScenarioId);
      if (setup) {
        setTimeout(() => {
          setup.objects.forEach((obj) => {
            addObject(obj.type, obj.x, obj.y, { angle: obj.angle });
          });
        }, 0);
      }
    } else if (selectedChallengeId) {
      const setup = getChallengeSetup(selectedChallengeId);
      if (setup) {
        setTimeout(() => {
          setup.objects.forEach((obj) => {
            addObject(obj.type, obj.x, obj.y, { angle: obj.angle });
          });
        }, 0);
      }
    }
  }, [
    selectedScenarioId,
    selectedChallengeId,
    getScenarioSetup,
    getChallengeSetup,
    clear,
    addObject,
  ]);

  useEffect(() => {
    loadCurrentSetup();
  }, [selectedScenarioId, selectedChallengeId]);

  useEffect(() => {
    const onReset = () => loadCurrentSetup();
    const onClear = () => clear();
    window.addEventListener('whatif-reset', onReset);
    window.addEventListener('whatif-clear', onClear);
    return () => {
      window.removeEventListener('whatif-reset', onReset);
      window.removeEventListener('whatif-clear', onClear);
    };
  }, [loadCurrentSetup, clear]);

  const handleClick = useCallback(
    (e: React.MouseEvent<HTMLDivElement>) => {
      const container = containerRef.current;
      if (!container) return;
      const rect = container.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      if (selectedObjectType) {
        addObject(selectedObjectType, x, y);
        incrementPlacedCount();
        setSelectedObjectType(null);
      }
    },
    [selectedObjectType, addObject, incrementPlacedCount, setSelectedObjectType]
  );

  return (
    <div
      ref={containerRef}
      className={styles.sandbox}
      onClick={handleClick}
      role="application"
      aria-label="Physics sandbox canvas"
    >
      <canvas ref={canvasRef} className={styles.canvas} />
    </div>
  );
}
