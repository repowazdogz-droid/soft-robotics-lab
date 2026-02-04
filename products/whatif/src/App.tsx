import { useEffect, useRef } from 'react';
import { useStore } from './store';
import { Sandbox } from './components/Sandbox';
import { ControlPanel } from './components/ControlPanel';
import { ScenarioSelector } from './components/ScenarioSelector';
import { ObjectPalette } from './components/ObjectPalette';
import { WhyButton } from './components/WhyButton';
import { ChallengeMode } from './components/ChallengeMode';
import { ResultsOverlay } from './components/ResultsOverlay';
import { Controls } from './components/Controls';
import styles from './App.module.css';

function App() {
  const mode = useStore((s) => s.mode);
  const selectedScenarioId = useStore((s) => s.selectedScenarioId);
  const selectedChallengeId = useStore((s) => s.selectedChallengeId);

  const showCanvas =
    mode === 'sandbox' ||
    (mode === 'scenarios' && selectedScenarioId !== null) ||
    (mode === 'challenges' && selectedChallengeId !== null);

  const setResultsOverlayOpen = useStore((s) => s.setResultsOverlayOpen);
  const resultsOverlayShownRef = useRef(false);
  useEffect(() => {
    if (!showCanvas || (!selectedScenarioId && !selectedChallengeId)) {
      resultsOverlayShownRef.current = false;
      return;
    }
    resultsOverlayShownRef.current = false;
    const t = setTimeout(() => {
      if (!resultsOverlayShownRef.current) {
        resultsOverlayShownRef.current = true;
        setResultsOverlayOpen(true);
      }
    }, 5000);
    return () => clearTimeout(t);
  }, [showCanvas, selectedScenarioId, selectedChallengeId, setResultsOverlayOpen]);

  return (
    <div className={styles.app}>
      <nav className={styles.nav}>
        <NavLink current={mode === 'scenarios' && !selectedScenarioId} to="scenarios">
          What If?
        </NavLink>
        <NavLink current={mode === 'challenges' && !selectedChallengeId} to="challenges">
          Challenges
        </NavLink>
        <NavLink current={mode === 'sandbox'} to="sandbox">
          Sandbox
        </NavLink>
        {showCanvas && <WhyButton />}
      </nav>

      {!showCanvas && mode === 'scenarios' && <ScenarioSelector />}
      {!showCanvas && mode === 'challenges' && <ChallengeMode />}

      {showCanvas && (
        <>
          <div className={styles.main}>
            <div className={styles.canvasWrap}>
              <Sandbox />
            </div>
            <aside className={styles.sidebar}>
              <ControlPanel />
              <ObjectPalette />
            </aside>
          </div>
          <Controls />
        </>
      )}

      <ResultsOverlay />
    </div>
  );
}

function NavLink({
  current,
  to,
  children,
}: {
  current: boolean;
  to: 'scenarios' | 'challenges' | 'sandbox';
  children: React.ReactNode;
}) {
  const setMode = useStore((s) => s.setMode);
  const setSelectedScenarioId = useStore((s) => s.setSelectedScenarioId);
  const setSelectedChallengeId = useStore((s) => s.setSelectedChallengeId);

  const handleClick = () => {
    setMode(to);
    if (to === 'scenarios') setSelectedChallengeId(null);
    if (to === 'challenges') setSelectedScenarioId(null);
    if (to === 'sandbox') {
      setSelectedScenarioId(null);
      setSelectedChallengeId(null);
    }
  };

  return (
    <button
      type="button"
      className={current ? styles.navActive : styles.navLink}
      onClick={handleClick}
    >
      {children}
    </button>
  );
}

export default App;
