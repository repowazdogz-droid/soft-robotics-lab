import { useState, useEffect, useRef } from 'react';
import type { CaseInput, CaseSynthesisOutput, AppState } from './types';
import { useLocale } from './context/LocaleContext';
import { Header } from './components/shared/Header';
import { Disclaimer } from './components/shared/Disclaimer';
import { CaseInputForm } from './components/CaseInput/CaseInputForm';
import { CaseOutputView } from './components/CaseOutput/CaseOutputView';
import { LoadingState } from './components/shared/LoadingState';
import { generateCaseSynthesis } from './services/gemini';

export default function App() {
  const { locale } = useLocale();
  const [appState, setAppState] = useState<AppState>('input');
  const [caseInput, setCaseInput] = useState<CaseInput | null>(null);
  const [caseOutput, setCaseOutput] = useState<CaseSynthesisOutput | null>(null);
  const [error, setError] = useState<string | null>(null);
  const skipLocaleRegenRef = useRef(true);

  const handleSubmit = async (input: CaseInput) => {
    setCaseInput(input);
    setAppState('loading');
    setError(null);

    try {
      const output = await generateCaseSynthesis(input, locale);
      setCaseOutput(output);
      setAppState('output');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
      setAppState('input');
    }
  };

  // When user changes language while viewing output, re-generate synthesis in the new language
  useEffect(() => {
    if (appState !== 'output' || !caseInput) return;
    if (skipLocaleRegenRef.current) {
      skipLocaleRegenRef.current = false;
      return;
    }

    let cancelled = false;
    setAppState('loading');
    setError(null);

    generateCaseSynthesis(caseInput, locale)
      .then(output => {
        if (!cancelled) {
          setCaseOutput(output);
          setAppState('output');
        }
      })
      .catch(err => {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : 'An error occurred');
          setAppState('output');
        }
      });

    return () => { cancelled = true; };
  }, [locale]);

  const handleReset = () => {
    setAppState('input');
    setCaseOutput(null);
    setError(null);
  };

  return (
    <div className="min-h-screen bg-slate-50">
      <Header />
      <Disclaimer />

      <main className="max-w-4xl mx-auto px-4 py-8">
        {error && (
          <div className="mb-6 p-4 bg-red-50 border border-red-200 rounded-lg text-red-800">
            {error}
          </div>
        )}

        {appState === 'input' && (
          <CaseInputForm onSubmit={handleSubmit} initialData={caseInput} />
        )}

        {appState === 'loading' && (
          <LoadingState />
        )}

        {appState === 'output' && caseOutput && (
          <CaseOutputView output={caseOutput} onReset={handleReset} />
        )}
      </main>
    </div>
  );
}
