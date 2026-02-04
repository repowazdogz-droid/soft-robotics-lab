import { createContext, useContext, useState, useCallback, type ReactNode } from 'react';

export type Locale = 'en' | 'de' | 'fr';

const LOCALE_LABELS: Record<Locale, string> = {
  en: 'English',
  de: 'Deutsch',
  fr: 'Français',
};

interface LocaleContextValue {
  locale: Locale;
  setLocale: (locale: Locale) => void;
  localeLabel: (locale: Locale) => string;
}

const LocaleContext = createContext<LocaleContextValue | null>(null);

export function LocaleProvider({ children }: { children: ReactNode }) {
  const [locale, setLocaleState] = useState<Locale>('en');
  const setLocale = useCallback((next: Locale) => setLocaleState(next), []);
  const localeLabel = useCallback((l: Locale) => LOCALE_LABELS[l], []);

  return (
    <LocaleContext.Provider value={{ locale, setLocale, localeLabel }}>
      {children}
    </LocaleContext.Provider>
  );
}

export function useLocale() {
  const ctx = useContext(LocaleContext);
  if (!ctx) throw new Error('useLocale must be used within LocaleProvider');
  return ctx;
}

export { LOCALE_LABELS };
