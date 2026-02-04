import { useLocale } from '../../context/LocaleContext';
import type { Locale } from '../../context/LocaleContext';

const LOCALES: Locale[] = ['en', 'de', 'fr'];

export function Header() {
  const { locale, setLocale, localeLabel } = useLocale();

  return (
    <header className="bg-white border-b border-slate-200">
      <div className="max-w-4xl mx-auto px-4 py-4 flex flex-wrap justify-between items-center gap-2">
        <div>
          <h1 className="text-xl font-semibold text-slate-800">Spine Case Synthesis</h1>
          <p className="text-sm text-slate-500">Pre-operative cognitive support tool</p>
        </div>
        <label className="flex items-center gap-2 text-sm text-slate-600">
          <span>Language</span>
          <select
            value={locale}
            onChange={e => setLocale(e.target.value as Locale)}
            className="border border-slate-300 rounded px-2 py-1 text-slate-800 bg-white"
          >
            {LOCALES.map(l => (
              <option key={l} value={l}>{localeLabel(l)}</option>
            ))}
          </select>
        </label>
      </div>
    </header>
  );
}
