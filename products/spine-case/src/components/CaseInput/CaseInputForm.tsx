import { useState } from 'react';
import type { CaseInput } from '../../types';
import { checkInputSafety } from '../../services/safetyFilter';

interface Props {
  onSubmit: (input: CaseInput) => void;
  initialData: CaseInput | null;
}

export function CaseInputForm({ onSubmit, initialData }: Props) {
  const [formData, setFormData] = useState<CaseInput>(initialData ?? {
    age: null,
    sex: null,
    symptomDuration: '',
    comorbidities: '',
    priorTreatments: '',
    presentingSymptoms: '',
    imagingSummary: '',
    patientGoals: '',
    clinicianLeaning: '',
  });

  const [warning, setWarning] = useState<string | null>(null);

  const handleChange = (field: keyof CaseInput, value: string | number | null) => {
    setFormData(prev => ({ ...prev, [field]: value }));

    if (typeof value === 'string') {
      const safety = checkInputSafety(value);
      setWarning(safety.warning ?? null);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit(formData);
  };

  const isValid = formData.presentingSymptoms.trim().length > 0;

  return (
    <form onSubmit={handleSubmit} className="space-y-8">
      <section>
        <h2 className="text-lg font-medium text-slate-800 mb-4">Patient Information</h2>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
          <div>
            <label className="block text-sm font-medium text-slate-700 mb-1">Age</label>
            <input
              type="number"
              min={1}
              max={120}
              value={formData.age ?? ''}
              onChange={e => handleChange('age', e.target.value ? parseInt(e.target.value, 10) : null)}
              className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
              placeholder="Years"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-slate-700 mb-1">Sex</label>
            <select
              value={formData.sex ?? ''}
              onChange={e => handleChange('sex', (e.target.value || null) as CaseInput['sex'])}
              className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            >
              <option value="">Select...</option>
              <option value="male">Male</option>
              <option value="female">Female</option>
              <option value="other">Other</option>
            </select>
          </div>
        </div>

        <div className="mt-4">
          <label className="block text-sm font-medium text-slate-700 mb-1">Duration of Symptoms</label>
          <input
            type="text"
            value={formData.symptomDuration}
            onChange={e => handleChange('symptomDuration', e.target.value)}
            className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            placeholder="e.g., 6 months, 2 years"
          />
        </div>

        <div className="mt-4">
          <label className="block text-sm font-medium text-slate-700 mb-1">Relevant Comorbidities</label>
          <textarea
            value={formData.comorbidities}
            onChange={e => handleChange('comorbidities', e.target.value)}
            rows={2}
            className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            placeholder="e.g., Type 2 diabetes, hypertension, previous cardiac surgery"
          />
        </div>

        <div className="mt-4">
          <label className="block text-sm font-medium text-slate-700 mb-1">Prior Treatments Attempted</label>
          <textarea
            value={formData.priorTreatments}
            onChange={e => handleChange('priorTreatments', e.target.value)}
            rows={2}
            className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            placeholder="e.g., Physiotherapy x 12 weeks, epidural injections x 2, oral analgesia"
          />
        </div>
      </section>

      <section>
        <h2 className="text-lg font-medium text-slate-800 mb-4">Clinical Picture</h2>

        <div>
          <label className="block text-sm font-medium text-slate-700 mb-1">
            Presenting Symptoms <span className="text-red-500">*</span>
          </label>
          <textarea
            value={formData.presentingSymptoms}
            onChange={e => handleChange('presentingSymptoms', e.target.value)}
            rows={4}
            className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            placeholder="Describe the patient's presenting symptoms, including location, character, severity, and functional impact..."
            required
          />
        </div>

        <div className="mt-4">
          <label className="block text-sm font-medium text-slate-700 mb-1">Imaging Summary</label>
          <textarea
            value={formData.imagingSummary}
            onChange={e => handleChange('imagingSummary', e.target.value)}
            rows={4}
            className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            placeholder="Copy/paste relevant radiology report findings or summarise key imaging observations..."
          />
          <p className="mt-1 text-xs text-slate-500">You can paste directly from radiology reports</p>
        </div>

        <div className="mt-4">
          <label className="block text-sm font-medium text-slate-700 mb-1">Patient Goals & Expectations</label>
          <textarea
            value={formData.patientGoals}
            onChange={e => handleChange('patientGoals', e.target.value)}
            rows={3}
            className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            placeholder="What does the patient hope to achieve? What are their expectations and priorities?"
          />
        </div>

        <div className="mt-4">
          <label className="block text-sm font-medium text-slate-700 mb-1">Your Current Thinking (Optional)</label>
          <textarea
            value={formData.clinicianLeaning}
            onChange={e => handleChange('clinicianLeaning', e.target.value)}
            rows={3}
            className="w-full px-3 py-2 border border-slate-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-blue-500"
            placeholder="What are you currently considering? Any specific concerns or uncertainties you're weighing?"
          />
          <p className="mt-1 text-xs text-slate-500">This helps contextualise the synthesis but does not influence any recommendations (this tool does not make recommendations)</p>
        </div>
      </section>

      {warning && (
        <div className="p-4 bg-amber-50 border border-amber-200 rounded-lg">
          <p className="text-sm text-amber-800">{warning}</p>
        </div>
      )}

      <div className="pt-4">
        <button
          type="submit"
          disabled={!isValid}
          className="w-full py-3 px-4 bg-slate-800 text-white font-medium rounded-lg hover:bg-slate-700 disabled:bg-slate-300 disabled:cursor-not-allowed transition-colors"
        >
          Generate Case Synthesis
        </button>
        <p className="mt-2 text-xs text-center text-slate-500">
          This will generate a thinking aid. It will not provide diagnoses or recommendations.
        </p>
      </div>
    </form>
  );
}
