import type { CaseSynthesisOutput } from '../../types';
import { SectionCard } from './SectionCard';

interface Props {
  output: CaseSynthesisOutput;
  onReset: () => void;
}

export function CaseOutputView({ output, onReset }: Props) {
  return (
    <div className="space-y-6">
      <div className="flex flex-wrap justify-between items-center gap-2">
        <button
          type="button"
          onClick={onReset}
          className="text-sm text-slate-600 hover:text-slate-800"
        >
          ← New Case
        </button>
        <span className="text-xs text-slate-400">
          Generated {new Date(output.generatedAt).toLocaleTimeString()}
        </span>
      </div>

      <SectionCard title="Case Synthesis" id="A">
        <p className="text-slate-700 mb-4">{output.caseSynthesis.summary}</p>

        {output.caseSynthesis.tensions.length > 0 && (
          <div className="mb-4">
            <h4 className="text-sm font-medium text-slate-600 mb-2">Tensions or Inconsistencies</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.caseSynthesis.tensions.map((t, i) => <li key={i}>{t}</li>)}
            </ul>
          </div>
        )}

        {output.caseSynthesis.missingInformation.length > 0 && (
          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Information That Appears Missing or Unclear</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.caseSynthesis.missingInformation.map((m, i) => <li key={i}>{m}</li>)}
            </ul>
          </div>
        )}
      </SectionCard>

      <SectionCard title="Considerations in Similar Cases" id="B">
        <p className="text-xs text-slate-500 mb-4 italic">
          These are factors clinicians commonly deliberate about, not recommendations.
        </p>

        <div className="grid gap-4">
          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Conservative Management Considerations</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.considerations.conservativeFactors.map((f, i) => <li key={i}>{f}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Interventional Considerations</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.considerations.interventionalFactors.map((f, i) => <li key={i}>{f}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Surgical Considerations</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.considerations.surgicalFactors.map((f, i) => <li key={i}>{f}</li>)}
            </ul>
          </div>

          {output.considerations.areasOfDebate.length > 0 && (
            <div>
              <h4 className="text-sm font-medium text-slate-600 mb-2">Areas Where Experts Commonly Differ</h4>
              <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
                {output.considerations.areasOfDebate.map((a, i) => <li key={i}>{a}</li>)}
              </ul>
            </div>
          )}
        </div>
      </SectionCard>

      <SectionCard title="Risk and Regret Awareness" id="C">
        <p className="text-xs text-slate-500 mb-4 italic">
          Qualitative considerations only. No probabilities.
        </p>

        <div className="space-y-4">
          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Failure Modes Observed in Similar Cases</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.riskRegretMap.commonFailureModes.map((f, i) => <li key={i}>{f}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Patient-Specific Complexity Factors</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.riskRegretMap.patientSpecificFactors.map((f, i) => <li key={i}>{f}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Common Drivers of Post-Treatment Dissatisfaction</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.riskRegretMap.dissatisfactionDrivers.map((d, i) => <li key={i}>{d}</li>)}
            </ul>
          </div>
        </div>
      </SectionCard>

      <SectionCard title="Assumptions and Sensitivities" id="D">
        <div className="space-y-4">
          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Current Assumptions</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.assumptions.currentAssumptions.map((a, i) => <li key={i}>{a}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Would Materially Change Thinking</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.assumptions.wouldChangeThinking.map((w, i) => <li key={i}>{w}</li>)}
            </ul>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div>
              <h4 className="text-sm font-medium text-slate-600 mb-2">Possibly Overweighted</h4>
              <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
                {output.assumptions.possibleOverweighting.map((o, i) => <li key={i}>{o}</li>)}
              </ul>
            </div>
            <div>
              <h4 className="text-sm font-medium text-slate-600 mb-2">Possibly Underweighted</h4>
              <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
                {output.assumptions.possibleUnderweighting.map((u, i) => <li key={i}>{u}</li>)}
              </ul>
            </div>
          </div>
        </div>
      </SectionCard>

      <SectionCard title="Patient Conversation Framing" id="E">
        <p className="text-xs text-slate-500 mb-4 italic">
          Language that may support shared decision-making discussions.
        </p>

        <div className="space-y-4">
          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Plain-Language Explanation</h4>
            <p className="text-sm text-slate-700 bg-slate-50 p-3 rounded-lg">
              {output.patientFraming.plainLanguageExplanation}
            </p>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Outcome Framing</h4>
            <p className="text-sm text-slate-700">{output.patientFraming.outcomeRanges}</p>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Uncertainty Statements</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.patientFraming.uncertaintyStatements.map((u, i) => <li key={i}>{u}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Shared Decision-Making Prompts</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.patientFraming.sharedDecisionPrompts.map((p, i) => <li key={i}>{p}</li>)}
            </ul>
          </div>
        </div>
      </SectionCard>

      <SectionCard title="Cognitive Checklist" id="F" className="bg-slate-50">
        <p className="text-xs text-slate-500 mb-4 italic">
          For clinician reflection. Not for documentation.
        </p>

        <div className="space-y-4">
          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Pause Questions</h4>
            <ul className="space-y-2">
              {output.cognitiveChecklist.pauseQuestions.map((q, i) => (
                <li key={i} className="flex items-start gap-2 text-sm text-slate-700">
                  <span className="text-slate-400">□</span>
                  {q}
                </li>
              ))}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-amber-700 mb-2">Proceed With Caution If...</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-amber-800">
              {output.cognitiveChecklist.doNotProceedUnless.map((d, i) => <li key={i}>{d}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-slate-600 mb-2">Second-Order Effects</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-slate-600">
              {output.cognitiveChecklist.secondOrderEffects.map((s, i) => <li key={i}>{s}</li>)}
            </ul>
          </div>
        </div>
      </SectionCard>

      <SectionCard title="Limitations of This Synthesis" id="G" className="border-amber-200 bg-amber-50">
        <div className="space-y-4">
          <div>
            <h4 className="text-sm font-medium text-amber-800 mb-2">This Analysis Cannot Determine</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-amber-900">
              {output.limitations.cannotDetermine.map((c, i) => <li key={i}>{c}</li>)}
            </ul>
          </div>

          <div>
            <h4 className="text-sm font-medium text-amber-800 mb-2">Requires Clinical Judgment</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-amber-900">
              {output.limitations.requiresClinicalJudgment.map((r, i) => <li key={i}>{r}</li>)}
            </ul>
          </div>
        </div>
      </SectionCard>

      <div className="p-4 border border-slate-200 rounded-lg bg-white">
        <p className="text-xs text-slate-500 text-center">
          {output.disclaimer}
        </p>
      </div>
    </div>
  );
}
