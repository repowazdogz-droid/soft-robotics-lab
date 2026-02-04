export type TrainingRole = "leadership" | "staff" | "parents" | "trusts";
export type TrainingTime = "3" | "7" | "15";
export type TrainingTopic = "predictability" | "repair" | "adult-regulation" | "boundaries" | "inspection";

export type Scenario = {
  id: string;
  title: string;
  role: TrainingRole;
  topic: TrainingTopic;
  time: TrainingTime;
  situation: string;
  constraints: string[]; // safety / boundaries reminders
  questions: Question[];
  answerKey: AnswerKeyItem[];
  outputs: {
    checklistTitle: string;
    checklist: string[];
    briefingTitle: string;
    briefingScript: string[]; // short staff briefing
    parentTitle?: string;
    parentTemplate?: string[]; // optional for parents-facing
    notes?: string[];
  };
};

export type Question =
  | {
      kind: "MCQ";
      id: string;
      prompt: string;
      options: { id: string; label: string }[];
      correctId: string;
      rationale: string;
    }
  | {
      kind: "SHORT";
      id: string;
      prompt: string;
      rubric: string[]; // what a good answer includes
      example: string; // safe exemplar
    };

export type AnswerKeyItem = {
  title: string;
  whatToSay: string[];
  whatToDo: string[];
  whatToAvoid: string[];
};

export function roles(): { id: TrainingRole; label: string; desc: string }[] {
  return [
    { id: "leadership", label: "Leadership", desc: "Systems, implementation, inspection framing." },
    { id: "staff", label: "Staff", desc: "Classroom response, predictability, repair." },
    { id: "parents", label: "Parents", desc: "Communication tone, boundaries, predictability at home." },
    { id: "trusts", label: "Trusts / LA", desc: "Governance, scale without founder dependency." },
  ];
}

export function topics(): { id: TrainingTopic; label: string; desc: string }[] {
  return [
    { id: "predictability", label: "Predictability", desc: "Reduce stress with stable routines and expectations." },
    { id: "repair", label: "Repair", desc: "Rupture → repair without shame or escalation." },
    { id: "adult-regulation", label: "Adult regulation", desc: "Support adults to respond, not react." },
    { id: "boundaries", label: "Boundaries", desc: "Stop lines: scope and responsibility boundaries." },
    { id: "inspection", label: "Inspection framing", desc: "Clear, inspection-safe language." },
  ];
}

export function scenarios(): Scenario[] {
  return [
    {
      id: "staff-predictability-7",
      title: "A volatile transition (end of lunch → lesson)",
      role: "staff",
      topic: "predictability",
      time: "7",
      situation:
        "After lunch, the class re-enters loudly. Two pupils refuse to sit. One adult starts raising their voice to regain control. The room escalates quickly.",
      constraints: [
        "No child diagnosis or 'why'. Focus on adult actions and environment.",
        "No behaviour-plan design here. This is a brief response + system tweak.",
        "If safeguarding indicators appear, follow school policy immediately.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "What is the best *first* move in the next 20 seconds?",
          options: [
            { id: "a", label: "Issue consequences immediately to reassert authority." },
            { id: "b", label: "Lower the temperature: reduce verbal load and make the next step predictable." },
            { id: "c", label: "Ask the pupils to explain why they are acting this way." },
            { id: "d", label: "Send both pupils out as quickly as possible." },
          ],
          correctId: "b",
          rationale:
            "Predictability reduces stress. In a hot moment, short, calm, concrete next steps beat explanation or punishment.",
        },
        {
          kind: "MCQ",
          id: "q2",
          prompt: "Which sentence is most aligned with this system?",
          options: [
            { id: "a", label: "'You're being disruptive—stop now or you'll be sanctioned.'" },
            { id: "b", label: "'We're safe. Two steps: bags down, then seats. I'll wait.'" },
            { id: "c", label: "'What's wrong with you today?'" },
            { id: "d", label: "'If you don't sit, you're choosing to fail.'" },
          ],
          correctId: "b",
          rationale:
            "Short, non-interpretive, stepwise instruction + calm waiting. No blame, no story, no escalation.",
        },
        {
          kind: "SHORT",
          id: "q3",
          prompt:
            "Write a 2-sentence 'reset' script you could use in that moment (keep it neutral, predictable, non-shaming).",
          rubric: [
            "Names safety and next steps (1–2 steps max).",
            "Neutral tone; no 'why', no blame.",
            "Offers a small choice if possible ('you can stand by your chair or sit').",
          ],
          example:
            `We're safe. Next step: bags down, then choose—stand by your chair or sit. I'll wait and then we'll begin.`,
        },
      ],
      answerKey: [
        {
          title: "What good looks like (in the room)",
          whatToSay: [
            "Short sentences. One instruction at a time.",
            "Name the next step and wait.",
            "Use 'when… then…' not threats.",
          ],
          whatToDo: [
            "Reduce verbal load; slow your pace.",
            "Make the transition routine visible (board / timer / steps).",
            "After: repair briefly and reset the routine.",
          ],
          whatToAvoid: [
            "Public power struggles.",
            "Asking for explanations mid-escalation.",
            "Long lectures or stacked instructions.",
          ],
        },
      ],
      outputs: {
        checklistTitle: "7-minute transition stabiliser (staff)",
        checklist: [
          "Before lunch ends: post the 2-step return routine visibly.",
          "Use a consistent entry cue (timer / phrase / gesture).",
          "In the moment: 1–2 steps, calm wait, minimal words.",
          "Offer a small choice that preserves dignity.",
          "After: 30-second repair ('We reset. Next time we'll do steps 1–2.').",
        ],
        briefingTitle: "Staff briefing (60 seconds)",
        briefingScript: [
          "Today's focus: predictable transitions reduce stress.",
          "We will use the same 2-step return routine after lunch.",
          "If it heats up: fewer words, name the next step, calm wait.",
          "We repair after, not during. No public power struggles.",
        ],
        parentTitle: "Optional parent message (neutral)",
        parentTemplate: [
          "This term we're strengthening predictable routines to support calm learning.",
          "You may hear your child mention 'steps' or 'reset' language—this is part of consistent classroom routines.",
          "If you have questions about routines, please contact the school. Safeguarding concerns follow our safeguarding procedures.",
        ],
        notes: [
          "This is not a behaviour programme. It's a predictable-environment micro-practice.",
        ],
      },
    },
    {
      id: "leadership-inspection-7",
      title: "Inspection conversation: 'Is this therapy?'",
      role: "leadership",
      topic: "inspection",
      time: "7",
      situation:
        "A governor/inspector asks whether your trauma-informed work is 'therapeutic' and how you ensure boundaries and safeguarding remain clear.",
      constraints: [
        "No efficacy claims. No 'research proves' language.",
        "Keep it system-level: adult practice, predictability, repair, safeguarding boundaries.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "Which framing is safest and most accurate?",
          options: [
            { id: "a", label: "'We treat trauma through staff training.'" },
            { id: "b", label: "'We're an evidence-based intervention that improves outcomes.'" },
            { id: "c", label: "'This is a school-safe framework for adult responses and predictable environments; safeguarding remains unchanged.'" },
            { id: "d", label: "'We diagnose early and intervene.'" },
          ],
          correctId: "c",
          rationale:
            "System-level, non-therapeutic, non-claiming, and boundary-clear.",
        },
        {
          kind: "SHORT",
          id: "q2",
          prompt: "Write a 3-sentence inspection-safe description you can reuse.",
          rubric: [
            "States what it is (framework for adult practice and environments).",
            "States what it focuses on (adult practice, predictable environments, organisational responses).",
            "States safeguarding remains with school policy.",
          ],
          example:
            `We use a trauma-informed *framework* to improve safety and predictability in adult practice and routines. This system focuses on adult practice, predictable environments, and organisational responses. Child-specific concerns should always be addressed through the school's safeguarding and statutory processes.`,
        },
      ],
      answerKey: [
        {
          title: "What good looks like (stakeholder confidence)",
          whatToSay: [
            "'Framework', 'adult practice', 'predictability', 'repair'.",
            "'Focuses on adult practice and environments' explicitly.",
            "'Safeguarding unchanged' explicitly.",
          ],
          whatToDo: [
            "Have agreed wording in SLT.",
            "Document boundaries in staff induction.",
            "Keep a clear handoff: wellbeing support ≠ safeguarding.",
          ],
          whatToAvoid: [
            "Outcome promises.",
            "Clinical language.",
            "Child-specific speculation.",
          ],
        },
      ],
      outputs: {
        checklistTitle: "Inspection-ready boundary checklist (leadership)",
        checklist: [
          "Agreed 3-sentence description shared by SLT.",
          "Written boundary: scope focuses on adult practice and system design.",
          "Safeguarding escalation routes reaffirmed.",
          "Staff know what to do when unsure (DSL).",
        ],
        briefingTitle: "60-second leadership briefing",
        briefingScript: [
          "We use a framework for adult practice and predictable routines.",
          "This system focuses on adult practice, predictable environments, and organisational responses.",
          "Safeguarding policy remains unchanged and is always the escalation route.",
        ],
      },
    },
    // ADD: 6 more scenarios (2 Staff, 2 Leadership, 2 Parents)

    // STAFF (3 min) — Repair micro-script
    {
      id: "staff-repair-3",
      title: "After a clash: micro-repair in 30 seconds",
      role: "staff",
      topic: "repair",
      time: "3",
      situation:
        "A pupil shouted at you. You raised your voice. The class is watching. You have a tiny window to repair without turning it into a 'big chat'.",
      constraints: [
        "No child diagnosis or 'why'. Focus on adult actions and environment.",
        "Repair is brief, behavioural, and forward-moving.",
        "If safeguarding indicators appear, follow school policy immediately.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "Which repair move fits this system best?",
          options: [
            { id: "a", label: "Explain why you raised your voice and ask them to apologise." },
            { id: "b", label: "Name the moment, own your part, and reset the next step." },
            { id: "c", label: "Ignore it completely so it doesn't look weak." },
            { id: "d", label: "Tell the class this is what trauma looks like." },
          ],
          correctId: "b",
          rationale:
            "Repair is brief, behavioural, and forward-moving. It restores safety without extracting apology or disclosure.",
        },
        {
          kind: "SHORT",
          id: "q2",
          prompt: "Write a one-sentence micro-repair you could say now (no blame, no therapy language).",
          rubric: [
            "Aim: 1) name, 2) own, 3) next step. Keep it short. No analysis of the child.",
          ],
          example:
            `That came out sharper than I meant. I'm resetting. Next step: line up quietly and we'll start again.`,
        },
      ],
      answerKey: [
        {
          title: "What good looks like (micro-repair)",
          whatToSay: [
            "Name the moment (brief)",
            "Own your part (one line)",
            "State the next step (clear, simple)",
          ],
          whatToDo: [
            "Keep it under 30 seconds",
            "No lecture, no diagnosis, no public negotiation",
            "Reset and move forward",
          ],
          whatToAvoid: [
            "Extracting apologies",
            "Long explanations",
            "Public power struggles",
          ],
        },
      ],
      outputs: {
        checklistTitle: "Micro-repair: 30-second reset",
        checklist: [
          "Name the moment (brief)",
          "Own your part (one line)",
          "State the next step (clear, simple)",
          "No lecture, no diagnosis, no public negotiation",
        ],
        briefingTitle: "Staff briefing (30 seconds)",
        briefingScript: [
          "Repair is brief and forward-moving.",
          "Name, own, next step—then reset.",
          "No apologies required, no long chats.",
        ],
      },
    },

    // STAFF (15 min) — Adult regulation under load
    {
      id: "staff-adultreg-15",
      title: "You're dysregulated: staying safe and predictable",
      role: "staff",
      topic: "adult-regulation",
      time: "15",
      situation:
        "You're carrying stress and feel close to snapping. The room is testing boundaries. You need a plan that protects you and the class.",
      constraints: [
        "No child diagnosis or 'why'. Focus on adult actions and environment.",
        "Adults first, children benefit. When adults are overloaded, complexity backfires.",
        "If safeguarding indicators appear, follow school policy immediately.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "What is the system's first priority here?",
          options: [
            { id: "a", label: "Push through and 'win' the behaviour battle." },
            { id: "b", label: "Increase adult regulation capacity and reduce load before adding strategies." },
            { id: "c", label: "Run a feelings circle so everyone can share." },
            { id: "d", label: "Remove consequences to avoid conflict." },
          ],
          correctId: "b",
          rationale:
            "Adults first, children benefit. When adults are overloaded, complexity backfires. Reduce load, increase clarity.",
        },
        {
          kind: "MCQ",
          id: "q2",
          prompt: "Which adjustment is most 'predictability first'?",
          options: [
            { id: "a", label: "Add new rules mid-lesson." },
            { id: "b", label: "Shrink the task, tighten transitions, and state the next 2 minutes clearly." },
            { id: "c", label: "Threaten detentions to regain control." },
            { id: "d", label: "Ask pupils to decide the lesson plan." },
          ],
          correctId: "b",
          rationale:
            "Predictability is a stabiliser. Small, clear steps reduce activation and protect classroom safety.",
        },
        {
          kind: "SHORT",
          id: "q3",
          prompt: "Write a 2-step plan for the next 5 minutes (adult move + environment move).",
          rubric: [
            "Keep it simple: one adult regulation move + one environmental simplification. No new programmes.",
          ],
          example:
            "Adult: take 2 slow breaths, lower voice, use a fixed script. Environment: set a 2-minute timer and give a single next instruction.",
        },
      ],
      answerKey: [
        {
          title: "What good looks like (5-minute stabiliser)",
          whatToSay: [
            "Use a fixed script (same words, calm tone)",
            "State the next 2 minutes clearly",
          ],
          whatToDo: [
            "Reduce load (shorten task, fewer transitions)",
            "Ask for help early (handoff, cover, reset)",
            "After: quick repair + reset, no post-mortem",
          ],
          whatToAvoid: [
            "Adding complexity",
            "Power struggles",
            "Long explanations",
          ],
        },
      ],
      outputs: {
        checklistTitle: "5-minute stabiliser (adult + environment)",
        checklist: [
          "Reduce load (shorten task, fewer transitions)",
          "Use a fixed script (same words, calm tone)",
          "Ask for help early (handoff, cover, reset)",
          "After: quick repair + reset, no post-mortem",
        ],
        briefingTitle: "Staff briefing (60 seconds)",
        briefingScript: [
          "When you're overloaded, reduce complexity first.",
          "Shrink the task, tighten transitions, use a fixed script.",
          "Ask for help early—it's not weakness, it's safety.",
        ],
      },
    },

    // LEADERSHIP (3 min) — Boundary request from staff
    {
      id: "leadership-boundaries-3",
      title: "Staff ask: 'Tell us what's wrong with this child'",
      role: "leadership",
      topic: "boundaries",
      time: "3",
      situation:
        "A stressed staff member asks for an explanation of a child's behaviour and wants a 'trauma-informed plan'. You must respond safely and clearly.",
      constraints: [
        "No child diagnosis or 'why'. Focus on adult actions and environment.",
        "This system is adult-and-environment focused. Individual child interpretation is outside scope.",
        "If safeguarding indicators appear, follow school policy immediately.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "Which response best fits the system boundaries?",
          options: [
            { id: "a", label: "Diagnose informally and suggest triggers to avoid." },
            { id: "b", label: "Refuse child-level analysis; redirect to environment + safeguarding pathway." },
            { id: "c", label: "Promise a behaviour plan template for the child." },
            { id: "d", label: "Ask the staff member to share their own trauma history to 'understand' the child." },
          ],
          correctId: "b",
          rationale:
            "This system is adult-and-environment focused. Individual child interpretation is outside scope and risks harm.",
        },
        {
          kind: "SHORT",
          id: "q2",
          prompt: "Write a one-paragraph leadership reply (calm, bounded, practical).",
          rubric: [
            "Refuse clearly, then offer a safe adjacent next step (predictability, repair, support, safeguarding if needed).",
          ],
          example:
            `We can't analyse or label individual children. What we can do is tighten predictability, clarify routines, and support staff responses. If there are safeguarding concerns, we follow the safeguarding process immediately. Let's review the environment triggers and agree a simple, consistent response plan for adults.`,
        },
      ],
      answerKey: [
        {
          title: "What good looks like (boundary script)",
          whatToSay: [
            "We don't diagnose or interpret children",
            "We change adult responses + predictability",
            "Safeguarding concerns → safeguarding policy",
          ],
          whatToDo: [
            "Redirect to environment review",
            "Agree a simple, consistent response plan for adults",
            "Support staff capacity before adding initiatives",
          ],
          whatToAvoid: [
            "Child-level analysis",
            "Promising behaviour plans",
            "Clinical language",
          ],
        },
      ],
      outputs: {
        checklistTitle: "Boundary script: child-level requests",
        checklist: [
          "We don't diagnose or interpret children",
          "We change adult responses + predictability",
          "Safeguarding concerns → safeguarding policy",
          "Support staff capacity before adding initiatives",
        ],
        briefingTitle: "60-second leadership briefing",
        briefingScript: [
          "When staff ask for child-level analysis, redirect to environment + adult responses.",
          "We don't diagnose. We change predictability and support staff consistency.",
          "Safeguarding concerns follow safeguarding policy immediately.",
        ],
      },
    },

    // LEADERSHIP (15 min) — Implementation cycle: removing load
    {
      id: "leadership-implementation-15",
      title: "90-day cycle: what do you remove to make space?",
      role: "leadership",
      topic: "predictability",
      time: "15",
      situation:
        "You want to 'implement trauma-informed practice', but staff are overloaded. You must protect time or the system fails.",
      constraints: [
        "No child diagnosis or 'why'. Focus on adult actions and environment.",
        "Time protection is a non-negotiable. Adding without removing increases load.",
        "If safeguarding indicators appear, follow school policy immediately.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "Which rule is most aligned with V1 implementation cycles?",
          options: [
            { id: "a", label: "Add training on top of everything and hope it sticks." },
            { id: "b", label: "Before adding, remove or pause something to create protected time." },
            { id: "c", label: "Make it optional so you can still claim you offered it." },
            { id: "d", label: "Roll it out as a one-off inset day." },
          ],
          correctId: "b",
          rationale:
            "Time protection is a non-negotiable. Adding without removing increases load and reduces safety.",
        },
        {
          kind: "SHORT",
          id: "q2",
          prompt: "List 2 things you could pause/remove for 90 days to protect time (systems-level, not 'try harder').",
          rubric: [
            "Choose low-value tasks, duplicative meetings, or competing initiatives—anything that steals attention from predictable routines.",
          ],
          example:
            "1) Pause a non-essential data tracking cycle. 2) Replace one meeting per fortnight with a 10-minute predictable reflection loop.",
        },
      ],
      answerKey: [
        {
          title: "What good looks like (90-day protection)",
          whatToSay: [
            "Name what you will pause/remove",
            "Protect a small, predictable slot",
            "Keep language inspection-safe (no therapy claims)",
          ],
          whatToDo: [
            "Choose low-value tasks or competing initiatives",
            "Measure process integrity, not child outcomes",
            "Support staff capacity before adding",
          ],
          whatToAvoid: [
            "Adding without removing",
            "Making it optional",
            "One-off rollouts",
          ],
        },
      ],
      outputs: {
        checklistTitle: "90-day protection checklist",
        checklist: [
          "Name what you will pause/remove",
          "Protect a small, predictable slot",
          "Keep language inspection-safe (no therapy claims)",
          "Measure process integrity, not child outcomes",
        ],
        briefingTitle: "60-second leadership briefing",
        briefingScript: [
          "Before adding, remove or pause something to create protected time.",
          "Choose low-value tasks or competing initiatives.",
          "Measure process integrity, not child outcomes.",
        ],
      },
    },

    // PARENTS (3 min) — Neutral message to school
    {
      id: "parents-communication-3",
      title: "Messaging school: calm, factual, non-escalating",
      role: "parents",
      topic: "boundaries",
      time: "3",
      situation:
        "You're worried about your child's day. You need to contact school without accusations or 'diagnosis talk', and ask for predictable support.",
      constraints: [
        "No child diagnosis or 'why'. Focus on adult actions and environment.",
        "Neutral tone lowers threat. Specific requests are easier for schools to act on.",
        "If safeguarding indicators appear, follow school policy immediately.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "Which message style is most likely to reduce friction?",
          options: [
            { id: "a", label: "Threaten complaint unless they admit they caused trauma." },
            { id: "b", label: "Short, factual, specific request for predictability and communication." },
            { id: "c", label: "A long narrative explaining your child's psychology." },
            { id: "d", label: "Demand the teacher apologise immediately." },
          ],
          correctId: "b",
          rationale:
            "Neutral tone lowers threat. Specific requests are easier for schools to act on and document safely.",
        },
        {
          kind: "SHORT",
          id: "q2",
          prompt: "Draft a 3–5 sentence message (factual + one clear request).",
          rubric: [
            "Keep it short: facts, impact, request, next step. No diagnosis or blame.",
          ],
          example:
            `Hi, I'm checking in about today. My child came home very upset after the transition after lunch. Could we agree a predictable plan for that transition and a brief note if it becomes difficult? Thank you.`,
        },
      ],
      answerKey: [
        {
          title: "What good looks like (parent message)",
          whatToSay: [
            "Facts (what happened)",
            "Impact (what you observed)",
            "One clear request (predictability/communication)",
            "Respectful close + next step",
          ],
          whatToDo: [
            "Keep it short and specific",
            "Avoid diagnosis or blame",
            "Request predictable support",
          ],
          whatToAvoid: [
            "Threats or accusations",
            "Long narratives",
            "Demanding apologies",
          ],
        },
      ],
      outputs: {
        checklistTitle: "Parent message template",
        checklist: [
          "Facts (what happened)",
          "Impact (what you observed)",
          "One clear request (predictability/communication)",
          "Respectful close + next step",
        ],
        briefingTitle: "Parent briefing (30 seconds)",
        briefingScript: [
          "Keep messages short, factual, and specific.",
          "Request predictable support and clear communication.",
          "Respectful tone reduces friction and increases cooperation.",
        ],
        parentTitle: "Example message",
        parentTemplate: [
          "Hi, I'm checking in about today. My child came home very upset after the transition after lunch.",
          "Could we agree a predictable plan for that transition and a brief note if it becomes difficult?",
          "Thank you.",
        ],
      },
    },

    // PARENTS (15 min) — Home predictability routine
    {
      id: "parents-predictability-15",
      title: "Home routine: predictable evenings without lectures",
      role: "parents",
      topic: "predictability",
      time: "15",
      situation:
        "Evenings are tense. You want a predictable routine that reduces conflict without turning home into 'behaviour management'.",
      constraints: [
        "No child diagnosis or 'why'. Focus on adult actions and environment.",
        "Predictability reduces activation. Small choices support agency without losing structure.",
        "If safeguarding indicators appear, follow school policy immediately.",
      ],
      questions: [
        {
          kind: "MCQ",
          id: "q1",
          prompt: "Which change best matches 'predictability first'?",
          options: [
            { id: "a", label: "New rules daily depending on mood." },
            { id: "b", label: "Same 3-step routine with visual/clear sequence and small choices." },
            { id: "c", label: "Extended negotiations until agreement." },
            { id: "d", label: "High consequences for minor slips." },
          ],
          correctId: "b",
          rationale:
            "Predictability reduces activation. Small choices support agency without losing structure.",
        },
        {
          kind: "SHORT",
          id: "q2",
          prompt: "Write a simple 3-step evening sequence + one choice you can offer.",
          rubric: [
            "Keep it small and repeatable. Avoid long explanations when stress is high.",
          ],
          example:
            "Sequence: 1) snack + sit, 2) wash + pyjamas, 3) story + lights. Choice: which story first or which pyjamas.",
        },
      ],
      answerKey: [
        {
          title: "What good looks like (predictable evening)",
          whatToSay: [
            "Use the same sequence daily",
            "Offer 1 small choice within structure",
            "Keep language calm + short",
          ],
          whatToDo: [
            "Repair fast after ruptures (no lecture)",
            "Make the sequence visible (chart/timer)",
            "Stick to the routine even when it's hard",
          ],
          whatToAvoid: [
            "New rules daily",
            "Extended negotiations",
            "High consequences",
          ],
        },
      ],
      outputs: {
        checklistTitle: "Predictable evening routine (3-step)",
        checklist: [
          "Use the same sequence daily",
          "Offer 1 small choice within structure",
          "Keep language calm + short",
          "Repair fast after ruptures (no lecture)",
        ],
        briefingTitle: "Parent briefing (60 seconds)",
        briefingScript: [
          "Predictable routines reduce stress at home.",
          "Use the same 3-step sequence daily with one small choice.",
          "Repair fast after ruptures—no lectures, just reset.",
        ],
        parentTitle: "Example routine",
        parentTemplate: [
          "Evening sequence: 1) snack + sit, 2) wash + pyjamas, 3) story + lights.",
          "Choice: which story first or which pyjamas.",
          "Keep it the same every day, even when it's hard.",
        ],
      },
    },
  ];
}

export function filterScenarios(input: {
  role: TrainingRole;
  time: TrainingTime;
  topic?: TrainingTopic | "any";
}) {
  const all = scenarios();
  return all.filter((s) => {
    if (s.role !== input.role) return false;
    if (s.time !== input.time) return false;
    if (input.topic && input.topic !== "any" && s.topic !== input.topic) return false;
    return true;
  });
}

export function buildDraftOutputs(s: Scenario, context: { schoolType?: string; audienceNote?: string }) {
  // Template-driven "AI-ish" drafting with guardrails.
  // Must remain generic: no child-level advice, no diagnosis, no predictions.
  const prefix: string[] = [];
  if (context.schoolType) prefix.push(`Context: ${context.schoolType}.`);
  if (context.audienceNote) prefix.push(`Audience note: ${context.audienceNote}.`);

  const brief = [
    ...prefix,
    ...s.outputs.briefingScript,
  ];
  const checklist = s.outputs.checklist;
  const parent = s.outputs.parentTemplate ?? [];

  return { brief, checklist, parent };
}

