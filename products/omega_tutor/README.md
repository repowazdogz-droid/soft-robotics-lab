# 🎓 OMEGA Tutor

**Adaptive Teaching with Personalized Learning**

Teach any topic at any level. Track how each person learns. Catch misconceptions before they stick.

---

## 🚀 Quick Start

```bash
cd products/omega_tutor
pip install -r requirements.txt
streamlit run app.py --server.port 8503
```

Open http://localhost:8503 in your browser.

---

## 💡 What Problem Does This Solve?

Everyone learns differently. Generic explanations don't work.

| Problem | Solution |
|---------|----------|
| One-size-fits-all teaching | 4 persona levels (kid → expert) |
| No feedback on understanding | Explain-back with scoring |
| Misconceptions persist | Detection and proactive correction |
| Progress forgotten | Learning fingerprint tracks everything |
| No spaced repetition | SM-2 algorithm for retention |

---

## 📦 Features

### 1. Four Teaching Personas

| Level | Style | Example |
|-------|-------|---------|
| **Kid** | Simple analogies, fun language | "Soft robots are like octopus arms!" |
| **Student** | Clear explanations with examples | "Pneumatic actuation works by..." |
| **Adult** | Professional, practical | "Key considerations for implementation..." |
| **Expert** | Technical, assumes background | "The constitutive model assumes..." |

Auto-adjusts based on questions and explain-back scores.

### 2. Explain-Back Evaluation
True understanding = can you teach it back?

1. Tutor explains concept
2. You explain it in your own words
3. Tutor scores your explanation (1-10)
4. Feedback on what you got right/wrong
5. Adapts difficulty based on score

**Scoring criteria:**
- Key concepts mentioned
- Technical accuracy
- Logical flow
- Depth of understanding

### 3. Learning Fingerprint
Tracks how YOU learn:

```
📊 Your Learning Profile

Topics studied: 12
Total events: 47

Strong areas: soft_robotics, materials
Working on: machine_learning, controls

You learn best with: analogies, examples
Optimal length: medium
```

**Tracks:**
- Which analogies worked for you
- Which explanations confused you
- Time-to-understand per concept
- Mastery score per topic
- Successful vs failed approaches

### 4. Misconception Detection
Catch errors before they stick:

**Example:**
```
Student: "Soft robots can't apply much force because they're soft"

⚠️ Misconception detected (severity: moderate)

Many people think: "Soft robots are weak and can't apply force"

Actually: Soft robots can apply significant force through pneumatic 
actuation, tendon drives, or granular jamming. Some can lift many 
times their weight.
```

**Preemptive warnings** for critical misconceptions shown before teaching.

**Built-in misconception database:**
- Soft robotics (3 misconceptions)
- Materials science (2)
- Machine learning (3)
- Physics (2)
- Biology (1)

### 5. Quiz System
Reinforce learning with quizzes:

- Multiple choice generated from topic
- Immediate feedback after each answer
- "Learn more" button for wrong answers
- Final summary with per-question breakdown
- SM-2 spaced repetition scheduling

### 6. Curriculum Builder
Structured learning paths:

```python
curriculum = {
    "soft-robotics-101": {
        "title": "Introduction to Soft Robotics",
        "topics": [
            {"id": "materials", "title": "Soft Materials"},
            {"id": "actuation", "title": "Actuation Methods"},
            {"id": "sensing", "title": "Proprioception"},
        ]
    }
}
```

Progress tracked per topic with mastery scores.

---

## 🏗️ Architecture

```
omega_tutor/
├── app.py                      # Streamlit UI
├── core/
│   ├── tutor_engine.py         # Teaching engine (LLM)
│   ├── level_adapter.py        # Level prompts
│   ├── level_adjust.py         # Simpler / deeper
│   ├── curriculum.py           # Structured curricula
│   ├── curriculum_progress.py  # Progress per curriculum
│   ├── quiz_generator.py       # Quiz generation
│   ├── explain_back.py        # Explain-back evaluation
│   ├── spaced_repetition.py    # SM-2 algorithm
│   ├── learning_fingerprint.py # Personal learning profile
│   ├── misconception_detector.py # Error detection
│   ├── progress.py             # Topic / question tracking
│   ├── user_profile.py         # Persona (kid/student/adult/expert)
│   ├── cognitive_load.py       # Overwhelm detection
│   └── voice.py                # TTS / STT (optional)
├── data/
│   ├── curricula/              # Curriculum JSONs
│   ├── learning_fingerprint.json
│   └── user_profile.json
└── prompts/
    └── tutor_system.py
```

---

## 🔄 Learning Loop

```
┌─────────────────────────────────────────────────────────────────┐
│   1. Pick topic (or ask anything)                               │
│   2. Tutor explains at your level                               │
│   3. You explain back                                           │
│   4. Tutor scores + catches misconceptions                      │
│   5. Quiz to reinforce                                          │
│   6. Fingerprint updates (what works for you)                   │
│   7. Next topic scheduled via SM-2                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📊 Explain-Back Scoring

| Score | Meaning | Action |
|-------|---------|--------|
| 9-10 | Expert understanding | Advance to harder topics |
| 7-8 | Solid understanding | Minor clarification, then advance |
| 5-6 | Partial understanding | Re-explain key points |
| 3-4 | Significant gaps | Simpler explanation, more examples |
| 1-2 | Major misunderstanding | Start from basics |

---

## 🧠 Misconception Types

| Severity | Handling |
|----------|----------|
| **Critical** | Preemptive warning before teaching |
| **Moderate** | Correction when detected in explain-back |
| **Minor** | Gentle note, doesn't block progress |

---

## 🧪 Example Workflow

```python
from core import TutorEngine
from core.learning_fingerprint import record_learning_event, get_fingerprint_summary
from core.misconception_detector import check_explanation_for_misconceptions

# 1. Get personalized teaching (question + level)
engine = TutorEngine()
response = engine.teach(
    "How does soft robotics actuation work?",
    level="student"
)
print(response.to_markdown())

# 2. Check explain-back for misconceptions
student_explanation = "Soft robots can't grip hard things..."
check = check_explanation_for_misconceptions("soft_robotics", student_explanation)

if check["has_misconceptions"]:
    for correction in check["corrections"]:
        print(correction)

# 3. Record the learning event
record_learning_event(
    topic="soft_robotics",
    level="student",
    event_type="explain_back",
    content=student_explanation[:500],
    outcome="confused" if check["has_misconceptions"] else "understood"
)

# 4. Check learning profile
summary = get_fingerprint_summary()
print(f"Best topics: {summary['best_topics']}")
print(f"You learn best with: {summary['learning_style']}")
```

---

## 🎯 Use Cases

1. **Self-study**: Learn any topic with personalized explanations
2. **Lab onboarding**: New students learn lab domain quickly
3. **Concept review**: Identify and fix misconceptions
4. **Spaced repetition**: Retain knowledge long-term
5. **Teaching prep**: Practice explaining concepts

---

## 📋 Requirements

```
streamlit>=1.28.0
openai>=1.0.0  # Or local LLM
```

**LLM Options:**
- OpenAI API (set `OPENAI_API_KEY`)
- Local LLM via LM Studio (port 1234)
- Anthropic API (set `ANTHROPIC_API_KEY`)
- Gemini (set `GEMINI_API_KEY` or `GOOGLE_API_KEY`)

---

## 📄 License

Research use permitted. Contact for commercial licensing.

---

**Built with OMEGA Research Platform**

*"Learn anything. At your level. With your style."*
