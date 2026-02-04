# What Is OMEGA Tutor?

**Product:** OMEGA Tutor (`products/omega_tutor`)  
**Type:** Adaptive teaching app — level-aware explanations, explain-back, misconceptions, spaced repetition, curricula.  
**UI:** Single Streamlit app (chat + sidebar).  
**Backend:** LLM (LM Studio local or Gemini cloud).

---

## 1. What Is OMEGA Tutor?

### Core purpose

- **Teach any topic at the user’s chosen level** (kid → researcher) with one question at a time.
- **Check understanding** via “explain it back” (user explains in their own words → LLM scores and gives feedback).
- **Catch misconceptions** (built-in DB + preemptive warnings + detection in explain-back).
- **Reinforce and retain** via quizzes and SM-2 spaced repetition (topics due for review).

So: **personalized teaching + verification + correction + retention**, not just Q&A.

### Problem it solves

| Problem | What OMEGA Tutor does |
|--------|------------------------|
| One-size-fits-all explanations | 4 personas → 4 internal levels (kid, undergrad, expert, researcher); Simpler/Deeper per message. |
| No check that the learner understood | Explain-back: user types explanation → score 0–100, correct/missing/misconceptions, encouraging feedback. |
| Misconceptions persist | Misconception DB + preemptive warnings before teaching + detection in explain-back. |
| Progress and retention forgotten | Progress DB (topics, questions, streak) + SM-2 scheduling + “Due for review” + quizzes. |
| Unstructured “what to learn next” | Curricula (paths) with prerequisites and per-topic progress. |

### Who it’s for

- **Self-study:** “Explain X at my level” and verify with explain-back.
- **Students (school/uni):** Clear explanations, quizzes, and review scheduling.
- **Professionals:** “Give me what I need to know” (expert mode), practical focus.
- **Researchers:** Full depth, nuance, open problems (researcher mode).
- **Kids (with adult setup):** Simple language, analogies, playful tone (kid mode).

So: **any learner who wants level-appropriate teaching plus verification and retention**, not just a generic chatbot.

---

## 2. Personas / Modes

### List of personas (what the user picks)

Defined in `core/user_profile.py` — **PERSONA_CONFIG**:

| Persona (ID) | Label | Tagline | Quote | Internal level |
|--------------|--------|---------|--------|-----------------|
| **kid** | Kid Mode | Fun explanations with examples | "Explain like I'm 10" | `kid` |
| **student** | Student Mode | Clear explanations for school/uni | "Help me understand and remember" | `undergrad` |
| **professional** | Professional Mode | Efficient, practical, no fluff | "Give me what I need to know" | `expert` |
| **researcher** | Researcher Mode | Full depth, cutting edge, OMEGA-MAX | "Show me everything" | `researcher` |

The **internal level** is what the teaching engine uses (vocabulary, depth, structure). The user only sees the persona label and tagline.

### What each persona does differently

- **Kid:** Short, simple words; analogies (toys, animals, nature); one idea at a time; encouraging, playful. (See `level_prompts.py` — "little" / "kid".)
- **Student (undergrad):** Clear structure; key concepts; thorough but efficient; “how to use this”; edge cases when relevant.
- **Professional (expert):** Assumes domain knowledge; nuance, edge cases, common pitfalls; papers/frameworks/standards when useful; current debates and limits.
- **Researcher:** OMEGA-MAX depth; substrates (materials, compute, bio, manufacturing, coordination, environment); T1–T4 temporal horizons; open problems and uncertainties; multiple frameworks/theories; citations.

So: **same question, different level of language, depth, and framing.**

### Researcher vs undergraduate

- **Undergrad:** “Assume basics; be thorough and practical; define terms if needed; focus on intuition and application.”
- **Researcher:** “Assume active researcher; full technical depth; substrates and temporal horizons; open problems; assumptions and uncertainties; cite when useful.”

So researcher mode is **deeper, more technical, and more research-oriented**; undergrad is **structured learning with application focus**.

### What actually changes between them

- **Depth:** kid &lt; student &lt; expert &lt; researcher.
- **Style:** playful → clear → practical → technical/research.
- **Sources:** kid/student rarely cite; expert/researcher can reference papers, standards, debates.
- **Features:** Same app features (explain-back, quiz, curriculum); no extra “researcher-only” features — the difference is **how the LLM is prompted** (via level).

The **active** `TutorEngine` (`core/tutor_engine.py`) uses **short inline level prompts** (one line per level). The **full** level descriptions live in `core/level_prompts.py` (and are used by `tutor_engine_backup.py` if you switch to that). So the product *design* is full level adaptation; the *shipped* engine uses a simpler prompt set.

---

## 3. Teaching Methods

### How it explains things

- User asks a question (or picks a topic).
- App calls `TutorEngine.teach(question, level=level)`.
- Level comes from the user’s persona (kid / undergrad / expert / researcher).
- The LLM gets a **system prompt** that encodes “OMEGA Tutor” + level instruction (e.g. “Explain simply for a child…” or “Full depth, cutting edge…”).
- Response is shown as markdown (explanation; optional example, exercise, follow-ups).

So: **one question → one explanation**, adapted to level. There is also a **Simpler / Deeper** control: same question re-asked at one step simpler or deeper on a fixed ladder (little → kid → teen → sixth_form → undergrad → adult → senior → expert → researcher).

### Does it adapt to the user?

- **At choice time:** User picks persona once (or switches in sidebar); that sets the default level.
- **Per message:** User can click “Simpler” or “Deeper” to re-explain at an adjacent level; the **profile level is unchanged**.
- **From explain-back:** The app records explain-back score and can update topic confidence; the **next* teaching* can in principle use that (e.g. via learning fingerprint or progress); the main visible adaptation is still the chosen persona and Simpler/Deeper.

So: **adaptation is level-based (persona + Simpler/Deeper)** plus **implicit** use of progress/fingerprint where wired in.

### Explain-back feature

- **Flow:** After the tutor explains, user clicks “Explain it back” → types their understanding in a text box → Submit.
- **Backend:** `core/explain_back.py` — `evaluate_explanation(topic, user_explanation, level, api_key)`.
- **LLM:** Asked to evaluate “gently”; focus on what’s right first; return JSON: `score` (0–100), `correct_points`, `missing_points`, `misconceptions`, `feedback`.
- **UI:** Score, “You got right: …”, “Also important: …”, “Watch out: …”, short feedback. Optionally updates topic confidence and learning fingerprint (e.g. “confused” vs “understood”).

So: **teaching → user explains → LLM scores and gives feedback** (not just “correct/incorrect”).

### Misconception detection

- **Stored misconceptions:** `core/misconception_detector.py` — **MISCONCEPTIONS** dict by topic (e.g. soft_robotics, materials_science, machine_learning, physics, biology). Each entry: wrong belief, correct understanding, trigger phrases, severity (minor/moderate/critical), correction approach.
- **In explain-back:** User’s explanation is checked against triggers; if a known misconception is detected, corrections from the DB are shown and the event can be recorded as “confused.”
- **Preemptive:** Before teaching a topic, `generate_preemptive_warning(topic)` can show a short warning for **critical** misconceptions so the tutor addresses them in the explanation.

So: **DB of known misconceptions + detection in user text + optional pre-teaching warning.**

### Spaced repetition (how it works)

- **Algorithm:** SM-2 in `core/spaced_repetition.py` — quality 0–5, easiness factor, intervals (1 day, 6 days, then previous_interval × easiness).
- **When it runs:** After a quiz answer, `progress_tracker.schedule_review(topic, quality)` is called (e.g. quality 5 correct, 2 wrong). That updates `easiness`, `sm2_interval`, `next_review`, `repetitions` in the progress DB.
- **Due reviews:** `progress_tracker.get_due_reviews()` returns topics where `next_review <= today` (or not set). Shown in sidebar as “Due for review” and in quiz source “Topics due for review.”
- **Decay:** `core/knowledge_decay.py` — confidence decays over time (e.g. half-life ~7 days); used to **prioritize** which topics to review first (e.g. low decayed confidence = higher priority).

So: **quiz/review → SM-2 update → due list → user can quiz on due topics or “Review due topics” in the UI.**

---

## 4. Learning Flow (full session)

1. **Open app** → If no profile: **Welcome** → choose one of 4 personas → profile saved.
2. **Topic picker** (no messages yet): “What do you want to learn about?” — 3 curriculum quick-picks (e.g. Soft Robotics, Machine Learning, Synthetic Biology) or “Or type anything” + text input + Go.
3. **First exchange:** User question (from pick or typed) → `TutorEngine.teach(question, level)` → explanation shown. Optional: preemptive misconception warning for that topic.
4. **Chat:** User keeps asking in the main input; each reply is level-adapted. Under each assistant message: **Simpler / Deeper** (re-explain at adjacent level), **Explain it back** (type explanation → score + feedback).
5. **Explain-back:** User clicks “Explain it back” → types → Submit → LLM evaluation (score, correct/missing/misconceptions, feedback); optional misconception correction; progress/fingerprint updated.
6. **Quiz (optional):** Sidebar “Test Yourself” → choose source (learned / curriculum / due / everything) and number of questions → Start Quiz → multiple-choice questions → feedback per question → “Learn more” on wrong → at the end: score, “Quiz again” / “Back to learning.” Quiz results feed **schedule_review** (SM-2).
7. **Curriculum (optional):** Sidebar “Learning Paths” → select path → main area shows path progress; “Learn: X” / “Quiz this topic” / “Review notes” per topic; prerequisites unlock in order.
8. **Due for review (optional):** Sidebar “Review due topics” (or “Due for review”) → list of topics due (SM-2) → “Quiz me” / “Re-learn” per topic.
9. **Exit (optional):** “That’s enough for now” → session summary (topics, time, suggested next, encouragement) → “Start Fresh” / “Continue later.”

So: **persona → topic or question → teach → (optional) explain-back / Simpler–Deeper / quiz / path / due review → exit.** The “core” experience is: **ask → get level-adapted explanation → optionally explain back and/or quiz.**

---

## 5. Curriculum / Paths

### What learning paths exist

- **Stored as JSON** in `data/curricula/`:
  - **soft-robotics-101** — “Soft Robotics Fundamentals” (e.g. materials, silicone, tendons, actuators, pneumatic, sensors, control, gripper, manipulation, fabrication).
  - **machine-learning-101** — Machine learning fundamentals (see `machine-learning-101.json`).
  - **synthetic-biology-101** — Synthetic biology fundamentals.
  - **custom** — User-defined path from comma-separated topics (generated by curriculum engine, saved as custom.json).

### How they work

- **CurriculumEngine** (`core/curriculum.py`): Loads JSON by id; lists curricula (id, name, description, topic_count); **get_progress(curriculum_id)** returns:
  - **completed** — topics with completed flag (from curriculum_progress).
  - **available** — topics whose prerequisites are all completed.
  - **locked** — topics with unmet prerequisites.
  - **percent_complete** — completed / total.
- **Prerequisites:** Each topic can list `prerequisites: [topic_id, ...]`. User must “complete” (learn or mark done) prereqs before that topic is **available**.
- **Progress:** `core/curriculum_progress.py` — which topic IDs are completed per curriculum; **mark_complete** when user learns or finishes a topic.
- **In UI:** User selects a path in sidebar → main area shows path name, progress bar, then lists: completed (with “Quiz this topic” / “Review notes”), available (“Learn: X”), locked (“needs: …”).

So: **structured paths with prerequisites and completion tracking**, not just a flat list of topics.

### Topics covered (example: Soft Robotics 101)

From `data/curricula/soft-robotics-101.json`: Materials Basics → Silicone Properties, Tendon Mechanics, Sensors, Actuator Types → Pneumatic Actuators, Control Basics, … → Gripper Design → Soft Manipulation, Fabrication Methods. Other curricula define their own topic graphs.

---

## 6. AI / Backend

### What the app uses today

- **Code path:** `app.py` imports `TutorEngine` from `core`; `core/__init__.py` exports `core/tutor_engine.py` (not `tutor_engine_backup.py`).
- **tutor_engine.py** (active):
  1. **LM Studio (local):** `OpenAI(base_url="http://localhost:1234/v1", api_key="not-needed")`, model `phi-3-mini-4k-instruct`. Tried first on every `teach()`.
  2. **Gemini (cloud):** If LM Studio fails, uses `GEMINI_API_KEY` or `GOOGLE_API_KEY`; model `gemini-pro` via `google.generativeai.GenerativeModel("gemini-pro")`.

So: **LM Studio first, then Gemini.** No OpenAI or Anthropic in the **active** engine.

### README vs code

- README lists: OpenAI, LM Studio, Anthropic, Gemini.
- **Shipped engine** only uses: LM Studio + Gemini. API key can be set in env or pasted in sidebar.

### Model choice

- **Fixed** in code: LM Studio uses `phi-3-mini-4k-instruct`; Gemini uses `gemini-pro`. No model picker in the UI; no Ollama in the current tutor_engine.py.

Other modules (e.g. explain_back, quiz_generator) may use their own LLM calls (e.g. Gemini 2.0 Flash in explain_back) for evaluation/generation.

---

## 7. What Makes It Special vs ChatGPT

- **Level as a product feature:** Persona (kid/student/professional/researcher) is first-class; every answer is explicitly level-adapted and user can step “Simpler” or “Deeper.” ChatGPT doesn’t ship a dedicated “kid vs researcher” teaching mode.
- **Explain-back as a loop:** “Explain it back” + LLM evaluation (score, correct/missing/misconceptions, feedback) is a deliberate **understanding check**, not just another message. Closer to a tutor than to open-ended chat.
- **Misconception system:** Stored misconception DB + detection in user text + preemptive warnings. Focus on **correcting known errors**, not only answering questions.
- **Spaced repetition:** SM-2 scheduling and “Due for review” + quizzes that feed it. **Retention** is part of the design, not an add-on.
- **Curriculum with prerequisites:** Paths and “Learn / Quiz / Review” per topic with a clear notion of **progress** and **order** (prereqs).
- **Learning fingerprint (optional):** Tracks what works for the user (e.g. analogies, examples); intended for future personalization.
- **Pedagogical framing:** System prompt (in `prompts/tutor_system.py`) stresses: teach don’t lecture, adapt to level, encourage follow-ups, preserve complexity, multiple paths, make it stick (analogies, examples). So the **design** is tutor-first, not generic assistant.

“Secret sauce” in one line: **level-adapted teaching + explain-back + misconception handling + SM-2 and curricula**, all in one product.

---

## 8. Core Teaching Prompt / System

### Full system prompt (intended design)

In **prompts/tutor_system.py** — **TUTOR_SYSTEM_PROMPT**:

- **Role:** “OMEGA Tutor, a learning companion built on OMEGA-MAX principles. Help humans understand anything, at any level, with zero friction.”
- **Behaviors:**
  1. **TEACH, DON'T LECTURE** — Engage with the question; Explain → Example → Exercise; conversational, not textbook.
  2. **ADAPT TO LEVEL** — Child: wonder/simplicity; Teen: relevance/encouragement; Adult: clarity/practicality; Expert: depth/nuance; Researcher: full OMEGA-MAX substrate/temporal analysis.
  3. **ENCOURAGE FOLLOW-UPS** — End with “You might also wonder…” or suggested next questions; never make the learner feel dumb; “Great question” is banned — just answer well.
  4. **PRESERVE COMPLEXITY** — Don’t oversimplify to wrongness; for lower levels, simplify language not concepts; flag “actually more complicated but this is the key idea.”
  5. **MULTIPLE PATHS** — Offer 2–3 ways to think about complex topics; “Some people find it helpful to think of it as…”
  6. **MAKE IT STICK** — Memorable analogies; connect to what they know; the example is as important as the explanation.
- **Output format:**  
  ## Explanation  
  ## Example  
  ## Try This (optional)  
  ## Go Deeper (optional)

This is the **canonical** tutor behavior. It is used by **tutor_engine_backup.py** (with `get_level_prompt(level)` from `level_prompts.py`). So: **full_system = TUTOR_SYSTEM_PROMPT + “Current level instruction: ” + get_level_prompt(level)**.

### What the active engine uses

**core/tutor_engine.py** (what the app actually runs):

- Does **not** use `TUTOR_SYSTEM_PROMPT` or `level_prompts.py`.
- Uses a **short inline** level prompt per level, e.g.:
  - kid: “Explain simply for a child. Use fun examples and comparisons.”
  - undergrad: “Explain for a university student. Be thorough.”
  - expert: “Give technical detail. Assume domain knowledge.”
  - researcher: “Full depth, cutting edge. OMEGA-MAX level.”
- System message: `"You are OMEGA Tutor, an expert teacher. {system}\n\nUser question: {question}"`.
- So the **product’s** intended pedagogy is in `prompts/tutor_system.py` and `level_prompts.py`; the **shipped** engine uses a reduced version for reliability/simplicity.

### Level instructions (full set)

In **core/level_prompts.py** — **LEVEL_PROMPTS** (used by backup engine and level_adapter):

- **little:** 5–7 yo; very simple words; sensory/playful; “Imagine…”, “What if…”; analogies to toys, animals, nature, food.
- **kid:** 8–11; simple words; one new term at a time; connect to games, school, hobbies; encouraging, playful.
- **teen:** 12–15; technical terms defined; connect to apps, social media, real-world impact; direct, respectful.
- **sixth_form:** 16–18; exam-aware; full vocabulary; key points then detail; “why this matters,” “what comes next.”
- **undergrad:** 19–25; full vocabulary; concise, practical; intuition and application; edge cases; respect time.
- **adult:** 26–64; clear, direct; key idea → example → takeaway; “how this applies”; 2–3 angles when relevant.
- **senior:** 65+; calm pace; define terms; everyday analogies; one idea at a time; patient.
- **expert:** Professional; assume foundation; nuance, edge cases, misconceptions; papers/frameworks/standards; debates and limits.
- **researcher:** OMEGA-MAX; full technical depth; substrates; T1–T4 where relevant; open problems; uncertainties; multiple frameworks; cite when useful.

So: **the full teaching prompt is “TUTOR_SYSTEM_PROMPT + level instruction from LEVEL_PROMPTS”; the active engine uses a one-line version of the level instruction.**

---

## Summary Table

| Aspect | What it is |
|--------|------------|
| **Product** | Adaptive teaching app: level-based explanations, explain-back, misconceptions, SM-2, curricula. |
| **Personas** | Kid, Student, Professional, Researcher → levels kid, undergrad, expert, researcher. |
| **Teaching** | One question → one level-adapted explanation (LM Studio or Gemini); Simpler/Deeper per message. |
| **Explain-back** | User explains in own words → LLM score 0–100 + correct/missing/misconceptions + feedback. |
| **Misconceptions** | DB per topic + detection in explain-back + preemptive warnings for critical ones. |
| **Spaced repetition** | SM-2 after quiz/review; due list in sidebar; knowledge decay for priority. |
| **Curriculum** | JSON paths (e.g. soft-robotics-101, machine-learning-101) with prerequisites and completion. |
| **Backend (active)** | LM Studio (localhost:1234, phi-3-mini-4k-instruct) then Gemini (gemini-pro). |
| **Core prompt (design)** | prompts/tutor_system.py + level_prompts.py (used in tutor_engine_backup). |
| **Core prompt (shipped)** | Short “You are OMEGA Tutor…” + one-line level in tutor_engine.py. |

This document is the full picture of **what** OMEGA Tutor is and does, from the code and docs.
