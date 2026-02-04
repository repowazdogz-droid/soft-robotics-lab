# OMEGA Tutor — Full UX/UI Audit

**Date:** 2025-01-31  
**Scope:** `products/omega_tutor` — Streamlit chat interface for level-adaptive learning.

---

## 1. App Structure

### Pages / views (modes, not separate URLs)

The app is a **single Streamlit script** with **mode switches** via `st.session_state` and `st.stop()` / `st.rerun()`. There are no separate routes; “views” are mutually exclusive blocks.

| View | Trigger | What user sees |
|------|---------|----------------|
| **Welcome (personas)** | First run: `not has_profile()` | “Pick who you are” — 4 persona cards (Kid, Student, Professional, Researcher). |
| **Topic picker** | Profile exists, `len(messages) == 0`, no `pending_starter_question` | “What do you want to learn about?” — 3 curriculum quick-picks + “Or type anything” text input + Go. |
| **Exit summary** | `show_exit_summary` + `exit_summary` | “Session Complete” — topics learned, time, suggested next, Start Fresh / Continue later. |
| **Main chat** | Profile + messages exist (or pending question just processed) | Title + caption, sidebar, chat history, chat input. |
| **Quiz setup** | `quiz_mode` True, no questions yet | “Quiz Mode” — Source (learned / curriculum / due / everything), How many questions?, Start Quiz. |
| **Quiz in progress** | `quiz_mode` + `quiz_questions` | One question at a time, radio options, Submit → feedback → Continue / Learn more. |
| **Quiz complete** | All answers in `quiz_answers` | Score, per-question correct/incorrect, Learn more / Quiz again / Back to learning. |
| **Quiz summary (paused)** | `quiz_state` + `show_quiz_summary` | Same as complete; “Return to quiz summary” flow. |
| **Curriculum panel** | `curriculum_engine` + `selected_curriculum_id` | Learning path name, progress bar, completed (Quiz/Review), available (Learn), locked (prereqs). |
| **Due for review** | `show_review` | Sidebar: list of topics due, “Quiz me” / “Re-learn” per topic, Close. |
| **?topic= query** | URL `?topic=...` | Info box + text input “Ask about this topic” + “Ask Tutor about this”. |

### Navigation flow (high level)

1. **First time:** Open app → Welcome (personas) → pick one → **Topic picker** → pick topic or type question → **Chat** (first exchange then chat).
2. **Returning:** Open app → **Chat** (if messages exist) or **Topic picker** (if messages empty and no pending question).
3. **From chat:** Sidebar → “Test Yourself” → **Quiz setup** → **Quiz** → **Quiz complete** or back to chat. Or “Review due topics” → **Due for review** (sidebar). Or “That’s enough” → **Exit summary**.
4. **From curriculum:** Sidebar “Learning Paths” → select path → **Curriculum panel** in main area; from there “Learn” / “Quiz” / “Review notes” → chat or quiz.

### How a user goes from start to learning

- **Minimum path:** Open app → click one persona → (topic picker) click one topic **or** type question + Go → first Q&A appears → then chat input for follow-up. **3–4 clicks** to first learning response (persona + topic or question + Go; topic picker may show 0 clicks if a default runs).
- **Friction:** Persona is mandatory first; topic picker appears every time until the first message exists (no “just type in the box” on first load).

---

## 2. UI Components

### Sidebar (top to bottom)

- **Persona:** “{icon} {label}” (e.g. Student Mode); “Switch” toggles 4 persona buttons.
- **Clear Chat** button; optional “Chat cleared” success.
- **Curriculum** (if selected): Path name, progress bar, “X% complete”.
- **“Ask anything below.”**
- **Divider.**
- **API key:** “Using API key from environment” or password text input + “No key set…” caption.
- **Backend caption:** “Cloud (Gemini)” / “Local (LM Studio)” / “No backend…”
- **Divider.**
- **Voice Mode:** “Voice input”, “Voice output” checkboxes; Voice selectbox (if TTS); “Upload audio” file uploader; captions like “Voice output: pip install edge-tts”.
- **Your Learning** (if `progress_tracker`): Topics learned, Ready to quiz, Due for review metrics; “Review due topics”, “See all topics”; optional “This Week” (questions, streak).
- **Learning Profile** (if `get_fingerprint_summary`): “View My Profile” toggle; when on: topics studied, events, strong/struggling, “You learn best with: …”.
- **“Test Yourself”** button (quiz).
- **Due for review** (if `show_review`): list of topics, “Quiz me” / “Re-learn” per topic, “Close”.
- **Learning Paths:** “Path” selectbox (curricula + custom), “+ Create Custom Path”, custom topics input + “Generate Path”.
- **“That’s enough for now”** → exit summary.
- **“New Topic”** and **“Go Deeper”** (near end of script, sidebar).

So: **one long sidebar** with persona, API key, voice, progress, fingerprint, quiz, review, curriculum, exit. No sub-pages; everything is on one scroll.

### Main area (by mode)

- **Topic picker:** Subheader, 3 topic buttons, “Or type anything” + text input + Go.
- **Chat:** Chat messages (user/assistant bubbles); under each assistant message: “Simpler” / “Deeper” / “Explain it back”, optional TTS audio, “Explaining at: X level”. Then `st.chat_input("What would you like to learn about?")`.
- **Quiz:** Subheader “Quiz Time!”, question text, radio options, Submit; then feedback (correct/incorrect, explanation, Continue / Learn more).
- **Curriculum panel:** Path title, progress bar, list of completed (Quiz / Review notes), available (Learn), locked (prereqs); optional “Learning: X” + “Ask about X”.
- **Exit summary:** Text list of topics, time, suggested next, encouragement, Start Fresh / Continue later.
- **?topic=:** Info + text input + “Ask Tutor about this”.

### Clicks for core task (“learn one thing”)

- **Ideal path:** 1 (persona) + 1 (topic or type + Go) = **2** to first answer; then 1 type + Enter for each follow-up. So **2 clicks to first learning**, then chat.
- **With curriculum:** 1 persona + 1 Path select + 1 “Learn: X” = **3** before first teach.
- **Quiz:** 1 “Test Yourself” + 1 Source + 1 “Start Quiz” = **3** to first question; then 1 per answer (Submit / Continue).

---

## 3. User Flow: “User opens app → … → User learns something”

### All steps (detailed)

1. User opens app (e.g. `streamlit run app.py`).
2. **If no profile:** Welcome screen → user clicks one of 4 personas → profile saved, rerun → go to 3.
3. **If profile:** App loads sidebar (persona, API key, voice, progress, quiz, curriculum, exit) and checks:
   - If **exit summary** active → show “Session Complete” and buttons → stop.
   - Else show **title + caption** in main area.
4. **If messages empty and no pending_starter_question:** Topic picker in main area (“What do you want to learn about?”) + 3 topic buttons + custom input + Go → **st.stop()** (rest of script not run). User must pick topic or type + Go.
5. **If pending_starter_question:** App appends user message, calls `_engine.teach(...)`, appends assistant message, rerun → now messages non-empty → continue.
6. **If messages non-empty:** Script continues: Voice section, progress session, Your Learning, Learning Profile, Quiz button, Due for review, Learning Paths, exit button.
7. **If quiz_mode:** Quiz block runs (setup or questions or results) → **st.stop()** so chat doesn’t show.
8. **If curriculum selected and main area:** Curriculum panel can show (path progress, Learn / Quiz / Review).
9. **Re-explain (Simpler/Deeper):** If `re_explain` in session, one more assistant message is appended, rerun.
10. **?topic=:** If present, show topic prompt + “Ask Tutor about this” and append messages on button click.
11. **Main chat:** Render all messages (with Simpler/Deeper/Explain it back); then `st.chat_input`. On submit: append user message, call `engine.teach`, append assistant message, optional TTS, progress/recording, rerun.

So: **Open → (persona) → (topic picker or pending question) → chat.** Quiz and curriculum are alternate “modes” that replace or sit above the chat main area.

### Friction points in the flow

- **Persona gate:** First-time and “no profile” users cannot skip; must pick persona before anything else.
- **Topic picker gate:** When `len(messages) == 0` and no pending question, **only** the topic picker is shown and **st.stop()** runs. User cannot use the chat input until they pick a topic or type in the picker and click Go. So “ask anything” is not true on first load.
- **API key in sidebar:** New users may not have `.env`; they see “No key set” and must paste a key or configure env. If backend is None, first teach shows “No backend available.”
- **Quiz behind sidebar:** “Test Yourself” is in the sidebar; users who expect a main “Quiz” tab may miss it.
- **Many sidebar sections:** Voice, Progress, Learning Profile, Quiz, Review, Paths, Exit — all in one column; can feel dense.
- **Explain it back:** Extra step (click “Explain it back” → type → Submit); not obvious that it’s for self-testing.
- **Curriculum vs chat:** Selecting a path shows the curriculum panel in main; “Learn: X” sets `curriculum_learn` and adds a hint; actual teaching still happens in chat. Flow is “pick path → pick topic → then chat,” which can feel indirect.

### Clunky or confusing

- **Two “entry” points:** Topic picker (main) vs chat input (only after first message). So “Ask anything” appears only after the first exchange.
- **st.stop() usage:** Topic picker and quiz modes stop the script, so the “main” chat UI (e.g. chat input) is not rendered in those modes. Easy to assume the app is “stuck” on topic picker if user doesn’t click a topic or Go.
- **Backend caption:** “No backend” is easy to miss in the sidebar; failed first teach only then shows “No backend available” in the thread.
- **Keys and state:** Many `st.session_state` keys (quiz_*, show_review, explain_back_*, curriculum_learn, etc.); state is not named in UI, so debugging “why am I in quiz?” or “why is topic picker showing?” is not transparent.
- **Voice:** Optional; if deps missing, captions say “pip install …” which is technical for some users.

---

## 4. Code Structure

### Single file vs components

- **One main file:** `app.py` is **~1,295 lines** and holds all UI and flow (welcome, topic picker, sidebar, chat, quiz, curriculum, exit summary, re-explain, ?topic=).
- **Core logic in `core/`:** TutorEngine, user_profile, level_adjust, progress, quiz_generator, explain_back, curriculum, voice, session_summary, cognitive_load, learning_fingerprint, misconception_detector, etc. So “business logic” is split out; **UI and flow are not** (no separate Streamlit components or pages).

### State management

- **st.session_state** only; no Redux/Zustand/etc. Used for:
  - messages, quiz_questions, quiz_index, quiz_answers, quiz_mode, quiz_state, show_quiz_summary, explain_back_*, re_explain, pending_starter_question, selected_curriculum_id, custom_curriculum, show_review, show_fingerprint, show_exit_summary, exit_summary, progress_session_id, chat_cleared, show_persona_switch, show_all_topics, review_selected_topic, curriculum_learn, api_key, voice_input, voice_output, last_tts_audio, misconception_warned_topics, pending_misconception_warning, etc.
- **~173** references to `st.session_state` in `app.py`. State is flat and key-based; no single “app state” object.
- **Persistence:** Profile in `data/user_profile.json`; progress/curriculum/fingerprint in core modules (files under `data/`). Session state is in-memory (lost on refresh except what’s re-loaded from profile/data).

### How messy is the code?

- **Long script, many branches:** One linear script with many `if` blocks and `st.stop()`/`st.rerun()`. Order of checks determines which “view” shows; refactors must preserve that order.
- **Repeated patterns:** e.g. “pop quiz keys + set quiz_mode + rerun” repeated in several places; similar for curriculum_learn and pending_starter_question.
- **Optional imports:** Many `try/except ImportError` for core modules; UI adapts (e.g. no quiz button if `generate_quiz` is None) but the same app.py serves “full” and “minimal” installs, which increases branches.
- **No UI components:** No `components/` or reusable `def render_sidebar()`; sidebar is inline. Hard to test or redesign one “screen” in isolation.
- **Good:** Core is separated; persona and level are clear; docstrings and helpers like `_clear_chat()` exist.

---

## 5. Current Features (what it does)

| Feature | Where | Notes |
|--------|--------|------|
| **Explain topics** | Chat; `TutorEngine.teach()` | Level-adaptive (kid / undergrad / expert / researcher). |
| **Quiz** | Sidebar “Test Yourself” → Quiz setup → questions | Source: learned / curriculum / due / everything; 3–20 questions; MC; feedback + “Learn more” on wrong. |
| **Spaced repetition** | `progress_tracker`, knowledge_decay | Due for review list; “Review due topics”; SM-2–style scheduling. |
| **Misconception detection** | Preemptive warning after teach; “Explain it back” + check | Preemptive warning per topic; explain-back checks user text for misconceptions. |
| **Progress tracking** | progress_tracker, curriculum progress | Topics learned, “Ready to quiz,” “Due for review,” streak, this week; curriculum % complete. |
| **Learning fingerprint** | get_fingerprint_summary (optional) | “Learning Profile”: topics studied, strong/struggling, “You learn best with: …”. |
| **Cognitive load** | should_show_quiz, should_show_dashboard, is_overwhelmed | Can hide quiz/dashboard; “Take your time” message when overwhelmed. |
| **Explain it back** | Per-message “Explain it back” → text area → Submit | Evaluates explanation, score, correct/missing/misconceptions; updates confidence. |
| **Simpler / Deeper** | Buttons under assistant messages | Re-teach same question at different level. |
| **Curriculum / paths** | Sidebar “Learning Paths” | Select path; main area shows path progress, Learn / Quiz / Review per topic; custom path from comma-separated topics. |
| **Voice** | Sidebar: input (upload/mic), output (TTS) | Optional; whisper + edge-tts; voice selection. |
| **Session summary / exit** | “That’s enough for now” | Generates summary (topics, time, suggested next); “Start Fresh” / “Continue later”. |
| **?topic=** | Query param | In-app prompt to ask about that topic. |

---

## 6. Screenshots (describe)

### First screen (no profile)

- **Title:** “OMEGA Tutor”
- **Subtitle:** “Pick who you are and start learning:”
- **Horizontal rule**
- **Four large buttons** (full width), each:
  - Icon + “I’m a Kid / Student / Professional / Researcher”
  - Tagline (e.g. “Fun explanations with examples”)
  - Quote (e.g. “Explain like I’10”)
- **Horizontal rule**
- **Sidebar:** Not shown until after a persona is chosen (script stops at welcome).

### First screen (has profile, no messages)

- **Main:** “What do you want to learn about?” → 3 topic buttons (e.g. Soft Robotics, Machine Learning, Synthetic Biology) in columns; below: “Or type anything:” → text input + “Go”.
- **Sidebar:** Persona label, Switch, Clear Chat, (optional) curriculum progress, “Ask anything below,” divider, API key field/caption, backend caption, Voice Mode (checkboxes, voice select, upload), Your Learning (metrics, Review due, See all topics), This Week (if enabled), Learning Profile (if enabled), Test Yourself, (if show_review) Due for review, Learning Paths, “That’s enough for now.”
- **Caption under topic picker:** “Pick a topic above or type your question.”

### Learning screen (chat)

- **Main:** “OMEGA Tutor” title, “Ask anything. Learn at your level.” caption; then chat messages (user left, assistant right or similar); each assistant message has:
  - Content (markdown)
  - Optional TTS audio
  - “Explaining at: **undergrad** level”
  - Buttons: “Simpler” | “Deeper” | “Explain it back”
  - If “Explain it back” active: text area “Explain in your own words” + Submit, then score and feedback
- At bottom: **Chat input** “What would you like to learn about?”
- **Sidebar:** Same as above; at bottom “New Topic” and “Go Deeper”.
- Optional: “Take your time…” (overwhelmed); misconception warning banner.

### Quiz setup

- **Main:** “Quiz Mode”, “What do you want to be quizzed on?”; **Source** = radio (Topics I’ve learned, Curriculum: …, Topics due for review, Everything); **How many questions?** = selectbox (3, 5, 10, 20); **Start Quiz** button.
- **Sidebar:** Same as chat.

### Quiz in progress

- **Main:** “Quiz Time!”, “Question 1 of N”, question text, radio options (A/B/C/D), “Submit Answer”. After submit: correct/incorrect, correct answer, explanation, “Continue to next question” or “Learn more about this”.
- **Sidebar:** Same.

### Quiz complete

- **Main:** “Quiz Complete: _topic_”, “Score: X/Y (Z%)”, then per question: success/warning + question snippet, correct answer, explanation; “Learn more” / “Review all wrong answers” / “Quiz again” / “Back to learning”.
- **Sidebar:** Same.

### Curriculum panel

- **Main:** Path name, progress bar, “X% complete”; list of completed topics with “Quiz this topic” / “Review notes”; available topics with “Learn: X”; locked topics with “needs: …”. If `curriculum_learn` set: “Learning: X. Ask below or click to teach” + “Ask about X”.
- **Sidebar:** Same.

### Exit summary

- **Main:** “Session Complete”, “Today you learned:”, list of topics with confidence labels, time, topic count, “Next time you could explore:”, encouragement, “Start Fresh” / “Continue later”.
- **Sidebar:** Same (or minimal).

---

## 7. Friction Points (list)

1. **Persona required before anything** — Cannot try the app without choosing Kid/Student/Professional/Researcher.
2. **Topic picker blocks chat on first load** — No single “ask in the box” until after first exchange; must use topic buttons or “Or type anything” + Go.
3. **API key prominent in sidebar** — New users see “No key set” and may not know about `.env`; key is in the main sidebar flow.
4. **Backend failure only visible after first teach** — “No backend” in sidebar is easy to miss; error appears in thread only when they ask something.
5. **Quiz is sidebar-only** — “Test Yourself” is one of many sidebar items; no main “Quiz” tab or CTA.
6. **Too many sidebar sections** — Persona, key, voice, progress, fingerprint, quiz, review, paths, exit; feels dense and technical.
7. **Explain it back is hidden** — Under each message; not obvious as “test yourself” or “check understanding.”
8. **Curriculum flow is indirect** — Pick path → pick topic → then chat; “Learn: X” doesn’t auto-send one question.
9. **Two ways to “start” learning** — Topic picker (main) vs chat input (after first message); inconsistent.
10. **State not visible** — User can’t tell why they’re on topic picker vs chat vs quiz; no breadcrumb or mode indicator.
11. **Voice is technical** — “pip install …” in captions; upload-only if no mic.
12. **Re-explain (Simpler/Deeper) adds buttons under every message** — Can clutter long threads.
13. **New Topic / Go Deeper at bottom of sidebar** — Easy to miss; “New Topic” clears chat (destructive) without confirmation.
14. **No onboarding** — No tooltips or first-time hints.
15. **Session summary only after “That’s enough”** — No obvious “end session” or “see my progress” in main flow.

---

## 8. Recommendations: What would make this 10/10?

### Remove or hide by default

- **API key from main sidebar:** Move to Settings/Advanced or a collapsible “Developer”; rely on `.env` for first-run.
- **Voice block when not installed:** Hide or one line “Voice (optional)” that expands; avoid “pip install” in main view.
- **Learning Fingerprint** until user has enough data: Show “Learning Profile” only after e.g. 5+ topics or N events.
- **Duplicate “Ask anything”** (sidebar caption + chat placeholder): One clear CTA in main area.

### Simplify

- **One entry to learning:** Either (A) show chat input immediately with placeholder “What do you want to learn? (or pick a topic below)” and topic chips/buttons below, or (B) keep topic picker but add “Or just type your question above” and a single input at top that both sends and hides the picker. Goal: **one box + one click** to first answer.
- **Persona:** Make it optional or post-first-question: “Answer one question first, then we’ll ask how you like to learn” or default to Student and allow “Switch” later.
- **Sidebar:** Group into 2–3 sections: (1) You (persona, profile, key), (2) Learning (path, progress, quiz, review), (3) Session (voice, exit). Or move “Test Yourself” and “Review due” into the main area as tabs or a secondary CTA.
- **Quiz:** Single “Quiz” entry: “Quiz me on what I’ve learned” (default 5 questions, learned topics) with “More options” for source/count. Fewer choices on first click.
- **Curriculum:** “Pick a path” → show path with one “Start” or “Next topic” that sends the first question into chat so user stays in one place (chat).

### Ideal flow (target)

1. **Open app** → Main: “What do you want to learn?” (one input) + optional topic chips (e.g. Soft Robotics, ML, SynBio). Sidebar: minimal (persona as small label, “Settings” for key/voice).
2. **User types or taps topic** → First answer in chat. Optional: “How was that? (Simpler / Deeper / Good)” to set level without forcing persona up front.
3. **Ongoing:** Chat is the home. Under messages: Simpler / Deeper / “Check my understanding.” Sidebar or top: “Quiz” (one click to 5 questions), “My progress,” “Paths,” “Done for today.”
4. **Quiz:** One button → “5 questions on what you’ve learned” → run; “More options” for source/count. Results in same area or compact summary with “Quiz again” / “Back to chat.”
5. **Paths:** “Learning paths” → choose path → “Next topic” or list; clicking “Learn: X” sends one message into chat and focuses chat.
6. **Exit:** “Done for today” → summary (topics, time, next) → “Start fresh” / “Continue later.” No mandatory persona or topic picker on next visit; resume in chat.

### Code / structure

- **Split app.py into views:** e.g. `views/welcome.py`, `views/chat.py`, `views/quiz.py`, `views/curriculum.py`, `views/exit_summary.py`; one router (e.g. `st.session_state["view"]`) and a single `render_sidebar()`. Easier to test and redesign one view.
- **Single state object:** One `AppState` (or similar) with clear modes (welcome, pick_topic, chat, quiz_setup, quiz_run, quiz_done, curriculum, exit_summary) instead of many booleans and keys.
- **Optional “design system”:** Shared constants for section titles, button labels, so copy is consistent and easy to change.

---

## Summary Table

| Dimension | Current | Target (10/10) |
|-----------|----------|-----------------|
| **Clicks to first learning** | 2–4 (persona + topic or type + Go) | 1–2 (type or tap topic) |
| **Entry to learning** | Topic picker only when messages empty | One input always visible; topic picker optional |
| **Sidebar** | Long; key, voice, progress, quiz, paths, exit | Short; You + Learning + Session or moved into main |
| **Quiz** | Sidebar “Test Yourself” → setup (source, N) | One “Quiz” CTA → default 5 on learned; options behind “More” |
| **Persona** | Required first screen | Optional or after first question |
| **State / mode** | Implicit (many session keys) | Explicit view/mode + minimal keys |
| **Code** | ~1.3k lines in one file | Views + shared sidebar + clear state |

This document is the full picture for redesign: reduce steps to first learning, make chat the home, simplify sidebar and quiz, and optionally refactor into view-based structure and a single app state.
