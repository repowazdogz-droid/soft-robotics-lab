# OMEGA Tutor v2

Cognitive infrastructure for deep learning. Not a chatbot — structured knowledge presentation, calm, precise.

## Tech Stack

- React 18 + TypeScript
- Vite
- Tailwind CSS (muted palette)
- React Router
- Gemini API (same pattern as Spine Case: `/api/generate` proxy when deployed)

## Quick Start

```bash
cd products/omega_tutor_v2
npm install
npm run dev
```

Open http://localhost:5173

## Information Architecture

| Path | Purpose |
|------|---------|
| `/` | Landing — choose Workspace, Dashboard, or Terrain |
| `/workspace` | Learning Workspace (main learning environment) |
| `/dashboard` | Cognitive Dashboard (knowledge instrumentation) |
| `/terrain` | Knowledge Terrain (curriculum graph) |
| `/settings` | Configuration |

## Phase 1 (Done)

- Vite + React + TypeScript project
- Tailwind with muted palette (CSS variables + theme)
- AppShell, EnvironmentNav, Sidebar (layout)
- Routing: Landing, Workspace, Dashboard, Terrain, Settings
- Gemini service (proxy or direct with `VITE_GEMINI_API_KEY`)
- Types: learning, dashboard, terrain, api
- Prompts: tutorSystem, levelPrompts, explainBack, misconceptionDetection
- Data: curricula (soft robotics, ML, syn bio), misconceptions (soft robotics)
- Utils: sm2, formatting
- Hooks: useLearning, useSpacedRepetition, useMisconceptions (stubs)
- `api/generate.ts` for Vercel (set `GEMINI_API_KEY` in project env)

## Next Phases

- **Phase 2:** Learning Workspace — QuestionInput, TopicHeader, DepthDial, ExplanationBlock, ExplainBack flow
- **Phase 3:** Explain-back & misconceptions — full integration
- **Phase 4:** Dashboard — MasteryGrid, RetentionChart, ReviewQueue, SM-2
- **Phase 5:** Knowledge Terrain — graph, curriculum paths, progress overlay
- **Phase 6:** Polish — responsive, settings, deploy

## Deploy (Vercel)

1. Add `GEMINI_API_KEY` in Vercel project → Environment Variables
2. `npm run build && vercel --prod`
3. API key stays server-side via `api/generate.ts`
