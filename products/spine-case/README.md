# Spine Case Synthesis

Pre-operative cognitive support tool for spine surgeons.

## What This Is

A thinking aid that helps clinicians:
- Synthesise complex case information
- Surface considerations and blind spots
- Prepare for MDTs and patient discussions
- Reflect before committing to irreversible decisions

## What This Is NOT

- Not a diagnostic system
- Not a treatment recommender
- Not a decision-maker
- Not a replacement for clinical judgment

## Running Locally

1. **API key (server-side only):** The Gemini key is never sent to the browser. Set `GEMINI_API_KEY` in your environment:
   - **With Vercel (recommended):** Run `vercel dev`, then set `GEMINI_API_KEY` in `.env` or in Vercel project → Settings → Environment Variables. The `/api/generate` serverless function will use it.
   - **Production (Vercel):** Add `GEMINI_API_KEY` (no `VITE_` prefix) in the Vercel project’s Environment Variables. Never use `VITE_GEMINI_API_KEY` in the client.
2. Install dependencies: `npm install`
3. Start development: `npm run dev` (Vite only; no API proxy) or `vercel dev` (Vite + API routes; use this to test synthesis).
4. Open http://localhost:5173 (or the URL shown by `vercel dev`)

## Safety

This tool is designed with safety as the primary constraint:
- No diagnostic language
- No treatment recommendations
- No numeric probabilities
- Humble, hedged language throughout
- Clear limitations stated on every output

## For Clinicians

This tool supports your thinking. It does not replace it.
All clinical decisions require your professional judgment and direct patient assessment.
