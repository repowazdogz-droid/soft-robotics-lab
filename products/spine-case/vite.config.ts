import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Gemini API key is server-side only (api/generate.ts). Set GEMINI_API_KEY in Vercel env or .env for vercel dev.
export default defineConfig({
  plugins: [react()],
});
