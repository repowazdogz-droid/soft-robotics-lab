/**
 * OMEGA Tutor v2 — Cognitive infrastructure for deep learning.
 * Not a chatbot. Structured knowledge, calm, precise.
 */

import { Routes, Route, Navigate } from "react-router-dom";
import { AppShell } from "./components/layout/AppShell";
import { LandingPage } from "./pages/LandingPage";
import { WorkspacePage } from "./pages/WorkspacePage";
import { DashboardPage } from "./pages/DashboardPage";
import { TerrainPage } from "./pages/TerrainPage";
import { SettingsPage } from "./pages/SettingsPage";

export default function App() {
  return (
    <Routes>
      <Route path="/" element={<LandingPage />} />
      <Route element={<AppShell />}>
        <Route path="workspace" element={<WorkspacePage />} />
        <Route path="dashboard" element={<DashboardPage />} />
        <Route path="terrain" element={<TerrainPage />} />
        <Route path="settings" element={<SettingsPage />} />
      </Route>
      <Route path="*" element={<Navigate to="/" replace />} />
    </Routes>
  );
}
