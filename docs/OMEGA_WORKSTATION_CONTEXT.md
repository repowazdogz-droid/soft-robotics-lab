# 🧱 OMEGA WORKSTATION CONTEXT

You are running inside OMEGA-MAX ΦΩ η-CIV — Sovereign Deterministic Civilization Kernel, on a multi-device workstation owned by a single user.

Treat the following as hard constraints about the environment.

---

## 1. Hardware Layout

### A. Sovereign PC (Windows Tower — Main Compute Host)

**Role:** Compute-Sovereign + Simulation-Sovereign + Render-Sovereign

| Component | Spec |
|-----------|------|
| CPU | AMD Ryzen 9 9950X |
| GPU | NVIDIA RTX 5090 AORUS MASTER, 32GB VRAM |
| RAM | 64GB DDR5 |
| Storage | 2TB + 4TB NVMe SSD |
| PSU | 1200W |
| OS | Windows 11 Pro |
| Python | Anaconda (compute env) |

**Capabilities:**
- Local LLMs (8–70B-class with quantization)
- Diffusion / video generation
- Physics engines (MuJoCo; Isaac Sim if installed)
- Game engines (Unity/Unreal)
- CUDA-heavy training and inference
- Multi-agent orchestration servers
- XR streaming servers (for Quest / Vision Pro)

**This is the primary host for any heavy workload.**

---

### B. Mac Mini (M4 Pro, 48GB RAM)

**Role:** Desk Mind / Orchestrator / Docs & Design Node

**Use for:**
- Day-to-day Claude/GPT/Gemini interaction
- Agents that plan, refactor, or supervise code
- Documentation, UX, pitch decks, writing
- Light Metal inference and tool GUIs

**No CUDA. No heavyweight physics.**

---

### C. MacBook Pro (M4 Max, 48GB RAM, 1TB)

**Role:** Mobile Cognitive Workstation

**Use for:**
- Mobile coding (Cursor etc.)
- Travel work, demos, slide builds
- Running thinner versions of tools
- Connecting remotely to the Sovereign PC when needed

**Strong CPU/GPU for general work, but no CUDA.**

---

### D. XR + Embodiment Layer

| Device | Role |
|--------|------|
| Apple Vision Pro | Spatial computing, AR interfaces |
| Meta Quest 3 | VR training, simulation |
| Steam Deck | Portable gaming/testing |
| Mocap suit | Motion capture input |
| Haptic vest | Force feedback |
| Vive trackers + SteamVR base stations | Full body tracking |

**Capabilities:**
- Spatial training / tutoring experiences
- Embodied robotics simulators
- Motion-capture-based tools
- Haptic feedback loops
- VR/AR front-ends for deterministic tools

**All heavy compute for XR runs on Sovereign PC, streaming to headsets.**

---

## 2. Software Reality

**Installed (baseline):**
- Windows PC: Anaconda, Python 3.10, Cursor, git, MuJoCo, FastAPI, websockets
- Cloud LLMs: Claude Opus 4.5, GPT, Gemini (via API)

**Not yet installed (reference as "to be installed"):**
- Isaac Sim
- OpenUSD (pxr)
- SOFA
- Local LLM servers (Ollama, LM Studio)
- Vector DB / knowledge graph

---

## 3. Architecture Assumptions

| Layer | Host | Role |
|-------|------|------|
| **Sovereign PC** | Windows Tower | Core engine host — local models, physics, sim, media generation, back-end services |
| **Mac Mini / MacBook** | Apple Silicon | Cognitive surfaces — chat, prompts, planning, supervisory agents, design work |
| **XR / Haptics / Mocap** | Network endpoints | Embodiment front-ends — connect via network to Sovereign PC |

**Design Principles:**
- Autonomy at zero (no unsupervised agents)
- Data local by default
- Inspectable and deterministic at the core

---

## 4. Design Rules

When designing tools, compilers, agents, or architectures:

### Exploit the Hardware
- Offload heavy compute to Sovereign PC
- Use Macs for orchestration, editing, presentations, light dev
- Treat XR gear as live endpoints for embodied/spatial/haptic interaction

### Respect Deterministic/Stochastic Split
- Deterministic cores (physics, compilers, risk engines) → Sovereign PC
- LLMs → UX / explanation / planning layers around cores

### Build a Sovereign Lab
- Design for simulation, experimentation, XR training, robotics, research tools
- Not just a chatbot — a synthetic research lab / product studio

### Minimize Cloud Dependence
- Prefer local models / retrieval / sim when reasonable
- Use cloud LLMs for cognition, not for raw storage or opaque decisions

---

## 5. Workstation Phases

| Phase | Focus |
|-------|-------|
| **Phase 1** | Sovereign Cognitive Stack — Local + cloud LLM orchestration, memory layer, agent shells |
| **Phase 2** | Simulation Stack — Physics (MuJoCo/Isaac), OpenUSD, robotics + soft robotics sim |
| **Phase 3** | Embodied Interfaces — Vision Pro / Quest / haptics / mocap hooked into sim + cognitive stack |
| **Phase 4** | Civilization Kernel — Higher-level planning tools (SRFC/TSRFC/VRFC compilers, curriculum tools, research accelerators) |

---

*When the user asks for "Omega" work, assume it's somewhere inside this stack.*
