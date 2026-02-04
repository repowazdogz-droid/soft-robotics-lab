# omega-rtx Architecture Diagram

High-level view of projects on omega-rtx and how they connect. See [OMEGA_RTX_PROJECT_ANALYSIS.md](OMEGA_RTX_PROJECT_ANALYSIS.md) for full analysis.

---

## System context

- **omega-rtx:** Windows PC (this machine).
- **OmegaStack:** Monorepo of research products; Reality Bridge :8000 and Substrate are shared.
- **spine-case:** Standalone app (also a copy under OmegaStack/products).
- **omega_kernel:** Empty directory.
- **My project:** Unity XR; no link to others.

---

## Mermaid diagram

```mermaid
flowchart TB
    subgraph OMEGA_RTX["omega-rtx (Windows PC)"]
        subgraph OmegaStack["OmegaStack repo"]
            OC["omega_console :8500"]
            RB["Reality Bridge API :8000"]
            RBUI["Reality Bridge UI :8501"]
            BE["Breakthrough Engine :8502"]
            OT["OMEGA Tutor :8503"]
            OF["OMEGA Foundry :8501/8504"]
            WMS["World Model Studio :8505"]
            OSci["OMEGA Scientist :8506"]
            DB["Decision Brief :8507"]
            SRL["Soft Robotics Lab :8501"]
            Sub["Substrate (memory/knowledge)"]
            FL["Frontline (TBD)"]
        end

        SC["spine-case :5173\nC:\\Users\\Warren\\spine-case"]
        SC2["spine-case\nproducts/spine-case"]
        UK["omega_kernel (empty)"]
        MP["My project (Unity XR)"]
    end

    subgraph External["External / Cloud"]
        Gemini["Google Gemini API"]
        OpenAI["OpenAI API"]
        LM["LM Studio :1234"]
    end

    subgraph Future["Planned"]
        MacMini["Mac Mini – Lab OS :8000"]
    end

    OC --> RB
    OC --> OF
    OC --> WMS
    OC --> BE
    OC --> DB
    OC --> OT
    OF --> RB
    WMS --> RB
    BE --> Sub
    DB --> Sub
    OF --> LM
    OF --> Gemini
    OT --> LM
    OT --> Gemini
    OSci --> LM
    DB --> LM
    DB --> Gemini
    DB --> OpenAI
    SRL -.->|lab_os_client| MacMini
    RB --> RBUI

    SC --> Gemini
    SC2 --> Gemini

    MP -.->|no link| OmegaStack
    UK -.->|empty| OmegaStack
```

---

## Port map (quick reference)

| Port | Service |
|------|---------|
| 8000 | Reality Bridge API (shared validation) |
| 8500 | OMEGA Console (entry point) |
| 8501 | Reality Bridge Dashboard / Foundry / Soft Robotics Lab |
| 8502 | Breakthrough Engine |
| 8503 | OMEGA Tutor |
| 8504 | OMEGA Foundry (alternate) |
| 8505 | World Model Studio |
| 8506 | OMEGA Scientist |
| 8507 | Decision Brief |
| 1234 | LM Studio (optional local LLM) |
| 5173 | spine-case (Vite dev) |
