# omega-rtx — Full System Inventory

**Machine:** Windows PC (omega-rtx)  
**Purpose:** Alignment inventory for MacBook and Mac mini  
**Date:** 2025-02-04  
**Collected via:** PowerShell (`scripts/inventory.ps1`)

---

## 1. SYSTEM

| Property | Value |
|----------|--------|
| **Computer name** | DESKTOP-J7AGHMV |
| **Windows version** | 2009 |
| **Build** | 26100.1.amd64fre.ge_release.240331-1435 |
| **RAM (visible)** | 66,703,248 KB ≈ **63.6 GB** |
| **CPU** | AMD Ryzen 9 9950X 16-Core Processor |
| **Display adapters** | Meta Virtual Monitor, Virtual Desktop Monitor, Parsec Virtual Display Adapter, SudoMaker Virtual Display Adapter, **NVIDIA GeForce RTX 5090** |

---

## 2. DEV TOOLS

| Tool | Path | Version |
|------|------|---------|
| **git** | `C:\Program Files\Git\cmd\git.exe` | 2.52.0.windows.1 |
| **node** | `C:\Program Files\nodejs\node.exe` | v25.5.0 |
| **python** | `C:\Users\Warren\anaconda3\python.exe` | 3.13.9 |
| **docker** | `C:\Program Files\Docker\Docker\resources\bin\docker.exe` | 29.1.5, build 0e6fee6 |
| **VS Code (code)** | `C:\Users\Warren\AppData\Local\Programs\Microsoft VS Code\bin\code.cmd` | — |
| **WSL** | `C:\WINDOWS\system32\wsl.exe` | — |

**WSL distros:** Ubuntu (default, Stopped), docker-desktop (Stopped) — WSL 2.

---

## 3. DOCKER

- **Client:** 29.1.5 | **Context:** desktop-linux  
- **Plugins:** docker-ai, buildx, compose, debug  
- **Containers:** (none listed)  
- **Images:** (none listed)

---

## 4. PROJECT DIRECTORIES

**Under `%USERPROFILE%` (C:\Users\Warren):**

- **Dev / config:** `.anaconda`, `.aws`, `.azure`, `.cache`, `.conda`, `.config`, `.cursor`, `.docker`, `.dotnet`, `.ipython`, `.jupyter`, `.lmstudio`, `.ollama`, `.streamlit`, `.vscode`, `anaconda3`
- **Projects:** `OmegaStack`, `omega_kernel`, `My project`, `spine-case`
- **Other:** `ComfyUI`, `Desktop`, `Documents`, `Downloads`, `OneDrive`, `Pictures`, `Videos`, etc.

**Note:** `Projects`, `code`, `repos` under profile were not present or empty. No C:\ or D:\ directories matched `project|code|dev|omega|repo` in name (workspace is under profile).

---

## 5. SERVICES (Running, filtered)

Selected running services (excluding core Windows/network/audio/crypto):

| Name | Display name |
|------|----------------|
| AMD Crash Defender Service | AMD Crash Defender Service |
| AmdAppCompatSvc | AMD Application Compatibility Database Service |
| AmdPpkgSvc | AMD Provisioning Packages Service |
| ApolloService | Apollo Service |
| CorsairDeviceControlService | Corsair Device Control Service |
| EasyTuneEngineService | EasyTune Engine Service |
| FvSvc | NVIDIA FrameView SDK service |
| GigabyteUpdateService | GIGABYTE Update Service |
| logi_lamparray_service | Logitech LampArray Service |
| NvContainerLocalSystem | NVIDIA LocalSystem Container |
| NVDisplay.ContainerLocalSystem | NVIDIA Display Container LS |
| OVRService | Oculus VR Runtime Service |
| Parsec | Parsec |
| Steam Client Service | Steam Client Service |
| Tailscale | Tailscale |
| VirtualDesktop.Service.exe | Virtual Desktop Service |
| WSLService | WSL Service |

*(Plus standard Windows services: BFE, DcomLaunch, EventLog, LanmanServer, Winmgmt, etc.)*

---

## 6. SSH

- **sshd service:** Not present (OpenSSH server not installed or not found).
- **`%USERPROFILE%\.ssh`:** Directory not present — no SSH keys collected on this run.

---

## 7. DISK USAGE

| Drive | Used (GB) | Free (GB) | Total (GB) |
|-------|-----------|-----------|------------|
| **C** | 405.9 | 3319.2 | 3725 |
| **E** | 0.2 | 1862.8 | 1863 |

---

## 8. GPU

| Model | Memory | Driver |
|-------|---------|--------|
| NVIDIA GeForce RTX 5090 | 32,607 MiB (~32 GB) | 591.74 |

---

## Alignment checklist (Mac parity)

Use this for matching MacBook / Mac mini:

- [ ] **Git** — version parity (e.g. 2.52.x on Mac).
- [ ] **Node** — v25.5.0 (or align LTS vs current).
- [ ] **Python** — 3.13.x and source (system vs conda vs pyenv).
- [ ] **Docker** — Desktop + WSL2 vs Docker Desktop on Mac.
- [ ] **Editor** — VS Code + Cursor; sync settings/extensions.
- [ ] **Repos** — Same top-level layout (e.g. `~/OmegaStack`, `~/omega_kernel`, `~/My project`, `~/spine-case`).
- [ ] **SSH** — If Mac has `~/.ssh` and keys, add Windows OpenSSH server + same keys if needed.
- [ ] **Services** — Tailscale, Parsec, etc. on Mac as needed.

---

*Inventory script: `scripts/inventory.ps1`. Re-run for updates.*
