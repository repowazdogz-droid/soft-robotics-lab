# Simulation Infrastructure Audit — C:\Users\Warren

**Date:** 2026-02-02  
**Scope:** Current installs, GPU, storage, network, OmegaStack simulation assets, Python environments.

---

## 1. Current Installs

| Component | Status | Location / Notes |
|-----------|--------|------------------|
| **NVIDIA Omniverse** | Evidence only (cache/config) | No standalone Omniverse app in Program Files. User data: `C:\Users\Warren\.nvidia-omniverse\` (logs, config). Cache: `C:\Users\Warren\AppData\Local\ov\` (Kit, shaders, ogn_generated). |
| **Isaac Sim** | **Installed** | **`C:\isa\`** — Full install: `kit\`, `apps\` (isaacsim.exp.full.kit, streaming, etc.), `exts\`, `standalone_examples\`, `tests\`. Log shows **Isaac-Sim Full 5.1.0**, Kit **107.3.3**. Last run from log: 2026-01-18. |
| **MuJoCo** | Via Python only | No standalone binary. **Conda env `compute`**: `mujoco==3.4.0`. Root/OmegaStack: `mujoco` commented out in root `requirements.txt`; products (omega_foundry, world_model_studio, reality_bridge) require `mujoco>=3.0.0`. |
| **Blender** | **Installed** | **`C:\Program Files\Blender Foundation\Blender 5.0\`** |
| **Unreal Engine** | Not found | `C:\Program Files\Epic Games\` does not exist. |
| **Unity** | **Installed** | **`C:\Program Files\Unity\`** (+ Unity Hub in Program Files). Also `C:\Users\Warren\AppData\Local\Unity\` (projects/cache). |
| **CUDA toolkit** | Not in standard path | `C:\Program Files\NVIDIA Corporation\NVIDIA GPU Computing Toolkit\CUDA` not present. Driver reports **CUDA 13.1** (driver-bundled capability). For custom CUDA toolkit (e.g. for compiling extensions), install separately. |
| **Python (simulation)** | Conda + Isaac | **Default `python`**: 3.13.9 (resolves to `C:\Users\Warren\OmegaStack\python` shim or anaconda3). **Anaconda base**: `C:\Users\Warren\anaconda3\`. **Conda env `compute`**: has `mujoco`, `usd-core`; no `omni.*` (omni only in Isaac Sim’s bundled Python). **Isaac Sim Python**: `C:\isa\kit\python\` (use via Isaac’s launcher or `C:\isa\kit\setup_python_env.bat`). |

**NVIDIA Corporation (Program Files):**  
FrameView SDK, Installer2 (display driver, PhysX, ShadowPlay, NvContainer, etc.), NvContainer, NVIDIA App, NvTelemetry. No Omniverse/Isaac there; Isaac lives under `C:\isa\`.

---

## 2. GPU Status

**nvidia-smi (summary):**

- **GPU:** NVIDIA GeForce RTX 5090 (WDDM)
- **Driver:** 591.74
- **CUDA (driver):** 13.1
- **VRAM:** 2306 MiB / 32607 MiB used (~7%) → **~30.3 GB available**
- **Power:** 65 W / 600 W, 25°C, P1

**Processes using GPU:** ShellExperienceHost, Edge, Explorer, LM Studio, Parsec, Cursor, Steam Web Helper, etc. LM Studio listed as compute (C).

---

## 3. Storage

| Drive | Total | Free | Notes |
|-------|--------|------|-------|
| **C:** | ~3.73 TB | ~3.31 TB | ~413 GB used. OS, apps, OmegaStack, Anaconda, ComfyUI, Isaac Sim (C:\isa), etc. |
| **E:** | ~1.86 TB | ~1.86 TB | Effectively full free. Good candidate for large simulation assets. |

**Recommendation:** Put large simulation assets (USD caches, Isaac/Omniverse projects, recorded data) on **E:\** if it’s a fast SSD to keep C: lean and avoid filling the boot drive.

---

## 4. Network

- **Local IPv4:** **192.168.0.221**
- **Mac Mini access:** Same subnet (192.168.0.x) can reach this machine by IP. Ensure Windows firewall allows the desired services (RDP, Parsec, file share, etc.).
- **Streaming:** **Parsec** is installed (`C:\Program Files\Parsec\`, process seen in nvidia-smi). **Virtual Desktop** and **Virtual Desktop Streamer** are in Program Files (VR streaming). No Sunshine/Moonlight detected in this audit; Parsec provides remote desktop/game streaming.

---

## 5. Existing Simulation Assets (OmegaStack)

**Paths checked:** `C:\Users\Warren\OmegaStack`

- **.usd files:** **None** in repo.
- **.mjcf files:** **107** total:
  - **50** in **soft_robotics_lab:** `products\soft_robotics_lab\gripper_zoo\designs\gd_20260201_0001` … `gd_20260201_0050\` (one `.mjcf` per design).
  - **Outside soft_robotics_lab:**
    - `products\world_model_studio\assets\grippers\test_gripper.mjcf`
    - `products\omega_foundry\outputs\OF_20260129_0001.mjcf` (root of outputs)
    - `products\omega_foundry\outputs\OF_20260129_0001\design.mjcf`
  - omega_foundry/outputs also has: `.xml`, `.stl`, `.urdf`, `.json` (e.g. OF_20260129_0001, OF_E_*).
- **Omniverse projects:** No Omniverse project folders found under OmegaStack; only code/docs references (e.g. soft_robotics_lab, omega_tutor_v2, OMEGA_WORKSTATION_CONTEXT).
- **Isaac Sim projects:** No dedicated Isaac “projects” folder under OmegaStack. Isaac Sim itself and examples live under `C:\isa\` (standalone_examples, etc.).

---

## 6. Python Environment

| Check | Result |
|------|--------|
| **Python version(s)** | System/default: **3.13.9**. Paths: `C:\Users\Warren\OmegaStack\python` (empty shim), `C:\Users\Warren\anaconda3\python.exe`, `%LocalAppData%\WindowsApps\python.exe`. |
| **`usd-core`** | **Yes** in conda env **`compute`**: `usd-core==25.11`. Not in anaconda **base**. |
| **`mujoco`** | **Yes** in conda env **`compute`**: `mujoco==3.4.0`. Base: no. OmegaStack root `requirements.txt`: commented out; product reqs (omega_foundry, world_model_studio, reality_bridge) list `mujoco>=3.0.0`. |
| **`omni.*`** | **No** in conda/base/compute. `omni` is only available inside **Isaac Sim’s Python** (e.g. `C:\isa\kit\python` after running Isaac’s `setup_python_env.bat` or launching an Isaac app). |

**Conda envs:** `base`, `compute`.  
**Compute env:** Has `mujoco`, `usd-core`; `import mujoco` and `import pxr` (USD) succeed; `import omni` fails (expected outside Isaac).

---

## Summary Table

| Item | Finding |
|------|---------|
| Isaac Sim | Installed at **C:\isa\** (5.1.0, Kit 107.3.3) |
| Omniverse | Cache/config only; no standalone app path in Program Files |
| MuJoCo | In conda **compute** (3.4.0); product reqs reference it |
| Blender | 5.0 @ Program Files |
| Unity | Installed (+ Hub); Unreal not found |
| CUDA toolkit | Not installed in standard path; driver CUDA 13.1 |
| GPU | RTX 5090, 32 GB VRAM, driver 591.74 |
| Storage | C: ~3.3 TB free; E: ~1.86 TB free — use E: for heavy simulation assets |
| Local IP | 192.168.0.221; Parsec + Virtual Desktop present |
| OmegaStack .usd | 0 |
| OmegaStack .mjcf | 107 (50 in soft_robotics_lab, 3 outside in world_model_studio + omega_foundry) |
| Python sim stack | **compute**: mujoco + usd-core; **omni** only in Isaac Sim Python |

---

*Audit performed by automated checks; paths and versions are as of the audit date.*
