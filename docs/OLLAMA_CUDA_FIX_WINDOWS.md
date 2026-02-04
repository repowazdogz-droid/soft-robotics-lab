# Ollama CUDA Error Fix — Windows 11 + RTX 5090

## 1. Diagnosis (what was checked)

- **GPU:** `nvidia-smi` — RTX 5090, Driver 591.74, CUDA 13.1. GPU is present and working (~14.8 GB in use; LM Studio and other apps are using it).
- **Ollama:** One process `ollama.exe` (e.g. PID 8932), version 0.15.2, at `%LOCALAPPDATA%\Programs\Ollama\ollama.exe`.
- **Environment:** You tried `OLLAMA_NO_GPU=1` before `ollama serve`; the error persisted. On Windows, **OLLAMA_NO_GPU is not the variable that forces CPU**. Ollama uses a different mechanism.

## 2. Root cause (likely)

1. **Wrong env var for CPU on Windows**  
   `OLLAMA_NO_GPU=1` does not force CPU in current Ollama on Windows. You must either:
   - Set **`OLLAMA_LLM_LIBRARY=cpu_avx2`** (or `cpu_avx` / `cpu`) so Ollama uses the CPU backend and does not touch the GPU, or  
   - Set **`CUDA_VISIBLE_DEVICES=-1`** so the CUDA backend sees no GPU and falls back.

2. **GPU in use by other apps**  
   LM Studio (and others) were using the GPU. That can cause:
   - VRAM pressure so Ollama’s model load fails with a CUDA error, or  
   - CUDA context/initialization conflicts when Ollama tries to use the same GPU.

3. **RTX 5090 / very new driver**  
   RTX 5090 (Blackwell) and driver 591.74 are very new. Ollama 0.15.2 may have been built against an older CUDA runtime; some “CUDA error” crashes can be compatibility issues that get fixed in a newer Ollama or driver.

So: the **immediate** fix is to force CPU with the correct env var and a clean Ollama restart. The **long‑term** fix for GPU is to free the GPU (e.g. close LM Studio), update Ollama/drivers, and then try GPU again.

## 3. What fixes it

### Option A — Force CPU (reliable fallback)

1. **Quit Ollama completely**  
   - Right‑click Ollama in the system tray → **Quit**, or  
   - Run:
     ```powershell
     Get-Process -Name "ollama*" -ErrorAction SilentlyContinue | Stop-Process -Force
     ```
2. **Start Ollama in CPU-only mode**  
   In PowerShell:
   ```powershell
   $env:OLLAMA_LLM_LIBRARY = "cpu_avx2"
   ollama serve
   ```
   Leave this window open. In a **second** PowerShell window:
   ```powershell
   ollama run llama3:8b "Say hello"
   ```
   You should get a text response (slower than GPU, but no CUDA error).

3. **Make CPU mode persistent (optional)**  
   - **System:**  
     `Win + R` → `sysdm.cpl` → **Advanced** → **Environment Variables** → **User** or **System** → **New**:  
     - Name: `OLLAMA_LLM_LIBRARY`  
     - Value: `cpu_avx2`  
   - Then start Ollama from the Start menu / tray as usual; it will use CPU until you remove the variable.

### Option B — Try GPU again (after CPU works)

1. **Close LM Studio** (and any other heavy GPU apps) so the RTX 5090 has free VRAM.
2. **Remove the CPU override**  
   - Delete the `OLLAMA_LLM_LIBRARY` env var if you set it, or  
   - Start Ollama from a shell **without** that variable.
3. **Restart Ollama** (tray → Quit, then start again).
4. Run:
   ```powershell
   ollama run llama3:8b "Say hello"
   ```
   If you still get “llama runner process has terminated: CUDA error”:
   - Update to the **latest Ollama** from [ollama.com](https://ollama.com).
   - Ensure **latest NVIDIA driver** (591.74 is already very new; check for any newer Game Ready / Studio driver).
   - Watch [Ollama GitHub issues](https://github.com/ollama/ollama/issues) for “RTX 5090” or “Blackwell” or “CUDA” + “Windows”.

## 4. Verify

- **CPU mode:** With `OLLAMA_LLM_LIBRARY=cpu_avx2` and `ollama serve` running, `ollama run llama3:8b "Say hello"` should return a short reply and no CUDA error.
- **Verified:** On this machine, after stopping Ollama, setting `OLLAMA_LLM_LIBRARY=cpu_avx2`, and starting `ollama serve`, `ollama run llama3:8b "Say hello"` returned: *"Hello! It's nice to meet you. Is there something I can help you with, or would you like to chat?"*
- **GPU mode:** With no `OLLAMA_LLM_LIBRARY`, other GPU apps closed, and Ollama restarted, the same command should work and typically run faster (if GPU is used).

## 5. How to prevent in future

- **If you rely on CPU fallback:** Keep `OLLAMA_LLM_LIBRARY=cpu_avx2` set in your user environment so every Ollama start (tray, CLI) uses CPU and never hits CUDA.
- **If you want GPU:**  
  - Close LM Studio (and similar) before using Ollama when you need the GPU.  
  - Keep Ollama and the NVIDIA driver updated to reduce RTX 5090 / Blackwell compatibility issues.

## 6. Quick script (run in PowerShell)

From the repo root you can run:

```powershell
.\scripts\ollama_cpu_fix.ps1
```

That script stops Ollama, sets `OLLAMA_LLM_LIBRARY=cpu_avx2`, and starts `ollama serve`. Then in a **new** terminal run:

```powershell
ollama run llama3:8b "Say hello"
```
