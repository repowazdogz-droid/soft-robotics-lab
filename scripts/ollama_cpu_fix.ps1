# Ollama CPU fallback fix for Windows (CUDA error workaround)
# 1. Stops all Ollama processes
# 2. Sets OLLAMA_LLM_LIBRARY=cpu_avx2 for this session
# 3. Starts ollama serve (foreground). Press Ctrl+C to stop.
# Then in a NEW terminal run: ollama run llama3:8b "Say hello"

Write-Host "Stopping all Ollama processes..." -ForegroundColor Yellow
Get-Process -Name "ollama*" -ErrorAction SilentlyContinue | ForEach-Object {
    Write-Host "  Stopping $($_.ProcessName) (PID $($_.Id))"
    Stop-Process -Id $_.Id -Force -ErrorAction SilentlyContinue
}
# Child processes (e.g. ollama_llama_server) may have different names; try common ones
@("ollama_llama_server", "ollama_run_server") | ForEach-Object {
    Get-Process -Name $_ -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue
}
Start-Sleep -Seconds 2

$env:OLLAMA_LLM_LIBRARY = "cpu_avx2"
Write-Host "Set OLLAMA_LLM_LIBRARY=cpu_avx2 (CPU-only mode)" -ForegroundColor Green
Write-Host "Starting ollama serve (leave this window open)..." -ForegroundColor Green
Write-Host "In a NEW terminal run: ollama run llama3:8b `"Say hello`"" -ForegroundColor Cyan
Write-Host ""

ollama serve
