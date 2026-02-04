# omega-rtx full system inventory for Mac alignment
$ErrorActionPreference = 'SilentlyContinue'

Write-Host "=== SYSTEM ==="
Get-ComputerInfo | Select-Object CsName, WindowsVersion, WindowsBuildLabEx, OsTotalVisibleMemorySize
Get-WmiObject Win32_Processor | Select-Object Name
Get-WmiObject Win32_VideoController | Select-Object Name

Write-Host "`n=== DEV TOOLS ==="
@('git','node','python','docker','code','wsl') | ForEach-Object {
    $exe = $_
    $p = (Get-Command $exe -ErrorAction SilentlyContinue).Source
    if ($p) { Write-Host "$exe : $p" } else { Write-Host "$exe : (not in PATH)" }
}
git --version 2>$null
node -v 2>$null
python --version 2>$null
docker --version 2>$null
wsl --list --verbose 2>$null

Write-Host "`n=== DOCKER ==="
docker info 2>$null | Select-Object -First 15
Write-Host "--- Containers ---"
docker ps -a 2>$null
Write-Host "--- Images ---"
docker images 2>$null

Write-Host "`n=== PROJECT DIRECTORIES (Profile) ==="
Get-ChildItem -Path $env:USERPROFILE -Directory | Select-Object Name
$extra = @("$env:USERPROFILE\Projects","$env:USERPROFILE\code","$env:USERPROFILE\repos")
Get-ChildItem -Path $extra -ErrorAction SilentlyContinue -Directory | Select-Object FullName
Get-ChildItem -Path "C:\","D:\" -Directory -ErrorAction SilentlyContinue | Where-Object { $_.Name -match 'project|code|dev|omega|repo' } | Select-Object FullName

Write-Host "`n=== SERVICES (Running, filtered) ==="
Get-Service | Where-Object { $_.Status -eq 'Running' -and $_.Name -notmatch '^(Windows|Wcm|Wlan|Token|Wsearch|Spooler|Power|Plug|Net|Audio|App|Crypt)' } | Select-Object Name, DisplayName | Format-Table -AutoSize

Write-Host "`n=== SSH ==="
Get-Service sshd -ErrorAction SilentlyContinue | Select-Object Status
Get-ChildItem "$env:USERPROFILE\.ssh\*.pub" -ErrorAction SilentlyContinue | Select-Object FullName

Write-Host "`n=== DISK USAGE ==="
Get-PSDrive -PSProvider FileSystem | Select-Object Name, @{N='Used(GB)';E={[math]::Round($_.Used/1GB,1)}}, @{N='Free(GB)';E={[math]::Round($_.Free/1GB,1)}}, @{N='Total(GB)';E={[math]::Round(($_.Used+$_.Free)/1GB,1)}}

Write-Host "`n=== GPU ==="
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv 2>$null
