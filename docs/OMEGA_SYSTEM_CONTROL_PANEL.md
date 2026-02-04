# OMEGA System Control Panel

> Single source of truth for operating the Omega ecosystem.

---

## Machine Roles

| Machine | Hostname | Tailscale IP | Role |
|---------|----------|--------------|------|
| **Windows PC** | omega-rtx | 100.71.44.93 | GPU compute, simulation, research tools (OmegaStack) |
| **MacBook Pro** | omega-control | 100.67.61.20 | Development, governance layer, applications |
| **Mac mini** | omega-core | 100.103.173.86 | Infrastructure services, backups, future Lab OS |

---

## Port Assignments

| Port | Service | Machine | Env Variable |
|------|---------|---------|--------------|
| 18000 | Reality Bridge | omega-rtx | `REALITY_BRIDGE_URL` |
| 18002 | Lab OS (future) | omega-core | `LAB_OS_URL` |
| 18001 | omega-f | omega-control | `OMEGA_F_URL` |
| 18500 | omega_console | omega-rtx | `OMEGA_CONSOLE_URL` |
| 3000 | app (Next.js) | omega-control | `APP_URL` |
| 1234 | LM Studio | omega-rtx | `LM_STUDIO_URL` |

**Rule:** No hardcoded ports. All services use env vars with defaults.

---

## How to Start Services

### Windows PC (omega-rtx)

```powershell
cd C:\Users\Warren\OmegaStack

# Reality Bridge (validation API)
cd products\reality_bridge
uvicorn app:app --port 18000

# omega_console (launcher dashboard)
cd products\omega_console
streamlit run app.py --server.port 18500

# OMEGA Scientist
cd products\omega_scientist
streamlit run app.py

# OMEGA Foundry
cd products\omega_foundry
streamlit run app.py

# OMEGA Tutor
cd products\omega_tutor
streamlit run app.py

# World Model Studio
cd products\world_model_studio
streamlit run app.py

# Soft Robotics Lab
cd products\soft_robotics_lab
streamlit run app.py

# Breakthrough Engine
cd products\breakthrough_engine
streamlit run app.py
```

### MacBook (omega-control)

```bash
cd ~/Omega

# app (Next.js UI)
cd app && npm run dev

# omega-f (governance API)
cd omega-f && uvicorn output.api_interface:app --port 18001

# constraint-universe
cd constraint-universe && python example_governance.py

# tsrfc (CLI)
cd tsrfc && tsrfc --help
```

### Mac mini (omega-core)

```bash
cd ~/omega-infra

# Start all Docker services
docker compose -f compose/backup.yml up -d

# Check status
docker ps

# Manual backup
docker compose -f compose/backup.yml exec backup /backups/backup.sh
```

---

## How to Stop Services

### Windows PC
- `Ctrl+C` in the terminal running the service
- Or close the terminal window

### MacBook
- `Ctrl+C` in the terminal running the service

### Mac mini
```bash
docker compose -f compose/backup.yml down
```

---

## Logs

| Service | Location |
|---------|----------|
| Backups (Mac mini) | `~/omega-infra/backup.log` |
| Docker containers | `docker logs omega-postgres`, `docker logs omega-redis`, `docker logs omega-backup` |
| Streamlit apps | Terminal stdout |
| Next.js app | Terminal stdout |

---

## Backups

### Schedule
- **Daily at 2am** via macOS launchd on Mac mini

### What's backed up
- PostgreSQL dump
- Redis RDB (via `redis-cli --rdb`)
- Config files (compose/, env/)

### Storage
- **Backblaze B2:** `b2:omega-backups:omega`

### Retention
- 7 daily, 4 weekly, 6 monthly snapshots

### Manual backup
```bash
# On Mac mini
docker compose -f compose/backup.yml exec backup /backups/backup.sh
```

### Check backup status
```bash
# On Mac mini
launchctl list | grep omega
cat ~/omega-infra/backup.log
docker compose -f compose/backup.yml exec backup restic snapshots
```

### Restore

**Postgres:**
```bash
docker compose -f compose/backup.yml exec backup \
  restic dump --tag postgres latest /staging/postgres/postgres-$(date +%Y-%m-%d).sql.gz \
  | gunzip | docker exec -i omega-postgres psql -U omega
```

**Redis:**
```bash
docker compose -f compose/backup.yml exec backup \
  restic dump --tag redis latest /staging/redis/redis-$(date +%Y-%m-%d).rdb \
  > /tmp/restored.rdb
# Then copy to Redis container and restart
```

---

## Secrets Location

| Secret | Location | Notes |
|--------|----------|-------|
| Restic password | `~/omega-infra/env/.env` | `RESTIC_PASSWORD` |
| B2 credentials | `~/omega-infra/env/.env` | `B2_ACCOUNT_ID`, `B2_ACCOUNT_KEY` |
| Postgres password | `~/omega-infra/env/.env` | `PGPASSWORD` |
| Redis password | `~/omega-infra/env/.env` | `REDIS_PASSWORD` |
| Gemini API key | OmegaStack `.env` | `GEMINI_API_KEY` |
| OpenAI API key | app `.env.local` | `OPENAI_API_KEY` |

**Important:** Back up these secrets separately (password manager, printed copy, etc.)

---

## Git Repositories

| Repo | Location | Remote |
|------|----------|--------|
| omega-protocol | MacBook `~/Omega` | github.com/repowazdogz-droid/omega-protocol |
| soft-robotics-lab | Windows `C:\Users\Warren\OmegaStack` | github.com/repowazdogz-droid/soft-robotics-lab |

### Spine Contracts (shared via subtree)
- **Source of truth:** omega-protocol (`~/Omega/spine/`)
- **Consumed by:** soft-robotics-lab (as git subtree at `spine/`)

**To update Spine in OmegaStack:**
```powershell
cd C:\Users\Warren\OmegaStack
git subtree pull --prefix=spine omega-protocol main --squash
```

---

## Health Checks

### Quick system check
```bash
# MacBook - ping all machines
ping -c 1 100.71.44.93   # omega-rtx
ping -c 1 100.103.173.86  # omega-core

# Mac mini - check Docker
docker ps

# Windows - check Reality Bridge
curl http://localhost:18000/health
```

### Full verification
1. Reality Bridge responds: `curl http://100.71.44.93:18000/health`
2. Docker containers running: `docker ps` on Mac mini
3. Backup ran today: `cat ~/omega-infra/backup.log | tail -20`
4. App loads: `http://localhost:3000` on MacBook

---

## Disaster Recovery

### Total loss of Mac mini
1. New Mac mini with Docker installed
2. Clone omega-infra: `git clone <repo>`
3. Restore `.env` from password manager
4. Start containers: `docker compose -f compose/backup.yml up -d`
5. Restore from B2:
   ```bash
   docker compose -f compose/backup.yml exec backup restic restore latest --target /
   ```

### Total loss of Windows PC
1. New Windows with Python, Node, Git
2. Clone OmegaStack: `git clone https://github.com/repowazdogz-droid/soft-robotics-lab.git OmegaStack`
3. Install dependencies: `pip install -r requirements.txt` per product
4. Restore `.env` from password manager

### Total loss of MacBook
1. New MacBook with Homebrew, Node, Python
2. Clone omega-protocol: `git clone https://github.com/repowazdogz-droid/omega-protocol.git Omega`
3. Install dependencies per project
4. Restore `.env.local` files from password manager

---

## Common Tasks

### Update Spine contracts everywhere
```bash
# 1. Edit on MacBook
cd ~/Omega/spine
# make changes
git add -A && git commit -m "update contracts"
git push

# 2. Pull to Windows
cd C:\Users\Warren\OmegaStack
git subtree pull --prefix=spine omega-protocol main --squash
git push
```

### Add a new service
1. Pick next available port (18xxx)
2. Add to this README
3. Use env var (e.g., `NEW_SERVICE_URL`)
4. Add to `.env.example`

### Check what's running on a port
```bash
# Mac
lsof -i :18000

# Windows
netstat -ano | findstr :18000
```

---

## Contacts / Resources

- **Tailscale admin:** https://login.tailscale.com/admin
- **Backblaze B2:** https://secure.backblaze.com/b2_buckets.htm
- **GitHub repos:** https://github.com/repowazdogz-droid

---

*Last updated: 2026-02-04*
