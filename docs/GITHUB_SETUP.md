# GitHub setup: omega-stack (private)

## 1. Create the repo on GitHub

- **GitHub UI**: [New repository](https://github.com/new)
  - Name: `omega-stack`
  - Visibility: **Private**
  - Do **not** initialize with README, .gitignore, or license (we already have them locally).

- **Or with GitHub CLI** (if installed):
  ```bash
  gh repo create omega-stack --private --source=. --remote=origin --push
  ```
  (Run from `C:\Users\Warren\OmegaStack`; omit `--push` if you prefer to push manually.)

## 2. Add remote and push (if not using `gh repo create --push`)

From the repo root (OmegaStack):

```bash
git remote add origin https://github.com/YOUR_ORG/omega-stack.git
git branch -M main
git push -u origin main
```

Replace `YOUR_ORG` with your GitHub username or org. If you use SSH:

```bash
git remote add origin git@github.com:YOUR_ORG/omega-stack.git
git push -u origin main
```

## 3. Optional: set your identity for this repo

The initial commit used a placeholder identity. To set your name/email for this repo only:

```bash
git config user.name "Your Name"
git config user.email "you@example.com"
```

## 4. Verify: clone to a fresh folder and run

```bash
git clone https://github.com/YOUR_ORG/omega-stack.git omega-stack-fresh
cd omega-stack-fresh
python scripts/smoke_test.py
```

Then run a product, e.g.:

```bash
cd products/omega_tutor
pip install -r requirements.txt
streamlit run app.py
```
