#!/usr/bin/env bash
# Create a public GitHub repo and push (requires: gh auth login).
set -euo pipefail

TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${TELEOP_ROOT}"

REPO_NAME="${1:-manus-wuji-teleop}"
VIS="${2:-public}"

if ! gh auth status >/dev/null 2>&1; then
  echo "Run first: gh auth login"
  exit 1
fi

gh repo create "${REPO_NAME}" --"${VIS}" --source=. --remote=origin \
  --description "Manus data gloves teleop for Wuji Hand 2 (MuJoCo sim + real robot TCP bridge)"

git push -u origin main
echo "Done: https://github.com/$(gh api user -q .login)/${REPO_NAME}"
