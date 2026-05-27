#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:-artifacts/logs}"
mkdir -p "${OUT_DIR}"

{
  echo "timestamp_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "git_sha: $(git rev-parse --verify HEAD 2>/dev/null || true)"
  echo "git_status:"
  git status --short 2>/dev/null || true
} > "${OUT_DIR}/experiment_context.txt"

if command -v ros2 >/dev/null 2>&1; then
  ros2 topic list > "${OUT_DIR}/ros2_topics.txt" 2>&1 || true
  ros2 node list > "${OUT_DIR}/ros2_nodes.txt" 2>&1 || true
  ros2 doctor --report > "${OUT_DIR}/ros2_doctor.txt" 2>&1 || true
fi

echo "Wrote logs to ${OUT_DIR}"
