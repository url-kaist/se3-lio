#!/usr/bin/env bash
# Build the python-only image. Context is the repo root (needs cpp/ + python/).
set -e
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
docker build -f "$REPO_ROOT/docker/python/Dockerfile" -t se3_lio:py "$REPO_ROOT"
