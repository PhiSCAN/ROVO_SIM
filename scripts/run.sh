#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# Stop any existing containers
cd "${PROJECT_DIR}" && docker compose down 2>/dev/null || true

xhost +local: >/dev/null 2>&1 || true

# Gazebo Fuel model cache vendored in-repo (gz_cache/) and mounted via docker-compose
mkdir -p "${PROJECT_DIR}/gz_cache"

cd "${PROJECT_DIR}" && docker compose up -d
