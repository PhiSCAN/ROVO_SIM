#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# Ensure container is running
if [ -z "$(cd "${PROJECT_DIR}" && docker compose ps -q rovosim)" ]; then
  echo "Starting rovosim container..."
  "${SCRIPT_DIR}/run.sh"
fi

cd "${PROJECT_DIR}" && docker compose exec rovosim bash
