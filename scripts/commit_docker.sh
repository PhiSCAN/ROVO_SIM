#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

echo "Building/updating rovosim image..."
cd "${PROJECT_DIR}" && docker compose build --no-cache

if [ $? -ne 0 ]; then
  echo "Error: Failed to build image."
  exit 1
fi

echo "Done. Image is now updated."
