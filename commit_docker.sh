#!/bin/bash

CONTAINER_NAME="rovo-sim"
COMMIT_IMAGE_NAME="rovo-sim-committed"
FINAL_IMAGE_TAG="navigation2:development"

echo "Committing container '$CONTAINER_NAME' to image '$COMMIT_IMAGE_NAME'..."
docker commit "$CONTAINER_NAME" "$COMMIT_IMAGE_NAME"

if [ $? -ne 0 ]; then
  echo "Error: Failed to commit container."
  exit 1
fi

echo "Stopping container '$CONTAINER_NAME'..."
docker stop "$CONTAINER_NAME"

if [ $? -ne 0 ]; then
  echo "Error: Failed to stop container."
  exit 1
fi

echo "Retagging image '$COMMIT_IMAGE_NAME' to '$FINAL_IMAGE_TAG'..."
docker tag "$COMMIT_IMAGE_NAME" "$FINAL_IMAGE_TAG"

if [ $? -ne 0 ]; then
  echo "Error: Failed to retag image."
  exit 1
fi

echo "Done. Image is now tagged as '$FINAL_IMAGE_TAG'."
