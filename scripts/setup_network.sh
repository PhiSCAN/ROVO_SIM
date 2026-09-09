#!/bin/bash
# setup_network.sh
# One-time setup for network configuration required by the simulation.
# This must be run on the host machine (not inside the container).

# Bind robot IP 192.168.1.200 to host loopback so the in-container Zenoh
# router (listening on tcp/[::]:7447) is reachable at tcp/192.168.1.200:7447,
# matching ZENOH_CONFIG_OVERRIDE.
if ! ip addr show lo | grep -q "192.168.1.200"; then
  echo "Setting up 192.168.1.200 loopback alias for Zenoh..."
  sudo ip addr add 192.168.1.200/32 dev lo
  if [ $? -eq 0 ]; then
    echo "Done. Loopback alias added."
  else
    echo "ERROR: Failed to add loopback alias. Try with sudo."
    exit 1
  fi
else
  echo "Loopback alias 192.168.1.200 already exists."
fi
