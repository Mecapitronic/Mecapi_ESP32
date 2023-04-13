#!/usr/bin/env bash

# Configure git
echo "Getting host' git configurations in devcontainer"
sudo cp /root/.gitconfig ~
sudo chown -R "$(id -u):$(id -g)" ~/.gitconfig

# enable pre-commit hook
# pre-commit install --install-hooks

