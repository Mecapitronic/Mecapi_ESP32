#!/usr/bin/env bash

# Configure ssh and git
echo "Getting host' git configurations in devcontainer"

sudo cp -rf /root/.ssh ~
sudo chown -R "$(id -u):$(id -g)" ~/.ssh

sudo cp /root/.gitconfig ~
sudo chown -R "$(id -u):$(id -g)" ~/.gitconfig

python3 -m pip install -r **/requirements.txt
# enable pre-commit hook
# pre-commit install --install-hooks

