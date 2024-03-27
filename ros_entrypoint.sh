#!/bin/bash
set -e

# Move to host home directory
sudo chown ioes-docker:ioes-docker /home/ioes-docker/host
cd /home/ioes-docker/host

exec "$@"