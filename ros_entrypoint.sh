#!/bin/bash
set -e

# Move to host home directory
cd /home/ioes-docker/host

exec "$@"