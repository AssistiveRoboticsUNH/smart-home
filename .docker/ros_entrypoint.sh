#!/bin/bash
set -e

# setup ros environment
source "install/setup.bash"
#ifconfig eth0 192.168.1.5 netmask 255.255.255.0
exec "$@"
