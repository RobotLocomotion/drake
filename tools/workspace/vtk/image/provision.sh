#!/bin/bash

set -eu -o pipefail

export DEBIAN_FRONTEND=noninteractive

# Install prerequisites.
apt-get -y update
apt-get -y upgrade

xargs -d$'\n' apt-get -y install --no-install-recommends < /image/prereqs
