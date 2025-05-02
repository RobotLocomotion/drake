#!/bin/bash

# Internal script to prepare the build location.

set -eu -o pipefail

mkdir -p ~/.drake-wheel-build
readonly d="$(mktemp -d ~/.drake-wheel-build/${1:-}XXXXXXXXXX)"
ln -nsf "${d}" /tmp/drake-wheel-build
