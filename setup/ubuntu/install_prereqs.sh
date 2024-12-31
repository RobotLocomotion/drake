#!/bin/bash

set -euo pipefail

echo 'WARNING: This script is deprecated and will be removed on or after 2025-07-01; instead, run drake/setup/install_prereqs' >&2

exec "${BASH_SOURCE%/*}/../install_prereqs" --verbose --developer "$@"
