#!/bin/bash

set -euo pipefail

# Ensure zero timestamping for hermetic results.
export ZERO_AR_DATE=1
/usr/bin/ar "$@"
