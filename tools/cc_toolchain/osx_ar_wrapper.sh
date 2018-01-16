#!/bin/bash

set -euo pipefail

export ZERO_AR_DATE=1
/usr/bin/ar "$@"
