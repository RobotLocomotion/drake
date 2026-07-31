#!/bin/bash

# upgrade.sh - Upgrades the lockfile for Drake's Rust crates.
#
# This program is only tested / supported on Ubuntu.

set -eu -o pipefail
cd $(dirname $(python3 -c 'import os; print(os.path.realpath("'"$0"'"))'))

# Update the Cargo lockfile using the Rust toolchain configured in MODULE.bazel.
bazel run @rules_rust//tools/upstream_wrapper:cargo -- \
    update --manifest-path lock/Cargo.toml

# Regenerate the crate names used for license installation.
python3 - lock/Cargo.lock lock/repo_names.bzl <<'PY'
import json
import pathlib
import sys
import tomllib

lockfile = pathlib.Path(sys.argv[1])
output = pathlib.Path(sys.argv[2])
packages = tomllib.loads(lockfile.read_text())['package']
names = sorted(
    ('crate__{}-{}'.format(package['name'], package['version']))
    .replace('+', '-')
    for package in packages
    if package.get('source')
)
output.write_text(
    'REPO_NAMES = [\n' +
    ''.join('    {},\n'.format(json.dumps(name)) for name in names) +
    ']\n'
)
PY
