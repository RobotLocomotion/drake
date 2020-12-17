#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on macOS.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euxo pipefail

with_maintainer_only=0
with_test_only=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Install prerequisites that are only needed to run select maintainer
    # scripts. Most developers will not need to install these dependencies.
    --with-maintainer-only)
      with_maintainer_only=1
      ;;
    # Do NOT install prerequisites that are only needed to build documentation,
    # i.e., those prerequisites that are dependencies of bazel { build, run }
    # { //doc:gen_sphinx, //bindings/pydrake/doc:gen_sphinx, //doc:doxygen }
    --without-doc-only)
      echo 'DEPRECATED: The --without-doc-only option is the default and will be deprecated on or after 2021-01-01' >&2
      ;;
    # Do NOT install prerequisites that are only needed to build and/or run
    # unit tests, i.e., those prerequisites that are not dependencies of
    # bazel { build, run } //:install.
    --without-test-only)
      with_test_only=0
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 5
  esac
  shift
done

if [[ "${EUID}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

if ! command -v /usr/local/bin/brew &>/dev/null; then
  echo 'ERROR: brew is NOT installed. Please ensure that the prerequisites for binary distributions have been installed.' >&2
  exit 4
fi

/usr/local/bin/brew update
/usr/local/bin/brew bundle --file="${BASH_SOURCE%/*}/Brewfile" --no-lock

if [[ "${with_maintainer_only}" -eq 1 ]]; then
  /usr/local/bin/brew bundle \
    --file="${BASH_SOURCE%/*}/Brewfile-maintainer-only" --no-lock
fi

if ! command -v /usr/local/opt/python@3.8/bin/pip3  &>/dev/null; then
  echo 'ERROR: pip3 for python@3.8 is NOT installed. The post-install step for the python@3.8 formula may have failed.' >&2
  exit 2
fi

if [[ "${with_test_only}" -eq 1 ]]; then
  /usr/local/opt/python@3.8/bin/pip3 install --upgrade --requirement \
    "${BASH_SOURCE%/*}/requirements-test-only.txt"
fi

if [[ "${with_maintainer_only}" -eq 1 ]]; then
  /usr/local/opt/python@3.8/bin/pip3 install --upgrade --requirement \
    "${BASH_SOURCE%/*}/requirements-maintainer-only.txt"
fi
