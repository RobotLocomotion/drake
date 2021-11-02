#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on macOS.

set -euxo pipefail

with_update=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Do NOT call brew update during execution of this script.
    --without-update)
      with_update=0
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

if command -v conda &>/dev/null; then
  echo 'WARNING: Anaconda is NOT supported for building and using the Drake Python bindings' >&2
fi

if ! command -v brew &>/dev/null; then
  /bin/bash -c \
    "$(/usr/bin/curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

# Pass --retry 2 when invoking the curl command line tool during execution of
# this script to retry twice the download of a bottle.
[[ -z "${HOMEBREW_CURL_RETRIES:-}" ]] || export HOMEBREW_CURL_RETRIES=2

# Do not automatically update before running brew install, brew upgrade, or brew
# tap during execution of this script. We manually update where necessary and
# retry if the update fails.
export HOMEBREW_NO_AUTO_UPDATE=1

# Do not automatically cleanup installed, upgraded, and/or reinstalled formulae
# during execution of this script. Manually run brew cleanup after the script
# completes or wait for the next automatic cleanup if necessary.
export HOMEBREW_NO_INSTALL_CLEANUP=1

binary_distribution_called_update=0

if [[ "${with_update}" -eq 1 ]]; then
  # Note that brew update uses git, so HOMEBREW_CURL_RETRIES does not take
  # effect.
  brew update || (sleep 30; brew update)

  # Do NOT call brew update again when installing prerequisites for source
  # distributions.
  binary_distribution_called_update=1
fi

brew bundle --file="${BASH_SOURCE%/*}/Brewfile" --no-lock

if ! command -v pip3.9 &>/dev/null; then
  echo 'ERROR: pip3.9 is NOT installed. The post-install step for the python@3.9 formula may have failed.' >&2
  exit 2
fi

pip3.9 install -r "${BASH_SOURCE%/*}/requirements.txt"
