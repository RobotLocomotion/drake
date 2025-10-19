#!/bin/bash
#
# Install development and runtime prerequisites for binary distributions of
# Drake on macOS.

set -euxo pipefail

with_update=1
with_python_dependencies=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Do NOT call brew update during execution of this script.
    --without-update)
      with_update=0
      ;;
    # Do NOT install Python (pip) dependencies.
    --without-python-dependencies)
      with_python_dependencies=0
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
  echo 'NOTE: Drake is not tested regularly with Anaconda, so you may experience compatibility hiccups; when asking for help, be sure to mention that Conda is involved.' >&2
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

if [[ "${with_update}" -eq 1 ]]; then
  # Note that brew update uses git, so HOMEBREW_CURL_RETRIES does not take
  # effect.
  brew update || (sleep 30; brew update)
fi

brew bundle --file="${BASH_SOURCE%/*}/Brewfile"

if ! command -v pip3.14 &>/dev/null; then
  echo 'ERROR: pip3.14 is NOT installed. The post-install step for the python@3.14 formula may have failed.' >&2
  exit 2
fi

if [[ "${with_python_dependencies}" -eq 1 ]]; then
  readonly setup="${BASH_SOURCE%/*}"
  readonly venv_pdm="${setup}"
  readonly venv_drake="$(cd "${setup}/../../.." && pwd)"
  python3.14 -m venv "${venv_pdm}"
  "${venv_pdm}/bin/pip3" install -U -r "${setup}/requirements.txt"
  "${venv_pdm}/bin/python3" -m venv "${venv_drake}"
  "${venv_pdm}/bin/pdm" use -p "${setup}" -f "${venv_drake}"
  "${venv_pdm}/bin/pdm" sync -p "${setup}" --prod
fi
