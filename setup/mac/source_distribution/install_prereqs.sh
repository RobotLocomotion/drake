# Installs development prerequisites for source distributions of Drake.
#
# This is (only) used as a subroutine of the parent directory's prereqs script.

set -euxo pipefail

with_test_only=1

# The docs for these options are in the parent ../install_prereqs.sh script.
while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      with_test_only=1
      ;;
    --with-test-only)
      with_test_only=1
      ;;
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

if ! command -v brew &>/dev/null; then
  echo 'ERROR: brew is NOT installed. Please ensure that the prerequisites for binary distributions have been installed.' >&2
  exit 4
fi

brew bundle --file="${BASH_SOURCE%/*}/Brewfile" --no-lock

if [[ "${with_test_only}" -eq 1 ]]; then
  brew bundle --file="${BASH_SOURCE%/*}/Brewfile-test-only" --no-lock
fi
