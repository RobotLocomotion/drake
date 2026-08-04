# Install build prerequisites (and optionally developer prerequisites) for Drake
# on macOS.

set -euxo pipefail

# ============================ Command line options ============================

binary_args=(--without-python-dependencies)
developer=0

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      developer=1
      ;;
    # Do NOT call brew update during execution of this script.
    --without-update)
      binary_args+=(--without-update)
      ;;
    # Ignored for compatibility with Ubuntu.
    -y)
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 1
  esac
  shift
done

# =============================== Binary prereqs ===============================

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.
#
# N.B. We need `${var:0}` here because macOS's older version of bash does
# not seem to be able to cope with an empty array.
source "${BASH_SOURCE%/*}/install_prereqs_binary.sh" "${binary_args[@]:0}"

# ================================ Build prereqs ===============================

brew bundle --file="${BASH_SOURCE%/*}/Brewfile-build"

# ============================== Developer prereqs =============================

if [[ "${developer}" -eq 1 ]]; then
  brew bundle --file="${BASH_SOURCE%/*}/Brewfile-developer"
fi

# ================================== Finished ==================================
