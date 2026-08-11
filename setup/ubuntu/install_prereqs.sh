# Install build prerequisites (and optionally developer prerequisites) for Drake
# on Ubuntu.

set -euo pipefail

at_exit () {
    echo "${me} has experienced an error on line ${LINENO}" \
        "while running the command ${BASH_COMMAND}"
}

me='The Drake source distribution prerequisite setup script'

trap at_exit EXIT

# ============================ Command line options ============================

binary_args=()
developer=0

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      developer=1
      ;;
    # Do NOT call apt-get update during execution of this script.
    --without-update)
      binary_args+=(--without-update)
      ;;
    -y)
      binary_args+=(-y)
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
source "${BASH_SOURCE%/*}/install_prereqs_binary.sh" "${binary_args[@]}"

# ================================ Build prereqs ===============================

readonly workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../.." && pwd)"

packages=$(cat "${BASH_SOURCE%/*}/packages-${VERSION_CODENAME}-build.txt")
${maybe_sudo} apt-get install ${maybe_yes} --no-install-recommends ${packages}

# ============================== Developer prereqs =============================

if [[ "${developer}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${VERSION_CODENAME}-developer.txt")
  ${maybe_sudo} apt-get install ${maybe_yes} --no-install-recommends ${packages}
fi

# ================================== Finished ==================================

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
