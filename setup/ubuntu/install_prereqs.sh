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

# Ensure that we have available a locale that supports UTF-8 for generating a
# C++ header containing Python API documentation during the build.
${maybe_sudo} apt-get install ${maybe_yes} --no-install-recommends locales
${maybe_sudo} locale-gen en_US.UTF-8

# We need a working /usr/bin/python (of any version).
if [[ ! -e /usr/bin/python ]]; then
  ${maybe_sudo} apt-get install ${maybe_yes} --no-install-recommends python-is-python3
else
  echo "/usr/bin/python is already installed"
fi

# On Noble, Drake doesn't install anything related to GCC 14, but if the user
# has chosen to install some GCC 14 libraries but has failed to install all of
# them correctly as a group, Drake's documentation header file parser will fail
# with a libclang-related complaint. Therefore, we'll help the user clean up
# their mess, to avoid apparent Drake build errors.
if [[ "${VERSION_CODENAME}" == "noble" ]]; then
  status=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgcc-14-dev 2>/dev/null || true)
  if [[ "${status}" == "ii " ]]; then
    status_stdcxx=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libstdc++-14-dev 2>/dev/null || true)
    status_fortran=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgfortran-14-dev 2>/dev/null || true)
    if [[ "${status_stdcxx}" != "ii " || "${status_fortran}" != "ii " ]]; then
      ${maybe_sudo} apt-get install ${maybe_yes} --no-install-recommends libgcc-14-dev libstdc++-14-dev libgfortran-14-dev
    fi
  fi
fi

# ============================== Developer prereqs =============================

if [[ "${developer}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${VERSION_CODENAME}-developer.txt")
  ${maybe_sudo} apt-get install ${maybe_yes} --no-install-recommends ${packages}
fi

# ================================== Finished ==================================

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
