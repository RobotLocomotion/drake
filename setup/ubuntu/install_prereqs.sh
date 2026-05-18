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

# ================================== Functions =================================

dpkg_install_from_wget() {
  package="$1"
  version="$2"
  url="$3"
  checksum="$4"

  # Skip the install if we're already at the exact version.
  installed=$(dpkg-query --showformat='${Version}\n' --show "${package}" 2>/dev/null || true)
  if [[ "${installed}" == "${version}" ]]; then
    echo "${package} is already at the desired version ${version}"
    return
  fi

  # If installing our desired version would be a downgrade, ask the user first.
  if dpkg --compare-versions "${installed}" gt "${version}"; then
    echo "This system has ${package} version ${installed} installed."
    echo "Drake suggests downgrading to version ${version}, our supported version."
    read -r -p 'Do you want to downgrade? [Y/n] ' reply
    if [[ ! "${reply}" =~ ^([yY][eE][sS]|[yY])*$ ]]; then
      echo "Skipping ${package} ${version} installation."
      return
    fi
  fi

  # Download and verify.
  tmpdeb="/tmp/${package}_${version}-$(dpkg-architecture -qDEB_HOST_ARCH).deb"
  wget -O "${tmpdeb}" "${url}"
  if echo "${checksum} ${tmpdeb}" | sha256sum -c -; then
    echo  # Blank line between checkout output and dpkg output.
  else
    echo "ERROR: The ${package} deb does NOT have the expected SHA256. Not installing." >&2
    exit 2
  fi

  # Install.
  ${maybe_sudo} dpkg -i "${tmpdeb}"
  rm "${tmpdeb}"
}

# =============================== Binary prereqs ===============================

# Dependencies that are installed by the following sourced script that are
# needed when developing with binary distributions are also needed when
# developing with source distributions.
source "${BASH_SOURCE%/*}/install_prereqs_binary.sh" "${binary_args[@]}"

# ================================ Build prereqs ===============================

readonly workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../.." && pwd)"

${maybe_sudo} apt-get install ${maybe_yes} --no-install-recommends wget

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

  # Install bazelisk.
  # If bazel.deb is already installed, we'll need to remove it first because
  # the Debian package of bazelisk will take over the `/usr/bin/bazel` path.
  ${maybe_sudo} apt-get remove bazel || true
  if [[ $(arch) = "aarch64" ]]; then
    dpkg_install_from_wget \
      bazelisk 1.28.1 \
      https://github.com/bazelbuild/bazelisk/releases/download/v1.28.1/bazelisk-arm64.deb \
      47f787fe814c1bbc3b414ec5a876de706fd12fd0fa62f51549f3487575a827f3
  else
    dpkg_install_from_wget \
      bazelisk 1.28.1 \
      https://github.com/bazelbuild/bazelisk/releases/download/v1.28.1/bazelisk-amd64.deb \
      16c2d58a2e78171cf8db21bb4ae7908d91d9cf54ba62efb140d2a42731a3ed60
  fi

  # Install kcov.
  if [[ "${VERSION_CODENAME}" == "noble" ]]; then
    # Because Noble does not offer kcov natively, this file was
    # mirrored from Ubuntu 25.04 Plucky at https://packages.ubuntu.com/plucky/kcov.
    if [[ $(arch) = "x86_64" ]]; then
      dpkg_install_from_wget \
        kcov 43+dfsg-1 \
        https://drake-mirror.csail.mit.edu/ubuntu/pool/universe/k/kcov/kcov_43%2Bdfsg-1_amd64.deb \
        d192fd3cfd0d63e95f13a1b2120d0603a31b3a034b82c618d5e18205517d5cbb
    fi
  fi
fi

# ================================== Finished ==================================

trap : EXIT  # Disable exit reporting.
echo 'install_prereqs: success'
