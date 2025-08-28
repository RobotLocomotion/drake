#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on
# Ubuntu.
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euo pipefail

with_doc_only=0
with_maintainer_only=0
with_bazel=1
with_clang=1
with_test_only=1
with_update=1
with_asking=1

# TODO(jwnimmer-tri) Eventually we should default to with_clang=0.

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      with_bazel=1
      with_clang=1
      with_test_only=1
      ;;
    # Install prerequisites that are only needed to build documentation,
    # i.e., those prerequisites that are dependencies of bazel run //doc:build.
    --with-doc-only)
      with_doc_only=1
      ;;
    # Install bazelisk from a deb package.
    --with-bazel)
      with_bazel=1
      ;;
    # Do NOT install bazelisk.
    --without-bazel)
      with_bazel=0
      ;;
    # Install prerequisites that are only needed for --config clang, i.e.,
    # opts-in to the ability to compile Drake's C++ code using Clang.
    --with-clang)
      with_clang=1
      ;;
    # Do NOT install prerequisites that are only needed for --config clang,
    # i.e., opts-out of the ability to compile Drake's C++ code using Clang.
    --without-clang)
      with_clang=0
      ;;
    # Install prerequisites that are only needed to run select maintainer
    # scripts. Most developers will not need to install these dependencies.
    --with-maintainer-only)
      with_maintainer_only=1
      ;;
    # Do NOT install prerequisites that are only needed to build and/or run
    # unit tests, i.e., those prerequisites that are not dependencies of
    # bazel { build, run } //:install.
    --without-test-only)
      with_test_only=0
      ;;
    # Do NOT call apt-get update during execution of this script.
    --without-update)
      with_update=0
      ;;
    # Pass -y along to apt-get.
    -y)
      with_asking=0
      ;;
    *)
      echo 'Invalid command line argument' >&2
      exit 3
  esac
  shift
done

if [[ "${EUID}" -ne 0 ]]; then
  echo 'ERROR: This script must be run as root' >&2
  exit 1
fi

if [[ "${with_asking}" -eq 0 ]]; then
  maybe_yes='-y'
else
  maybe_yes=''
fi

if [[ "${with_update}" -eq 1 && "${binary_distribution_called_update:-0}" -ne 1 ]]; then
  apt-get update || (sleep 30; apt-get update)
fi

apt-get install ${maybe_yes} --no-install-recommends $(cat <<EOF
ca-certificates
wget
EOF
)

codename=$(lsb_release -sc)

packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}.txt")
apt-get install ${maybe_yes} --no-install-recommends ${packages}

# Ensure that we have available a locale that supports UTF-8 for generating a
# C++ header containing Python API documentation during the build.
apt-get install ${maybe_yes} --no-install-recommends locales
locale-gen en_US.UTF-8

# We need a working /usr/bin/python (of any version).
if [[ ! -e /usr/bin/python ]]; then
  apt-get install ${maybe_yes} --no-install-recommends python-is-python3
else
  echo "/usr/bin/python is already installed"
fi

if [[ "${with_doc_only}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-doc-only.txt")
  apt-get install ${maybe_yes} --no-install-recommends ${packages}
fi

if [[ "${with_bazel}" -eq 1 ]]; then
  source "${BASH_SOURCE%/*}/install_bazelisk.sh"
fi

if [[ "${with_clang}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-clang.txt")
  apt-get install ${maybe_yes} --no-install-recommends ${packages}
fi

if [[ "${with_test_only}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-test-only.txt")
  apt-get install ${maybe_yes} --no-install-recommends ${packages}
  if [[ "${codename}" == "noble" ]]; then
    "${BASH_SOURCE%/*}/install_kcov.sh"
  fi
fi

if [[ "${with_maintainer_only}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-maintainer-only.txt")
  apt-get install ${maybe_yes} --no-install-recommends ${packages}
fi

# On Jammy, Drake doesn't install anything related to GCC 12, but if the user
# has chosen to install some GCC 12 libraries but has failed to install all of
# them correctly as a group, Drake's documentation header file parser will fail
# with a libclang-related complaint. Therefore, we'll help the user clean up
# their mess, to avoid apparent Drake build errors.
if [[ "${codename}" == "jammy" ]]; then
  status=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgcc-12-dev 2>/dev/null || true)
  if [[ "${status}" == "ii " ]]; then
    status_stdcxx=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libstdc++-12-dev 2>/dev/null || true)
    status_fortran=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgfortran-12-dev 2>/dev/null || true)
    if [[ "${status_stdcxx}" != "ii " || "${status_fortran}" != "ii " ]]; then
      apt-get install ${maybe_yes} --no-install-recommends libgcc-12-dev libstdc++-12-dev libgfortran-12-dev
    fi
  fi
fi

# On Noble, Drake doesn't install anything related to GCC 14, but if the user
# has chosen to install some GCC 14 libraries but has failed to install all of
# them correctly as a group, Drake's documentation header file parser will fail
# with a libclang-related complaint. Therefore, we'll help the user clean up
# their mess, to avoid apparent Drake build errors.
if [[ "${codename}" == "noble" ]]; then
  status=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgcc-14-dev 2>/dev/null || true)
  if [[ "${status}" == "ii " ]]; then
    status_stdcxx=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libstdc++-14-dev 2>/dev/null || true)
    status_fortran=$(dpkg-query --show --showformat='${db:Status-Abbrev}' libgfortran-14-dev 2>/dev/null || true)
    if [[ "${status_stdcxx}" != "ii " || "${status_fortran}" != "ii " ]]; then
      apt-get install ${maybe_yes} --no-install-recommends libgcc-14-dev libstdc++-14-dev libgfortran-14-dev
    fi
  fi
fi
