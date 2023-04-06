#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on
# Ubuntu 20.04 (Focal) or 22.04 (Jammy).
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euo pipefail

with_doc_only=0
with_maintainer_only=0
with_clang=1
with_test_only=1
with_update=1
with_asking=1

# TODO(jwnimmer-tri) Eventually we should default to with_clang=0.

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Install prerequisites that are only needed to build documentation,
    # i.e., those prerequisites that are dependencies of bazel run //doc:build.
    --with-doc-only)
      with_doc_only=1
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

if [[ "${with_clang}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-clang.txt")
  apt-get install ${maybe_yes} --no-install-recommends ${packages}
fi

if [[ "${with_test_only}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-test-only.txt")
  apt-get install ${maybe_yes} --no-install-recommends ${packages}
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
  tmpdeb="/tmp/${package}_${version}-amd64.deb"
  wget -O "${tmpdeb}" "${url}"
  if echo "${checksum} ${tmpdeb}" | sha256sum -c -; then
    echo  # Blank line between checkout output and dpkg output.
  else
    echo "ERROR: The ${package} deb does NOT have the expected SHA256. Not installing." >&2
    exit 2
  fi

  # Install.
  dpkg -i "${tmpdeb}"
  rm "${tmpdeb}"
}

# Install bazel package dependencies (these may duplicate dependencies of
# drake).
apt-get install ${maybe_yes} --no-install-recommends $(cat <<EOF
g++
unzip
zlib1g-dev
EOF
)

# Install bazel.
# Keep this version number in sync with the drake/.bazeliskrc version number.
if [[ $(arch) = "aarch64" ]]; then
  # Check if bazel is already installed.
  if [[ "$(which bazel)" ]]; then
    echo "Bazel is already installed." >&2
  else
    echo "WARNING: On Ubuntu arm64 systems, Drake's install_prereqs does not" \
    "automatically install Bazel on your behalf. You will need to install" \
    "Bazel yourself. See https://bazel.build for instructions." >&2
  fi
else
  dpkg_install_from_wget \
    bazel 6.1.1 \
    https://releases.bazel.build/6.1.1/release/bazel_6.1.1-linux-x86_64.deb \
    a90246165f0972629506132975a7c5d5aecd42453e03e0f88e175a33601cdf70
fi
