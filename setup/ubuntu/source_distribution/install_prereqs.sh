#!/bin/bash
#
# Install development prerequisites for source distributions of Drake on
# Ubuntu 18.04 (Bionic) or 20.04 (Focal).
#
# The development and runtime prerequisites for binary distributions should be
# installed before running this script.

set -euo pipefail

with_doc_only=0
with_kcov=0
with_maintainer_only=0
with_test_only=1
with_update=1
with_asking=1

while [ "${1:-}" != "" ]; do
  case "$1" in
    # Install prerequisites that are only needed to build documentation,
    # i.e., those prerequisites that are dependencies of bazel run //doc:build.
    --with-doc-only)
      with_doc_only=1
      ;;
    # Install the kcov code coverage analysis tool from the
    # drake-apt.csail.mit.edu apt repository on Ubuntu 18.04 (Bionic). Ignored
    # on Ubuntu 20.04 (Focal) where kcov is always installed from the Ubuntu
    # "universe" apt repository.
    --with-kcov)
      with_kcov=1
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
  apt_get_install='apt-get install -y'
else
  apt_get_install='apt-get install'
fi

if [[ "${with_update}" -eq 1 && "${binary_distribution_called_update:-0}" -ne 1 ]]; then
  apt-get update || (sleep 30; apt-get update)
fi

$apt_get_install --no-install-recommends $(cat <<EOF
ca-certificates
wget
EOF
)

codename=$(lsb_release -sc)

# On Bionic, developers must opt-in to kcov support; it comes in with the
# non-standard package name kcov-35 via a Drake-specific apt repository. If
# --without-update is passed to this script, then the gpg public key must
# already be trusted, the apt repository must already have been added to the
# list of sources, and apt-get update must have been called.
if [[ "${codename}" == 'bionic' ]] && [[ "${with_kcov}" -eq 1 ]]; then
  $apt_get_install --no-install-recommends gnupg
  wget -q -O- https://drake-apt.csail.mit.edu/drake.asc \
    | apt-key --keyring /etc/apt/trusted.gpg.d/drake.gpg add
  if [[ "${with_update}" -eq 1 ]]; then
    echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/${codename} ${codename} main" \
      > /etc/apt/sources.list.d/drake.list
    apt-get update || (sleep 30; apt-get update)
  fi
  $apt_get_install --no-install-recommends kcov-35
fi

packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}.txt")
$apt_get_install --no-install-recommends ${packages}

# Ensure that we have available a locale that supports UTF-8 for generating a
# C++ header containing Python API documentation during the build.
$apt_get_install --no-install-recommends locales
locale-gen en_US.UTF-8

if [[ "${codename}" == 'focal' ]]; then
  # We need a working /usr/bin/python (of any version).  On Bionic it's there
  # by default, but on Focal we have to ask for it.
  if [[ ! -e /usr/bin/python ]]; then
    $apt_get_install --no-install-recommends python-is-python3
  else
    echo "/usr/bin/python is already installed"
  fi
fi

if [[ "${with_doc_only}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-doc-only.txt")
  $apt_get_install --no-install-recommends ${packages}
fi

if [[ "${with_test_only}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-test-only.txt")
  # Suppress Python 3.8 warnings when installing python3-pandas on Focal.
  PYTHONWARNINGS=ignore::SyntaxWarning \
    $apt_get_install --no-install-recommends ${packages}
fi

if [[ "${with_maintainer_only}" -eq 1 ]]; then
  packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}-maintainer-only.txt")
  $apt_get_install --no-install-recommends ${packages}
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
$apt_get_install --no-install-recommends $(cat <<EOF
g++
unzip
zlib1g-dev
EOF
)

dpkg_install_from_wget \
  bazel 4.2.1 \
  https://releases.bazel.build/4.2.1/release/bazel_4.2.1-linux-x86_64.deb \
  67447658b8313316295cd98323dfda2a27683456a237f7a3226b68c9c6c81b3a
