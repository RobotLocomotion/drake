# Installs prerequisites for source distributions of Drake.
#
# This is (only) used as a subroutine of the parent directory's prereqs script.

set -euo pipefail

with_doc_only=0
with_maintainer_only=0
with_bazel=1
with_clang=1
with_test_only=1
with_asking=1

# TODO(jwnimmer-tri) On 2025-01-01 change the default value for with_bazel,
# with_clang, and with_test_only to 0.

# The docs for these options are in the parent ../install_prereqs.sh script.
while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      with_doc_only=1
      with_bazel=1
      with_clang=1
      with_test_only=1
      ;;
    --with-doc-only)
      with_doc_only=1
      ;;
    --without-doc-only)
      with_doc_only=0
      ;;
    --with-bazel)
      with_bazel=1
      ;;
    --without-bazel)
      with_bazel=0
      ;;
    --with-clang)
      with_clang=1
      ;;
    --without-clang)
      with_clang=0
      ;;
    --with-maintainer-only)
      with_maintainer_only=1
      ;;
    --without-maintainer-only)
      with_maintainer_only=0
      ;;
    --with-test-only)
      with_test_only=1
      ;;
    --without-test-only)
      with_test_only=0
      ;;
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

codename=$(lsb_release -sc)

packages=$(cat "${BASH_SOURCE%/*}/packages-${codename}.txt")
apt-get install ${maybe_yes} --no-install-recommends ${packages}

# Ensure that we have available a locale that supports UTF-8 for generating a
# C++ header containing Python API documentation during the build.
if ! command -v locale-gen &>/dev/null; then
  apt-get install ${maybe_yes} --no-install-recommends locales
else
  echo 'locale-gen is already installed' >&2
fi
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
