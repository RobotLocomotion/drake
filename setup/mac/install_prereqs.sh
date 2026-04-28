# Install build prerequisites (and optionally developer prerequisites) for Drake
# on macOS.

set -euxo pipefail

# ============================ Command line options ============================

binary_args=(--without-python-dependencies)
user_environment_only=0
developer=0

while [ "${1:-}" != "" ]; do
  case "$1" in
    --developer)
      developer=1
      ;;
    --user-environment-only)
      user_environment_only=1
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

if [[ "${user_environment_only}" -eq 0 ]]; then
  # Dependencies that are installed by the following sourced script that are
  # needed when developing with binary distributions are also needed when
  # developing with source distributions.
  #
  # N.B. We need `${var:-}` here because mac's older version of bash does
  # not seem to be able to cope with an empty array.
  source "${BASH_SOURCE%/*}/install_prereqs_binary.sh" "${binary_args[@]:-}"
fi

# ================================ Build prereqs ===============================

if [[ "${user_environment_only}" -eq 0 ]]; then
  brew bundle --file="${BASH_SOURCE%/*}/Brewfile-build"
else
  developer=0
fi

# ============================== Developer prereqs =============================

if [[ "${developer}" -eq 1 ]]; then
  brew bundle --file="${BASH_SOURCE%/*}/Brewfile-developer"
fi

# ============================== User environment ==============================

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../.." && pwd)"
bazelrc="${workspace_dir}/gen/environment.bazelrc"

mkdir -p "$(dirname "${bazelrc}")"
cat > "${bazelrc}" <<EOF
EOF

# Prefetch the bazelisk download of bazel.
# This is especially helpful for the "Provisioned" images in CI.
(cd "${workspace_dir}" && bazelisk version) > /dev/null

# Our MODULE.bazel uses this file to determine the default python version.
# When changing this, see drake/tools/workspace/python/README.md.
echo "3.14" > "${workspace_dir}/gen/python_version.txt"

# ================================== Finished ==================================
