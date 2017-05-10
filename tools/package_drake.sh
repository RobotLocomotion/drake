#!/bin/bash
### Build and install/package Drake and its dependencies.
### After installing the package, OS X users may want to adjust the .so file's
### id to match its installed location, using install_name_tool -id.

# If any command in the script exits non-zero, stop.
set -e

# If any unset variable is referenced, stop.
set -u

#------------------------------------------------------------------------------
# Convert path to canonical form.
function canonicalize
{
  # Get the absolute path with symlinks resolved. Using Perl seems to be the
  # most portable way to do this, since `readlink -f` is a GNU extension that
  # isn't available on BSD derived platforms (e.g. MacOS).
  perl -MCwd -le 'print Cwd::abs_path(shift)' "$1"
}

#------------------------------------------------------------------------------
# Copy source artifacts to specified install prefix.
function copy_source_artifacts
{
  cp LICENSE.TXT "$1/"
}

#------------------------------------------------------------------------------
# Copy built artifacts to specified install prefix.
function copy_build_artifacts
{
  cp bazel-bin/external/lcm/liblcm.so "$1/lib/"
  cp bazel-bin/drake/libdrake.so "$1/lib/"
  cp bazel-genfiles/tools/drake-config.cmake "$1/lib/cmake/drake/"
}

#------------------------------------------------------------------------------
# Extract headers to specified install prefix.
function extract_headers
{ (
  cd "$1/include/external"
  mkdir scratch && cd scratch

  # Un-tar the headers. The -P flag and scratch directory are necessary because
  # Bazel bakes a .. into the paths of external headers.
  tar -xPzf "$workspace/bazel-bin/drake/libdrake_headers.tar.gz"

  # Headers from Drake proper are now inside include/external/scratch, while
  # headers from externals are in include/external. Move the Drake headers up
  # to include/drake.
  mv drake "$tmpdir/include"

  # Remove the scratch directory
  cd ..
  rmdir scratch
) }

#------------------------------------------------------------------------------
# Extract licenses to specified install prefix.
function extract_licenses
{ (
  cd "$1/include/external"
  tar -xzf "$workspace/bazel-bin/drake/external_licenses.tar.gz"
) }

#------------------------------------------------------------------------------
# Verify licenses in the specified install prefix.
function verify_licenses
{ (
  cd "$1/include/external"

  missing=()

  # Inspect each external.
  for ext in $(ls -d1 */); do
    # Look for a license file or files.
    if [ -z "$(find $ext -name 'LICENSE*' -o -name 'COPYING*')" ]; then
      missing+=(${ext%/})
    fi
  done

  # Check if any licenses were missing, and if so, report them.
  if [ ${#missing[*]} -gt 0 ]; then
    printf -v missing '%s, ' "${missing[@]}"
    echo "Fatal: missing license for ${missing%, }" >&2
    exit 1
  fi
) }

#------------------------------------------------------------------------------
# Create tarball of specified install prefix ($1) with specified name ($2).
function create_tarfile
{ (
  cd "$1"
  tar -czf "$2" *
) }

#------------------------------------------------------------------------------
# Copy temporary install prefix ($1) to final install prefix ($2).
function install_tree
{ (
  # Change to source tree and iterate over files.
  cd "$1"
  for f in *; do
    [ "$f" == '*' ] && break # guard against empty directory

    # Test if directory entry is a file or another directory.
    if [ -d "$f" ]; then
      # Recursively traverse directories.
      mkdir -p "$2/$f"
      install_tree "$1/$f" "$2/$f" "${3-}$f/"
    else
      # Copy file if different, otherwise do nothing.
      if cmp "$1/$f" "$2/$f" >/dev/null 2>&1; then
        echo "[Up to Date] ${3-}$f"
      else
        echo "[Installing] ${3-}$f"
        cp "$1/$f" "$2/$f"
      fi
    fi
  done
) }

###############################################################################

# Initialize action variables.
install_prefix=
output_tarfile=

# Get actions
while getopts 'd:f:v' opt; do
  case $opt in
    d) install_prefix="$OPTARG";;
    f) output_tarfile="$OPTARG";;
    v) set -x
  esac
done

# Test that we were asked to do something.
if [ -z "$install_prefix" ] && [ -z "$output_tarfile" ]; then
  echo "No install directory or output tar file specified." >&2
  echo "Usage: $0 [-d INSTALL_DIR] [-f FILE.tar.gz]"
  exit 1
fi

# Resolve output paths (before changing directory!).
if [ -n "$output_tarfile" ]; then
  tmp="$(canonicalize "$output_tarfile")"
  if [ -z "$output_tarfile" ]; then
    echo "Fatal: cannot write '$output_tarfile'" >&2
    exit 1
  fi
  output_tarfile="$tmp"
  unset tmp
fi
if [ -n "$install_prefix" ]; then
  mkdir -p "$install_prefix"
  if ! [ -d "$install_prefix" ]; then
    echo "Fatal: cannot create install directory '$install_prefix'" >&2
    exit 1
  fi
  install_prefix="$(canonicalize "$install_prefix")"
fi

# Find the bazel workspace root and set as working directory.
cd "$(dirname "$(canonicalize "$0")")"
workspace="$(canonicalize "$(bazel info workspace)")"
cd "$workspace"

# Build Drake.
bazel build \
  //drake:libdrake.so \
  //drake:libdrake_headers \
  //drake:external_licenses

# Create and populate temporary install tree.
tmpdir=$(mktemp -d)
mkdir -p "$tmpdir/lib/cmake/drake"
mkdir -p "$tmpdir/include/external"

copy_source_artifacts "$tmpdir"
copy_build_artifacts "$tmpdir"
extract_headers "$tmpdir"
extract_licenses "$tmpdir"

verify_licenses "$tmpdir"

chmod -R u+w,ugo+r "$tmpdir"

# Execute requested actions.
[ -n "$output_tarfile" ] && create_tarfile "$tmpdir" "$output_tarfile"
[ -n "$install_prefix" ] && install_tree "$tmpdir" "$install_prefix"

# Remove temporary install tree
rm -r "$tmpdir"
