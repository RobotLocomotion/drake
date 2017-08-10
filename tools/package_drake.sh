#!/bin/bash
### Build and install/package Drake and its dependencies.

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

###############################################################################

# Initialize action variables.
output_tarfile=""

# Get actions
while getopts 'd:f:v' opt; do
  case $opt in
    f) output_tarfile="$OPTARG";;
    d) echo "The -d option is deprecated; use 'bazel run install -- <prefix>'";;
    v) set -x;;
  esac
done
shift $((OPTIND - 1))
[ -z "$output_tarfile" ] && [ $# -gt 0 ] && output_tarfile="$1"

# Test that an output tar file name was given.
if [ -z "$output_tarfile" ]; then
  echo "No output tar file specified." >&2
  echo "Usage: $0 [-v] [-f] FILE.tar.gz"
  exit 1
fi

# Resolve output path (before changing directory!).
tmp="$(canonicalize "$output_tarfile")"
if [ -z "$tmp" ]; then
  echo "Fatal: cannot write '$output_tarfile'" >&2
  exit 1
fi
output_tarfile="$tmp"
unset tmp

# Find the bazel workspace root and set as working directory.
cd "$(dirname "$(canonicalize "$0")")"
workspace="$(canonicalize "$(bazel info workspace)")"
cd "$workspace"

# Create and install into a temporary install tree.
tmpdir=$(mktemp -d)
# Build Drake.
bazel run install -- "$tmpdir" >&-
chmod -R u+w,ugo+r "$tmpdir"

# Create tar file.
(
  cd "$tmpdir"
  tar -czf "$output_tarfile" *
)

# Remove temporary install tree.
rm -r "$tmpdir"
