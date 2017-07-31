#!/bin/bash
#
# With arguments, fixes the given bazel file(s).  Without arguments, fixes
# every BUILD, .BUILD, and *.bzl file except third_party.

set -e

usage() {
  cat <<EOF
usage: $0 <options> -- <buildifier_options> <file>...

Build //tools:buildifier and run on a given set of files.

Without any files or buildifier options, this will reformat all BUILD
and *.bzl files in the entire Drake workspace using -mode=fix.

  -f, --force    Skip prompt when reformatting all.

To see buildifier's help:
       $0 -- --help

EOF
}

workspace=$(cd $(dirname $0) && bazel info workspace)
if [[ ! -f "$workspace/WORKSPACE" ]]; then
  echo "$0: Cannot find WORKSPACE"
  exit 1
fi

force=
while [[ $# -gt 0 ]]; do
  case $1 in
    -f|--force)
      force=1
      shift;;
    --)
      shift
      break;;
    -h|--help)
      usage
      exit 0;;
    *)
      break;;
  esac
done

format_all=
if [[ $# -eq 0 ]]; then
  format_all=1
  if [[ -z $force ]]; then
    echo "This will reformat ALL build files underneath the drake workspace:"
    echo "  $workspace"
    echo "HINT: Use -f to avoid this prompt in the future."
    echo
    echo "Are you sure you want to continue? [y/N]"
    while true; do
      read ans
      case $ans in
        y)
          break;;
        yes|Y)
          echo "Please answer [y/N]";;
        *)
          echo "Aborting"
          exit 1;;
      esac
    done
  fi
fi

echo "Refreshing buildifier binary ..."
bazel build --show_result=0 //tools:buildifier
buildifier="$workspace"/bazel-bin/external/com_github_bazelbuild_buildtools/buildifier/buildifier
tables="$workspace"/tools/buildifier-tables.json
if [[ ! -x $buildifier ]]; then
  echo "Failed to build buildifier at $buildifier"
  exit 1
fi

if [[ -z $format_all ]]; then
  echo "Running buildifier with passed arguments..."
  echo "$buildifier -add_tables=$tables $@"
  "$buildifier" -add_tables="$tables" "$@"
else
  echo "Applying buildifier to everything! This may take a moment..."
  find "$workspace" \
       -name third_party -prune -o \
       \( -name WORKSPACE -o \
          -name BUILD -o \
          -name BUILD.bazel -o \
          -name '*.BUILD' -o \
          -name '*.bzl' \) -print |
      xargs -t "$buildifier" -mode=fix -add_tables="$tables"
fi

echo "... done"
