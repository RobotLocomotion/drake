#!/bin/bash
# cdexec is a command wrapper for adapting utilities like cmake or ./configure
# which expect to read or output files relative to their current working
# directory for running under Bazel.

usage() {
  cat <<EOF
usage: cdexec [-q|--quiet] <directory> <command> <command args ...>
       cdexec [-q|--quiet] -t|--temp TEMPLATE <command> <command args ...>

cdexec is a utility which changes current working directory, replaces all
occurrences of '\$\{PWD\}' from the command line with the previous PWD
and then execs that command line.
EOF
}

_cdexec() {
  local -r _ROOT="$PWD"  # Save PWD.
  local _QUIET=""        # Allow silencing stdout.
  local _DEST=""         # The directory to cd into.

  while true; do
    case "$1" in
      -q | --quiet)
        _QUIET="yes"
        shift
        ;;
      -t | --temp)
        shift
        _DEST="$(mktemp -d "$1")"
        shift
        ;;
      -*)
        echo "Unrecognized option: $1"
        usage
        exit 1
        ;;
      *)
        if [[ "$_DEST" == "" ]]; then
          _DEST="$1"
          shift
        fi
        break  # Exit at the first non-option argument.
        ;;
    esac
  done
  cd "$_DEST"
  # Exec the remaining arguments, replacing ${PWD} with the original root.
  # TODO(shahms): allow escaping the ${PWD}.
  if [ "$_QUIET" = "yes" ]; then
    exec "${@//\$\{PWD\}/${_ROOT}}" > /dev/null
  else
    exec "${@//\$\{PWD\}/${_ROOT}}"
  fi
}

_cdexec "$@"
