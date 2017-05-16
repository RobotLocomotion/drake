#!/bin/bash

set -e

me=$(basename "$0")

function die() {
    set +x
    echo "$@" 1>&2
    find -L . -name config.log | xargs -n1 --verbose cat 1>&2
    exit 1
}

# Log the current environment varibles.
echo "*******************************************************************" 1>&2
echo "$me: Initial environment is ..." 1>&2
env | sort 1>&2
echo "... end of environment." 1>&2
echo "*******************************************************************" 1>&2

# Check if everything we need exists.
[ -x "$cdexec" ] || die 'Missing $cdexec'
[ -n "$top_builddir" ] || die 'Missing $top_builddir'
configure="$PWD"/external/ipopt/configure
[ -x "$configure" ] || die 'Missing configure'

# How many cores to use while building.  This is a compromise in order to not
# totally bog down the machine, while still achieving some parallelism.
half_the_cores="$[($(getconf _NPROCESSORS_ONLN)+1)/2]"

# Set these to match --with-pic below.
export ADD_CFLAGS=-fPIC
export ADD_CXXFLAGS=-fPIC

# Log the current environment varibles.
echo "*******************************************************************" 1>&2
echo "$me: Final environment is ..." 1>&2
env | sort 1>&2
echo "... end of environment." 1>&2
echo "*******************************************************************" 1>&2

# From now on, echo commands before running them.
set -x

"$cdexec" "$top_builddir" "$configure" --disable-shared --with-pic \
    || die "*** IPOPT configure step failed ***"

"$cdexec" "$top_builddir" make -j "$half_the_cores" V=1 \
    || die "*** IPOPT make step failed ***"

"$cdexec" "$top_builddir" make install \
    || die "*** IPOPT install step failed ***"
