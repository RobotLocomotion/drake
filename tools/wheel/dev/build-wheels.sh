#!/bin/bash

# This shell script uses Docker to build the PyPI wheel and automatically
# extracts the generated wheel to the current directory.

set -e

me=$(python3 -c 'import os; print(os.path.realpath("'"${BASH_SOURCE}"'"))')
mydir=$(dirname "$me")
drake_source_root=$(dirname $(dirname $(dirname ${mydir})))
[[ -f ${drake_source_root}/.drake-find_resource-sentinel ]]

salt=$(dd if=/dev/urandom bs=2 count=4 status=none | od -An -x | tr -d ' ')
id=pip-drake:$(date -u +%Y%m%d%H%M%S)-$salt

###############################################################################

images_to_remove=""
files_to_remove=""
at_exit()
{
    rm -f $files_to_remove > /dev/null 2>&1
    docker image rm $images_to_remove > /dev/null 2>&1
}
trap at_exit EXIT

build()
{
    local id=$1
    shift 1

    # Remove --force-rm if you need to inspect artifacts of a failed build
    images_to_remove="${images_to_remove} $id"
    docker build --force-rm --tag $id "$@" "$(dirname "${BASH_SOURCE}")"
}

extract()
{
    docker run --rm $1 \
        bash -c 'tar -cf - /wheel/wheelhouse/*.whl' | \
        tar --strip-components=2 -xf -
}

###############################################################################

# Snapshot the current Drake sources to feed to Docker.
files_to_remove=${mydir}/image/drake-src.tar.gz
(cd ${drake_source_root} &&
 git archive -o ${mydir}/image/drake-src.tar.gz HEAD)

# Build wheels
build ${id}-py36 --build-arg PYTHON=3.6 --build-arg PLATFORM=ubuntu:18.04
extract ${id}-py36

build ${id}-py37 --build-arg PYTHON=3.7 --build-arg PLATFORM=ubuntu:18.04
extract ${id}-py37

# TODO(mwoehlke-kitware) VTK needs a patch to build against Python 3.8
# build ${id}-py38 --build-arg PYTHON=3.8 --build-arg PLATFORM=ubuntu:18.04
# extract ${id}-py38

rm ${mydir}/image/drake-src.tar.gz
