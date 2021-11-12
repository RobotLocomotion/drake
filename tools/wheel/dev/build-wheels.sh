#!/bin/bash

# This shell script uses Docker to build the PyPI wheel and automatically
# extracts the generated wheel to the current directory.

set -e

###############################################################################

canonicalize()
{
    perl -e 'use Cwd;print Cwd::abs_path(shift) . "\n";' -- "$@"
}

cleanup()
{
    [ -f "$source_tar" ] && rm -f "$source_tar"
    if [ -n "$images_to_remove" ]; then
        docker image rm ${images_to_remove[@]}
    fi
}

build()
{
    local id=$1
    shift 1

    # Remove --force-rm if you need to inspect artifacts of a failed build.
    images_to_remove+=($id)
    docker build \
        --ssh default --force-rm --tag $id \
        --build-arg REPO_ARCHIVE=image/drake-src.tar.gz \
        "$@" "$(dirname "${BASH_SOURCE}")"
}

extract()
{
    docker run --rm $1 \
        bash -c 'tar -cf - /wheel/wheelhouse/*.whl' | \
        tar --strip-components=2 -xf -
}

###############################################################################

export DOCKER_BUILDKIT=1

trap cleanup EXIT

# Figure out our location and the repository root.
docker_root="$(canonicalize "$(dirname "${BASH_SOURCE}")")"
repo_root="$(cd $docker_root && git rev-parse --show-toplevel)"
[[ -f "$repo_root/.drake-find_resource-sentinel" ]]

# Snapshot the current Drake sources to feed to Docker.
source_tar="$docker_root/image/drake-src.tar.gz"
(cd "$repo_root" && git archive -o "$source_tar" HEAD)

# Generate a unique identifier for temporary tagging.
salt=$(dd if=/dev/urandom bs=2 count=4 status=none | od -An -x | tr -d ' ')
id=pip-drake:$(date -u +%Y%m%d%H%M%S)-$salt

images_to_remove=()

# Build wheels.
build ${id}-py36 --build-arg PYTHON=3.6 --build-arg PLATFORM=ubuntu:18.04
extract ${id}-py36

build ${id}-py37 --build-arg PYTHON=3.7 --build-arg PLATFORM=ubuntu:18.04
extract ${id}-py37

# TODO(mwoehlke-kitware) VTK 8.2 needs a patch to build against Python 3.8.
# build ${id}-py38 --build-arg PYTHON=3.8 --build-arg PLATFORM=ubuntu:18.04
# extract ${id}-py38
