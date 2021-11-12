#!/bin/bash

# This shell script uses Docker to build the PyPI wheel and automatically
# extracts the generated wheel to the current directory.

set -e

drake_source_root=$(cd "$(dirname "${BASH_SOURCE}")" &&
  git rev-parse --show-toplevel)

salt=$(dd if=/dev/urandom bs=2 count=4 status=none | od -An -x | tr -d ' ')
id=pip-drake:$(date -u +%Y%m%d%H%M%S)-$salt

export DOCKER_BUILDKIT=1

###############################################################################

images_to_remove=()
files_to_remove=()
at_exit()
{
    rm -f ${files_to_remove[*]}
    if [[ ${#images_to_remove[@]} -gt 0 ]]; then
        docker image rm ${images_to_remove[*]}
    fi
}
trap at_exit EXIT

build()
{
    local id=$1
    shift 1

    # Remove --force-rm if you need to inspect artifacts of a failed build.
    docker build \
        --ssh default --force-rm --tag $id \
        "$@" "$(dirname "${BASH_SOURCE}")"
    images_to_remove+=($id)
}

extract()
{
    docker run --rm $1 \
        bash -c 'tar -cf - /wheel/wheelhouse/*.whl' | \
        tar --strip-components=2 -xf -
}

###############################################################################

# Snapshot the current Drake sources to feed to Docker. Exclude the tools/wheel
# folder because it's unused by the Docker build and clogs up the caching.
# Refer to https://reproducible-builds.org/docs/archives/ for tips.
echo -n "Creating Drake source archive ... "
files_to_remove+=("${drake_source_root}"/tools/wheel/dev/image/drake-src.tar.gz)
(cd "${drake_source_root}" &&
 tar cfz tools/wheel/dev/image/drake-src.tar.gz \
     --mtime="2021-01-01 00:00Z" \
     --sort=name \
     --owner=0 --group=0 --numeric-owner \
     --format=gnu \
     --exclude-vcs \
     --exclude "bazel-*" \
     --exclude "tools/wheel" \
     .)
echo "done."

# Build all wheels.
build ${id}-py36 --build-arg PYTHON=3.6 --build-arg PLATFORM=ubuntu:18.04
extract ${id}-py36

build ${id}-py37 --build-arg PYTHON=3.7 --build-arg PLATFORM=ubuntu:18.04
extract ${id}-py37

# TODO(mwoehlke-kitware) VTK needs a patch to build against Python 3.8
# build ${id}-py38 --build-arg PYTHON=3.8 --build-arg PLATFORM=ubuntu:18.04
# extract ${id}-py38
