# This Dockerfile and the accompanying shell scripts and patch files are used
# by the project maintainers to create the precompiled vtk binaries that are
# downloaded during the build. They are neither called during the build nor
# expected to be called by most developers or users of the project.

ARG PLATFORM=ubuntu:20.04

# -----------------------------------------------------------------------------
# Create a base provisioned image.
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ADD image/provision.sh /image/
ADD image/prereqs /image/

# RUN /image/provision.sh

# -----------------------------------------------------------------------------
# Build VTK.
# -----------------------------------------------------------------------------

FROM base AS vtk

ADD image/build-vtk.sh /image/
ADD image/vtk-args /vtk/

RUN /image/build-vtk.sh

# ADD image/package.sh /image/

# RUN /image/package.sh
