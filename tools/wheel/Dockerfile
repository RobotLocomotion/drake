ARG PLATFORM=ubuntu:20.04

# -----------------------------------------------------------------------------
# Create a base provisioned image.
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ENV DEBIAN_FRONTEND=noninteractive

ADD image/packages-* /image/
ADD image/provision-base.sh /image/
ADD image/known_hosts /root/.ssh/

RUN /image/provision-base.sh

# -----------------------------------------------------------------------------
# Build Drake's dependencies.
# -----------------------------------------------------------------------------

FROM base AS incubator

ADD image/dependencies/ /opt/drake-wheel-build/dependencies/src/
ADD image/build-dependencies.sh /image/

RUN /image/build-dependencies.sh

# NOTE: see image/build-vtk.sh which sources / uses these other files.
ADD image/vtk-common-cmake-args /image/
ADD image/vtk-cmake-args.sh /image/
ADD image/clone-vtk.sh /image/
ADD image/build-vtk.sh /image/

RUN /image/build-vtk.sh

# -----------------------------------------------------------------------------
# Install Python.
# -----------------------------------------------------------------------------

FROM incubator AS clean

ARG PYTHON=3
ARG PYTHON_SHA=

ADD image/provision-python.sh image/build-python.sh /image/

RUN /image/provision-python.sh ${PYTHON} ${PYTHON_SHA}

# -----------------------------------------------------------------------------
# Inject the primary build scripts.
# -----------------------------------------------------------------------------

ADD image/build-drake.sh /image/
ADD image/pip-drake.patch /image/
ADD image/drake-src.tar.xz /opt/drake-wheel-build/drake/

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

FROM clean AS wheel

ARG DRAKE_VERSION

RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/bazel \
    /image/build-drake.sh

ADD image/build-wheel.sh /image/
ADD image/setup.py /opt/drake-wheel-build/wheel/
ADD content /opt/drake-wheel-content

ENV DRAKE_VERSION=${DRAKE_VERSION}

RUN /image/build-wheel.sh
