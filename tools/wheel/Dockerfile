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

ADD image/vtk-args /opt/drake-wheel-build/vtk/
ADD image/build-vtk.sh /image/

RUN /image/build-vtk.sh

# -----------------------------------------------------------------------------
# Install Python.
# -----------------------------------------------------------------------------

FROM incubator AS clean

ARG PYTHON=3

ADD image/provision-python.sh /image/

RUN /image/provision-python.sh ${PYTHON}

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

ENV DRAKE_VERSION=${DRAKE_VERSION}

RUN /image/build-wheel.sh
