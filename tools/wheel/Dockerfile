ARG PLATFORM=ubuntu:18.04

# -----------------------------------------------------------------------------
# Create a base provisioned image.
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ENV DEBIAN_FRONTEND=noninteractive

ADD image/provision-base.sh /image/
ADD image/known_hosts /root/.ssh/

RUN /image/provision-base.sh

# -----------------------------------------------------------------------------
# Build Drake's dependencies.
# -----------------------------------------------------------------------------

FROM base AS incubator

ADD image/dependencies/ /dependencies/src/
ADD image/build-dependencies.sh /image/

RUN /image/build-dependencies.sh

ADD image/build-vtk.sh /image/
ADD image/vtk-args /vtk/

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
ADD image/drake-src.tar.xz /drake/

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

FROM clean AS wheel

ARG DRAKE_VERSION

RUN --mount=type=ssh \
    --mount=type=cache,target=/var/cache/bazel \
    /image/build-drake.sh

ADD image/build-wheel.sh /image/
ADD image/setup.py /wheel/

ENV DRAKE_VERSION=${DRAKE_VERSION}

RUN /image/build-wheel.sh
