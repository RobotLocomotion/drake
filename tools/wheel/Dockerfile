ARG PLATFORM=ubuntu:20.04

# -----------------------------------------------------------------------------
# Create a base provisioned image.
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ENV DEBIAN_FRONTEND=noninteractive

ADD image/packages-* /image/
ADD image/provision-base.sh /image/

RUN /image/provision-base.sh

# -----------------------------------------------------------------------------
# Build Drake's dependencies.
# -----------------------------------------------------------------------------

FROM base AS incubator

ADD image/dependencies/ /opt/drake-wheel-build/dependencies/src/
ADD image/build-dependencies.sh /image/

RUN /image/build-dependencies.sh

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
ADD image/drake-src.tar /opt/drake-wheel-build/drake/

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

FROM clean AS wheel

ARG DRAKE_VERSION
ARG DRAKE_GIT_SHA

ENV DRAKE_VERSION=${DRAKE_VERSION}
ENV DRAKE_GIT_SHA=${DRAKE_GIT_SHA}
ENV SNOPT_PATH=/opt/drake-wheel-build/dependencies/src/snopt.tar.gz

RUN --mount=type=cache,target=/var/cache/bazel \
    /image/build-drake.sh

ADD image/build-wheel.sh /image/
ADD image/setup.py /opt/drake-wheel-build/wheel/
ADD content /opt/drake-wheel-content

RUN /image/build-wheel.sh
