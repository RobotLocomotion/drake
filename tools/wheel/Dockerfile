ARG PLATFORM=amd64/almalinux:9

# -----------------------------------------------------------------------------
# Create a base provisioned image.
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ENV DEBIAN_FRONTEND=noninteractive

ADD image/packages-* /image/
ADD image/provision-base.sh /image/

RUN /image/provision-base.sh

ADD image/provision-build.sh /image/
RUN /image/provision-build.sh

# -----------------------------------------------------------------------------
# Install Python.
# -----------------------------------------------------------------------------

FROM base AS clean

ARG PYTHON=3
ARG PYTHON_SHA=

ADD image/provision-python.sh image/build-python.sh /image/

RUN /image/provision-python.sh ${PYTHON} ${PYTHON_SHA}

# -----------------------------------------------------------------------------
# Inject the primary build scripts.
# -----------------------------------------------------------------------------

ADD image/build-drake.sh /image/
ADD image/drake-src.tar /tmp/drake-wheel-build/drake-src/
COPY image/snopt.tar.gz /tmp/drake-wheel-build/snopt/

# -----------------------------------------------------------------------------
# Build the Drake wheel.
# -----------------------------------------------------------------------------

FROM clean AS wheel

ARG DRAKE_VERSION
ARG DRAKE_GIT_SHA

ENV DRAKE_VERSION=${DRAKE_VERSION}
ENV DRAKE_GIT_SHA=${DRAKE_GIT_SHA}
ENV SNOPT_PATH=/tmp/drake-wheel-build/snopt/snopt.tar.gz

RUN --mount=type=cache,target=/var/cache/bazel \
    /image/build-drake.sh

ADD image/build-wheel.sh /image/
ADD image/setup.py /tmp/drake-wheel-build/drake-wheel/
ADD content /tmp/drake-wheel-build/drake-wheel-content
ADD licenses /tmp/drake-wheel-build/drake-wheel-licenses

RUN /image/build-wheel.sh
