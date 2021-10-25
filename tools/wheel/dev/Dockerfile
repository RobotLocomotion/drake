ARG PLATFORM=ubuntu:18.04

# -----------------------------------------------------------------------------
# Create a base provisioned image
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ARG PYTHON=3

ENV DEBIAN_FRONTEND=noninteractive

ADD image/provision-base.sh /image/

RUN /image/provision-base.sh

ADD image/provision-python.sh /image/

RUN /image/provision-python.sh ${PYTHON}

# -----------------------------------------------------------------------------
# Build Drake's dependencies
# -----------------------------------------------------------------------------

FROM base AS incubator

ADD image/dependencies/ /dependencies/src/
ADD image/build-dependencies.sh /image/

RUN /image/build-dependencies.sh

ADD image/build-vtk.sh /image/
ADD image/vtk-args /vtk/

RUN /image/build-vtk.sh

# -----------------------------------------------------------------------------
# Build the Drake wheel
# -----------------------------------------------------------------------------

FROM incubator

ENV REPO=https://api.github.com/repos/robotlocomotion/drake

ADD image/build-drake.sh /image/
ADD image/pip-drake.patch /image/
ADD ${REPO}/git/refs/heads/master /tmp/drake.sha

RUN /image/build-drake.sh

ADD image/build-wheel.sh /image/
ADD image/setup.py /wheel/

RUN /image/build-wheel.sh
