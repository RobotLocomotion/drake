ARG PLATFORM=ubuntu:24.04

# -----------------------------------------------------------------------------
# Create a base provisioned image.
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ADD provision-base.sh /image/

RUN /image/provision-base.sh

# -----------------------------------------------------------------------------
# Provision the requested Python version.
# -----------------------------------------------------------------------------

FROM base AS python

ARG PYTHON=3
ARG PYTHON_MANAGER=pip

ADD provision-python.sh /image/

RUN /image/provision-python.sh ${PYTHON} ${PYTHON_MANAGER}
