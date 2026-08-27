ARG PLATFORM=ubuntu:24.04

# -----------------------------------------------------------------------------
# Create a base provisioned image.
# -----------------------------------------------------------------------------

FROM ${PLATFORM} AS base

ARG PYTHON_MANAGER=pip

ADD provision-base.sh /image/

RUN /image/provision-base.sh ${PYTHON_MANAGER}

# -----------------------------------------------------------------------------
# Provision the requested Python version.
# -----------------------------------------------------------------------------

FROM base AS python

ARG PYTHON=3

ADD provision-python.sh /image/

RUN /image/provision-python.sh ${PYTHON} ${PYTHON_MANAGER}
