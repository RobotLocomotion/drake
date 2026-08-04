ARG PLATFORM=ubuntu:24.04

FROM ${PLATFORM}

ARG PYTHON=3
ARG PYTHON_MANAGER=pip

ADD provision.sh /image/

RUN /image/provision.sh ${PYTHON} ${PYTHON_MANAGER}
