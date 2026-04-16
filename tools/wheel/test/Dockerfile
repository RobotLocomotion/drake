ARG PLATFORM=ubuntu:24.04

FROM ${PLATFORM}

ARG PYTHON=3
ARG UV=

ADD provision.sh /image/

RUN /image/provision.sh ${PYTHON} ${UV}
