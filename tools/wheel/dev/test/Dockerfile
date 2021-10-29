ARG PLATFORM=ubuntu:18.04

FROM ${PLATFORM}

ARG PYTHON=3

ADD provision.sh /image/

RUN /image/provision.sh ${PYTHON}
