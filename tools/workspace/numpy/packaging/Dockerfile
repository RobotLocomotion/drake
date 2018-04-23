FROM ubuntu:16.04

RUN apt update -y \
    && apt install --no-install-recommends -q -y \
        build-essential git \
        python-dev \
        python-pip python-setuptools python-wheel python-virtualenv \
        cython gfortran liblapack-dev libblas-dev \
    && rm -rf /var/lib/apt/lists/*
