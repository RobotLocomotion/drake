FROM ubuntu:16.04

RUN apt update -y \
    && apt install --no-install-recommends -q -y \
        build-essential git \
        make libssl-dev zlib1g-dev libbz2-dev libsqlite3-dev \
        python2.7 python2.7-dev \
        python-pip python-setuptools python-wheel python-virtualenv \
        cython gfortran liblapack-dev libblas-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean all
