# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

FROM ubuntu:focal
ADD drake-latest-focal.tar.gz /opt
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy \
    $(cat /opt/drake/share/drake/setup/packages-focal.txt) \
  && rm -rf /opt/drake/share/drake/setup /var/lib/apt/lists/*
ENV LD_LIBRARY_PATH="/opt/drake/lib:${LD_LIBRARY_PATH}" \
  PATH="/opt/drake/bin:${PATH}" \
  PYTHONPATH="/opt/drake/lib/python3.8/site-packages:${PYTHONPATH}"
