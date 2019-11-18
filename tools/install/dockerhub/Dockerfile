# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

# To create a Docker image similar to that hosted on Docker Hub
# (https://hub.docker.com/r/robotlocomotion/drake), download the latest binary
# package drake-latest-bionic.tar.gz (https://drake.mit.edu/from_binary.html)
# to this directory, and then run the following command:
#
#   docker-compose build
#
# To start a Docker container with an interactive pseudo-terminal, run the
# following command:
#
#   docker-compose run --rm drake bash
#
# The environment variables LD_LIBRARY, PATH, and PYTHONPATH are preset to
# suitable values for using Drake (and, in particular, its Python bindings).
#
# To run the test, an example from pydrake.examples, run the following command:
#
#   docker-compose -f docker-compose.test.yml up --exit-code-from sut

# Note that any files added to this directory that are not mentioned in
# .dockerignore will be uploaded to the Docker image prior to building, so
# please exercise caution when adding files.

FROM ubuntu:bionic
ADD drake-latest-bionic.tar.gz /opt
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy \
    $(cat /opt/drake/share/drake/setup/packages-bionic.txt | tr "\n" " ") \
  && rm -rf /opt/drake/share/drake/setup /var/lib/apt/lists/*
ENV LD_LIBRARY_PATH="/opt/drake/lib:$LD_LIBRARY_PATH" \
  PATH="/opt/drake/bin:$PATH" \
  PYTHONPATH="/opt/drake/lib/python3.6/site-packages:$PYTHONPATH"
