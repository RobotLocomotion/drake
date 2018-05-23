# Create a Docker container with development and runtime prerequisites for both
# binary and source distributions of Drake on Ubuntu 18.04 (Bionic). Note that
# this is currently an UNSUPPORTED platform.

FROM ubuntu:18.04
COPY . /tmp/drake-setup-ubuntu-18.04
ARG DEBIAN_FRONTEND=noninteractive
RUN yes | /tmp/drake-setup-ubuntu-18.04/install_prereqs.sh \
 && rm -rf /tmp/drake-setup-ubuntu-18.04 \
 && rm -rf /var/lib/apt/lists/*
