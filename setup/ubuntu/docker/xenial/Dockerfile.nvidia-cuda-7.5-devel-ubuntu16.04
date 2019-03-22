# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

FROM ubuntu:xenial
WORKDIR /drake
COPY setup/ubuntu setup/ubuntu
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install --no-install-recommends --yes \
       nvidia-384 \
       nvidia-cuda-gdb \
       nvidia-cuda-toolkit \
  && yes | setup/ubuntu/install_prereqs.sh \
  && rm -rf /var/lib/apt/lists/*
COPY . .
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=7.5"
ENTRYPOINT ["./setup/ubuntu/docker/entrypoint.sh"]
