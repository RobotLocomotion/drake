# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

FROM ubuntu:bionic
WORKDIR /drake
COPY setup/ubuntu setup/ubuntu
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install --no-install-recommends --yes \
       libglvnd-dev \
       nvidia-cuda-gdb \
       nvidia-cuda-toolkit \
       nvidia-driver-390 \
  && yes | setup/ubuntu/install_prereqs.sh \
  && rm -rf /var/lib/apt/lists/*
COPY . .
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=9.1"
ENTRYPOINT ["./setup/ubuntu/docker/entrypoint.sh"]
