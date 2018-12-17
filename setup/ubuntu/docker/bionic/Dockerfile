FROM ubuntu:bionic
WORKDIR /drake
COPY setup/ubuntu setup/ubuntu
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && yes | setup/ubuntu/install_prereqs.sh \
  && rm -rf /var/lib/apt/lists/*
COPY . .
RUN set -eux \
  && ./setup/select_python -f python3 \
  && bazel build //tools:drake_visualizer
ENTRYPOINT ["drake/setup/ubuntu/docker/entrypoint.sh"]
