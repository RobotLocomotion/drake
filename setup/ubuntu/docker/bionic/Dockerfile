FROM ubuntu:bionic
WORKDIR /drake
COPY setup/ubuntu setup/ubuntu
RUN set -eux \
  && export DEBIAN_FRONTEND=noninteractive \
  && yes | setup/ubuntu/install_prereqs.sh \
  && rm -rf /var/lib/apt/lists/*
COPY . .
RUN set -eux \
  && bazel build //tools:drake_visualizer //examples/acrobot:run_passive
ENTRYPOINT ["drake/setup/ubuntu/docker/entrypoint.sh"]
