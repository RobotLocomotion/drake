FROM ubuntu:16.04
RUN mkdir /drake
COPY . /drake
RUN apt-get update && yes "Y" \
      | /drake/setup/ubuntu/16.04/install_prereqs.sh \
      && rm -rf /var/lib/apt/lists/* \
      && apt-get clean all
RUN cd /drake && bazel build //tools:drake_visualizer //examples/acrobot:run_passive
ENTRYPOINT ["/drake/setup/docker/entrypoint.sh"]
