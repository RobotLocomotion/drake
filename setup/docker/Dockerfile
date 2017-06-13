FROM ubuntu:16.04
RUN mkdir /drake-distro
COPY drake /drake-distro
RUN apt-get update && yes "Y" \
      | /drake-distro/setup/ubuntu/16.04/install_prereqs.sh \
      && rm -rf /var/lib/apt/lists/* \
      && apt-get clean all
RUN cd /drake-distro && bazel build ... && bazel test ...
ENTRYPOINT ["/drake-distro/setup/docker/entrypoint.sh"]
