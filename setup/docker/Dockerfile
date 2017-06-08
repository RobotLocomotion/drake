FROM ubuntu:16.04
RUN mkdir /drake-distro
COPY . /drake-distro
RUN apt-get update && yes "Y" \
      | /drake-distro/setup/ubuntu/16.04/install_prereqs.sh \
      && rm -rf /var/lib/apt/lists/* \
      && apt-get clean all
RUN mkdir /drake-distro/build && cd /drake-distro/build && cmake .. \
      && make -j16 && cd /drake-distro && bazel build ... && bazel test ...
COPY setup/docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
