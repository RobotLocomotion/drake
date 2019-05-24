# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

FROM ubuntu:xenial
WORKDIR /tmp
COPY setup/ubuntu setup/ubuntu
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update --quiet \
  && apt-get install --no-install-recommends --quiet --yes \
       passwd \
       sudo \
  && yes | ./setup/ubuntu/binary_distribution/install_prereqs.sh \
  && yes | ./setup/ubuntu/source_distribution/install_prereqs.sh \
  && rm -rf setup /var/lib/apt/lists/* \
  && useradd --create-home --groups sudo --shell /bin/bash ubuntu \
  && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
  && mkdir --parents /home/ubuntu/drake \
  && chown --recursive ubuntu:ubuntu /home/ubuntu/drake
USER ubuntu
WORKDIR /home/ubuntu/drake
COPY --chown=ubuntu:ubuntu . .
RUN ./setup/ubuntu/source_distribution/install_prereqs_user_environment.sh
ENTRYPOINT ["./setup/ubuntu/docker/entrypoint.sh"]
