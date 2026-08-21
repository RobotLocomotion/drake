# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

FROM ubuntu:noble
ADD drake-latest-noble.tar.gz /opt
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy \
    $(cat /opt/drake/share/drake/setup/packages-noble.txt) \
# Make 'python' (not just 'python3') and pip available.
    python3-pip python-is-python3 \
  && rm -rf /var/lib/apt/lists/* \
# Bake model data into the image up-front, instead of fetching on demand.
  && PYTHONPATH=/opt/drake/lib/python3.12/site-packages python3 -c \
    'from pydrake.all import PackageMap; PackageMap().GetPath("drake_models")' \
ENV PATH="/opt/drake/bin:${PATH}" \
  PYTHONPATH="/opt/drake/lib/python3.12/site-packages:${PYTHONPATH}"
