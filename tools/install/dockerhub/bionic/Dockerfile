# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

FROM ubuntu:bionic
ADD drake-latest-bionic.tar.gz /opt
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy \
    $(cat /opt/drake/share/drake/setup/packages-bionic.txt) \
  && rm -rf /opt/drake/share/drake/setup /var/lib/apt/lists/* \
  && mv /opt/drake/bin/drake-visualizer /opt/drake/bin/drake-visualizer.image
RUN printf '#!/bin/bash\n \
echo "drake-visualizer is not supported on Docker containers.\n\
Although not guaranteed to run, the executable is available to try as-is:\n\
/opt/drake/bin/drake-visualizer.image"' > /opt/drake/bin/drake-visualizer\
  && chmod a+x /opt/drake/bin/drake-visualizer
ENV LD_LIBRARY_PATH="/opt/drake/lib:${LD_LIBRARY_PATH}" \
  PATH="/opt/drake/bin:${PATH}" \
  PYTHONPATH="/opt/drake/lib/python3.6/site-packages:${PYTHONPATH}"
