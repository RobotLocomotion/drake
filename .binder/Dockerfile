# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

# TODO(jamiesnape): Use a different tag. Per the Binder documentation, a tag
# named "latest" is not allowed, however they do not (yet) enforce that
# restriction.
FROM robotlocomotion/drake:latest
ARG NB_USER=jovyan
ARG NB_UID=1000
ARG NB_GID=100
EXPOSE 7000/tcp
EXPOSE 8888/tcp
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -qq \
    -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confnew \
    -o Dpkg::Use-Pty=0 \
      locales \
      xvfb \
  && rm -rf /var/lib/apt/lists/* \
  && locale-gen en_US.UTF-8 \
  && useradd -d "/home/$NB_USER" -G $NB_GID -mU -s /bin/bash "$NB_USER"
ENV HOME="/home/$NB_USER" \
  LANG=en_US.UTF-8 \
  LANGUAGE=en_US.UTF-8 \
  LC_ALL=en_US.UTF-8 \
  SHELL=/bin/bash \
  USER="$NB_USER"
WORKDIR $HOME
COPY [".binder/binder-entrypoint", "/opt/drake/bin/binder-entrypoint"]
COPY ["tutorials/*.ipynb", "$HOME/tutorials/"]
# TODO(jamiesnape): Use COPY --chown $NB_UID:$NB_GID instead of adding an extra
# layer when all Binder environments are built with Docker 19.03 or above.
RUN chown -R $NB_UID:$NB_GID \
  /opt/drake/bin/binder-entrypoint \
  "$HOME/tutorials"
USER "$NB_USER"
ENTRYPOINT ["/opt/drake/bin/binder-entrypoint"]
CMD ["jupyter", "notebook", "--ip", "0.0.0.0"]
