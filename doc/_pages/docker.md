---
title: Installation via Docker
---

# Docker Hub

<div class="warning" markdown="1">
Drake's published Docker images are no longer supported as of 2026-06-01.
The last stable release to receive updates was v1.53.0. This page serves as a
historical record for how to install Drake via published Docker images.

For users who want to run Drake in Docker, a sample `Dockerfile` is given
below that you can customize to fit your needs.
To learn about other installation methods, refer to
[Installation and Quickstart](/installation.html).
</div>

```dockerfile
FROM ubuntu:noble
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy \
    ca-certificates wget \
  && wget -O drake-noble.tar.gz https://github.com/RobotLocomotion/drake/releases/download/v1.53.0/drake-1.53.0-noble.tar.gz \
  && tar xvf drake-noble.tar.gz -C /opt \
  && /opt/drake/share/drake/setup/install_prereqs -y \
  # Bake model data into the image up-front, instead of fetching on demand.
  && PYTHONPATH=/opt/drake/lib/python3.12/site-packages python3 -c \
    'from pydrake.all import PackageMap; PackageMap().GetPath("drake_models")' \
ENV PATH="/opt/drake/bin:${PATH}" \
  PYTHONPATH="/opt/drake/lib/python3.12/site-packages:${PYTHONPATH}"
```

To learn about other installation methods, refer to
[Installation and Quickstart](/installation.html).

If you experience any problems with or have questions about Drake, please
[ask for help](/getting_help.html).

# Historical Notes

Drake published pre-compiled binaries as Docker images on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake). Refer to
[Supported Configurations](/installation.html#supported-configurations)
for additional compatibility details.

<div class="note" markdown="1">
Drake's Docker images do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions
in [Source Installation](/from_source.html).
</div>

Old stable images can be pulled from
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake)
as follows:

```
docker pull robotlocomotion/drake:X.Y.Z
```

Refer to [Release Notes](/release_notes/release_notes.html) for a list of
published X.Y.Z version numbers.

The Docker tags for Drake's stable releases are spelled like:

* ``noble-X.Y.Z`` for the Ubuntu 24.04 image of Drake vX.Y.Z.
* ``X.Y.Z`` is a synonym for ``noble-X.Y.Z``.

Old nightly images can be pulled from
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake)
as follows:

```
docker pull robotlocomotion/drake:latest
```

The Docker tags for Drake's nightly releases are spelled like:

* ``noble-YYYYMMDD`` for the Ubuntu 24.04 image of Drake as of date YYYY-MM-DD.
* ``noble`` is a synonym for the most recent ``noble-YYYYMMDD``.
* ``YYYYMMDD`` is a synonym for the most recent ``noble-YYYYMMDD``.
* ``latest`` is a synonym for the most recent ``YYYYMMDD``.

Refer to [Quickstart](/installation.html#quickstart) for next steps.
