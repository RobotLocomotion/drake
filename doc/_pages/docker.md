---
title: Installation via Docker
---

# Docker Hub

Drake publishes pre-compiled binaries as Docker images on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake). Refer to
[Supported Configurations](/installation.html#supported-configurations)
for additional compatibility details.

To learn about other installation methods, refer to
[Installation and Quickstart](/installation.html).

If you experience any problems with or have questions about Drake, please
[ask for help](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

Drake's docker images do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions
in [Source Installation](/from_source.html).

## Stable Releases

The latest stable image can be pulled from
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake)
as follows:

```
docker pull robotlocomotion/drake:X.Y.Z
```

Refer to [Release Notes](/release_notes/release_notes.html) for a list of
published X.Y.Z version numbers.

The docker tags for Drake's stable releases are spelled like:

* ``jammy-X.Y.Z`` for the Ubuntu 22.04 image of Drake vX.Y.Z.
* ``X.Y.Z`` is a synonym for ``jammy-X.Y.Z``.

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

The latest nightly image can be pulled from
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake)
as follows:

```
docker pull robotlocomotion/drake:latest
```

The docker tags for Drake's nightly releases are spelled like:

* ``jammy-YYYYMMDD`` for the Ubuntu 22.04 image of Drake as of date YYYY-MM-DD.
* ``jammy`` is a synonym for the most recent ``jammy-YYYYMMDD``.
* ``YYYYMMDD`` is a synonym for the most recent ``jammy-YYYYMMDD``.
* ``latest`` is a synonym for the most recent ``YYYYMMDD``.

Refer to [Quickstart](/installation.html#quickstart) for next steps.
