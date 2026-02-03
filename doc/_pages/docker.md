---
title: Installation via Docker
---

# Docker Hub

<div class="warning" markdown="1">
**Drake's published Docker images are deprecated and will no longer receive
updates after 2026-06-01.**

For users who want to run Drake in Docker, you can fork the current Dockerfile
and customize it your needs. Or, can switch to a simple `pip install drake`
(or uv, etc.), or `apt install drake-dev`, etc.
</div>

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

<div class="note" markdown="1">
Drake's Docker images do not support the Gurobi solver. To use
Gurobi, you will need to build Drake from source following the instructions
in [Source Installation](/from_source.html).
</div>

## Stable Releases

<div class="warning" markdown="1">
**Drake's published Docker images are deprecated and will no longer receive
updates after 2026-06-01.**
</div>

The latest stable image can be pulled from
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake)
as follows:

```
docker pull robotlocomotion/drake:X.Y.Z
```

Refer to [Release Notes](/release_notes/release_notes.html) for a list of
published X.Y.Z version numbers.

The Docker tags for Drake's stable releases are spelled like:

* ``jammy-X.Y.Z`` for the Ubuntu 22.04 image of Drake vX.Y.Z.
* ``noble-X.Y.Z`` for the Ubuntu 24.04 image of Drake vX.Y.Z.
* ``X.Y.Z`` is a synonym for ``noble-X.Y.Z``.

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

<div class="warning" markdown="1">
**Drake's published Docker images are deprecated and will no longer receive
updates after 2026-06-01.**
</div>

The latest nightly image can be pulled from
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake)
as follows:

```
docker pull robotlocomotion/drake:latest
```

The Docker tags for Drake's nightly releases are spelled like:

* ``jammy-YYYYMMDD`` for the Ubuntu 22.04 image of Drake as of date YYYY-MM-DD.
* ``jammy`` is a synonym for the most recent ``jammy-YYYYMMDD``.
* ``noble-YYYYMMDD`` for the Ubuntu 24.04 image of Drake as of date YYYY-MM-DD.
* ``noble`` is a synonym for the most recent ``noble-YYYYMMDD``.
* ``YYYYMMDD`` is a synonym for the most recent ``noble-YYYYMMDD``.
* ``latest`` is a synonym for the most recent ``YYYYMMDD``.

Refer to [Quickstart](/installation.html#quickstart) for next steps.
