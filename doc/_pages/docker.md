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

If you experience any problems or questions with Drake, please
[ask for help on Stack Overflow](/getting_help.html).

Drake binary releases incorporate a pre-compiled version of
[SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/) as part of the
[Mathematical Program toolbox](https://drake.mit.edu/doxygen_cxx/group__solvers.html).
Thanks to Philip E. Gill and Elizabeth Wong for their kind support.

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

* ``bionic-X.Y.Z`` for the Ubuntu 18.04 image of Drake vX.Y.Z.
* ``focal-X.Y.Z`` for the Ubuntu 20.04 image of Drake vX.Y.Z.
* ``X.Y.Z`` is a synonym for one of the above, currently bionic-X.Y.Z.

Refer to [Quickstart](/installation.html#quickstart) for next steps.

## Nightly Releases

The latest nightly image can be pulled from
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake)
as follows:

```
docker pull robotlocomotion/drake:latest
```

The docker tags for Drake's nightly releases are spelled like:

* ``bionic-YYYYMMDD`` for the Ubuntu 18.04 image of Drake as of date YYYY-MM-DD.
* ``bionic`` is a synonym for the most recent ``bionic-YYYYMMDD``.
* ``focal-YYYYMMDD`` for the Ubuntu 20.04 image of Drake as of date YYYY-MM-DD.
* ``focal`` is a synonym for the most recent ``focal-YYYYMMDD``.
* ``YYYYMMDD`` is a synonym for one of the above, currently ``bionic-YYYYMMDD``.
* ``latest`` is a synonym for the most recent ``YYYYMMDD``.

Refer to [Quickstart](/installation.html#quickstart) for next steps.
