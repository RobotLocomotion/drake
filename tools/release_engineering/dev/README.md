# Docker Release Process

This describes the process of running the `push_docker.py` script to update
Drake's Dockerhub repository with images from a new release, by converting the
tags from the images pushed during the release staging jobs.

## Initial Setup

This process only needs to be done once per system.

Note that this script is only supported on our primary developer platform,
which is currently Ubuntu Noble 24.04.

### Install required packages

Install the maintainer-required prerequisites:

  setup/install_prereqs --with-maintainer-only

Follow instructions to install Docker
https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository

### Log into Docker

Log into Docker to ensure that you will be able to push the Docker images:

    docker login

The Docker ID and password may be found in the AWS Secrets Manager.

## Run script for Docker

Run the `push_docker` script as described below:

    bazel run //tools/release_engineering/dev:push_docker -- <version>

The release creator will provide the version. Don't use `v` on the version
string; for example:

    bazel run //tools/release_engineering/dev:push_docker -- 1.0.0

### Verification

Verify that
[Dockerhub](https://hub.docker.com/r/robotlocomotion/drake/tags?ordering=last_updated&page=1)
has a plain `<version>` tag as well as a `<version>` tag for each supported
configuration (e.g. noble).
