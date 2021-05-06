---
title: Using the Drake Docker Images From Docker Hub
---

Drake's continuous integration has nightly jobs that publish the binary
archives that are described in the [Nightly releases](/from_binary.html#nightly-releases) section. The Ubuntu
18.04 (Bionic) binary archive is also used to publish a Docker image on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake). You can see the
available Docker images here:

[https://hub.docker.com/r/robotlocomotion/drake/tags](https://hub.docker.com/r/robotlocomotion/drake/tags)

The [latest](https://hub.docker.com/r/robotlocomotion/drake/tags?name=latest)
image can be pulled from Docker Hub as follows:

```
docker pull robotlocomotion/drake:latest
```

Docker images based on the Ubuntu 20.04 (Focal) binary archive are not
available.
