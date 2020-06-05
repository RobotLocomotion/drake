# Docker Image for Docker Hub (Focal)

To create a Docker image similar to those hosted on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake), download the
latest [binary package](https://drake.mit.edu/from_binary.html)
`drake-latest-focal.tar.gz` to this directory, and then run the following
command:

```bash
docker build -t robotlocomotion/drake:focal .
```

To start a Docker container with an interactive pseudo-terminal, run the
following command:

```bash
docker run -it robotlocomotion/drake:focal
```

The environment variables `LD_LIBRARY`, `PATH`, and `PYTHONPATH` are preset to
suitable values for using Drake, including, in particular, its Python
bindings.
