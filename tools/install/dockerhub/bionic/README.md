# Docker Image for Docker Hub (Bionic)

To create a Docker image similar to those hosted on
[Docker Hub](https://hub.docker.com/r/robotlocomotion/drake), download the
latest [binary package](https://drake.mit.edu/from_binary.html)
`drake-latest-bionic.tar.gz` to this directory, and then run the following
command:

```bash
docker build -t robotlocomotion/drake:bionic -t robotlocomotion/drake:latest .
```

To start a Docker container with an interactive pseudo-terminal, run the
following command:

```bash
docker run -it robotlocomotion/drake:latest
```

The environment variables `LD_LIBRARY`, `PATH`, and `PYTHONPATH` are preset to
suitable values for using Drake, including, in particular, its Python
bindings.
