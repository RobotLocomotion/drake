# Docker Image for Binder

*Note that due to Binder conventions, this directory MUST always be in the root
of the repository and named either `binder` or `.binder`. This image is NOT
intended for use by most developers or users. Please use the
[robotlocomotion/drake](https://hub.docker.com/r/robotlocomotion/drake) image
from Docker Hub instead.*

To create a Docker image and run a Docker container similar to those used by
[Binder](https://mybinder.org) for local debugging purposes, execute the
following `pull`, `build`, and `run` commands from the top level of the Drake
Git repository:

```bash
docker pull robotlocomotion/drake:latest
docker build -f .binder/Dockerfile -t binder .
docker run --rm -it --name mybinder -p 8888:8888 binder
```

The `meshcat-visualizer` is NOT supported by Binder since port 7000 is not
exposed, but to allow its use locally, substitute the following `run` command
for the above:

```bash
docker run --rm -it --name mybinder -p 7000:7000 -p 8888:8888 binder
```

Copy and paste the URL (including the login token) that is displayed in the
terminal into the web browser of your choice.

To stop the running container, simply exit it from the terminal with Ctrl+C.
