# Developing your own Server
A [testing suite](../../render_gltf_client/test), including a sample
[flask](https://flask.palletsprojects.com/en/2.0.x/)
[server](../../render_gltf_client/test/gltf_render_server.py) and a
[VTK server backend](../../render_gltf_client/test/gltf_render_server_backend.cc),
is provided as a working example that can be used with minimal change for your
development.

## Install Dependencies

The test server uses `flask(>=1.1)`, so be sure that you've run
`setup/ubuntu/install_prereqs.sh` (or `setup/mac/install_prereqs.sh`) to install
it before proceeding.

**Note:** `flask` is a "test-only" requirement, so don't provide
`--without-test-only` flag when running the `install_prereqs.sh` script.

## Run the Client-Server Testing Suite
There are three commands to run to launch the client, the server, and optionally
Drake Visualizer for displaying rendered images.

### Run the Server
A single threaded / single worker flask development server on host `127.0.0.1`
and port `8000` can be launched by:

```
$ bazel run //geometry/render_gltf_client:gltf_render_server
```

You may specify host and port by supplying extra arguments.

```
$ bazel run //geometry/render_gltf_client:gltf_render_server -- --host 0.0.0.0 --port 8192
```

There is also a `--debug` option available to support reloading the server
automatically, see the documentation in `gltf_render_server.py` for more
information on flask debug targets.

### Run the Client
In another terminal, run the test simulation and the client.

```
$ bazel run //geometry/render/dev/render_gltf_client:run_simulation_and_render -- --render_engine client
```
As a comparison, you can also render with the normal VTK by setting
--render_engine to vtk instead.

```
$ bazel run //geometry/render/dev/render_gltf_client:run_simulation_and_render -- --render_engine vtk
```

Note that if you ran your server on an alternate `--host` or `--port`, you will
need to specify that in `--server_base_url` when running
`run_simulation_and_render` executable as well.

### (Optional) Run Drake Visualizer

In a separate terminal, launch drake_visualizer.

```
$ bazel run //tools:drake_visualizer
```

## Prototyping your own Server
If everything is running as expected, then you can begin changing the
implementation of `render_callback` in `gltf_render_server.py` to invoke the
renderer you desire.

### Notes on Developing with a `flask` Server
`gltf_render_server.py` leverages `flask` to host a server.  During the
development phase, it can be helpful to add the `--debug` flag to trigger
automatic server reloading any time a file changes.

```
$ bazel run //geometry/render_gltf_client:gltf_render_server -- --debug
```

See also the documentation on the `FLASK_ENV` environment variable.  Its default
value is `production`, but setting it to `development` can be very helpful for
iterative development.

This server does not use any additional flask plugins, so the `--debug` flag
will work reliably.  However, the environment variable is to be set **before**
launching the server, and any extensions you may be using could have already
been initialized before `app.run(..., debug=True)`. Thus, you may get unreliable
behavior and should instead prefer using the environment variable.  Examples can
be found in the flask documentation
[flask_env](https://flask.palletsprojects.com/en/2.0.x/config/#environment-and-debug-features).

Lastly, the server implemented here provides a brief html page rendering when a
user issues a `GET /` (e.g., you visit `localhost:8000` in your browser).  When
`FLASK_ENV=development`, this server will include the path to the temp folder so
you may easily navigate there for debugging purposes.  Information about the
server directory structure should not be revealed in production.

### Notes on Deploying your Server

A single threaded worker flask server is not a good idea to run in production,
as it will not be able to handle concurrent requests.  You may use any number
of WSGI runners, in the example below we use [gunicorn](https://gunicorn.org/):

```
$ gunicorn --timeout 0 --workers 8 --bind 127.0.0.1:8000 wsgi:app
```

This will spawn a server with:

- No timeout.  If your rendering is going to take any sizable amount of time,
  by default `gunicorn` will kill and restart workers that have gone silent.
  Setting the `timeout` to `0` disables this behavior, at the expense of never
  being able to kill a worker that truly has been stuck.  If you know your
  rendering backend will take a maximum time, choose that instead.
- 8 workers to receive client communications.  This means that 8 requests can
  be processed at once.  Depending on your server capabilities and expected
  demand, adjust the number of workers accordingly.  **Note** that in the API
  the worker does not respond until the rendering is complete -- so if 8 workers
  are being used actively, the ninth request will not be able to be handled
  until all previous requests are finished.
  [See the discussion here](https://github.com/benoitc/gunicorn/issues/1492#issuecomment-294436705)
  for more information about _request timeouts_ as they pertain to `gunicorn` --
  the amount of requests able to wait in the queue of requests to be processed
  while all the workers are tied up depend entirely on your server backend and
  architecture.
- `--bind` to the URL and (optional) port.  What your final choice will be
  depends on your configurations for the tool you are using to expose your
  server (e.g., nginx or apache).

For the last part, `wsgi:app`, this is assuming you have a file named `wsgi.py`
in the same directory as where you are running the command `gunicorn` with the
contents:

```
# NOTE: you may need to update sys.path to import your server implementation
# depending on your directory structure.
from gltf_render_server import app

if __name__ == "__main__":
    app.run()
```

**Note:** On Ubuntu, to make the `gunicorn` executable available you will want
to `sudo apt-get install gunicorn` (not `python3-gunicorn`).  If using a virtual
environment or `pip` directly, `pip install gunicorn` will make the `gunicorn`
executable available.

There are some global variables in the current server implementation, e.g.,
`RENDER_ENDPOINT`.  This will only work in the single threaded development
server provided by flask, but will not be coherent when using a wsgi wrapper
such as `gunicorn`.  If your needs require shared state between server
processes, you will want to use a proper database or implement a scheme using
multiprocessing.
