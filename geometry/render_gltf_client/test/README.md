# Developing your own Server
A [testing suite](../../render_gltf_client/test), including a sample
[flask](https://flask.palletsprojects.com/en/2.0.x/)
[server](../../render_gltf_client/test/server_demo.py) and a
[VTK server backend](../../render_gltf_client/test/server_vtk_backend.cc),
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
Meldis to observe what's happening.

### Run the Server
A single threaded / single worker flask development server on host `127.0.0.1`
and port `8000` can be launched by:

```
$ bazel run //geometry/render_gltf_client:server_demo
```

You may specify host and port by supplying extra arguments.

```
$ bazel run //geometry/render_gltf_client:server_demo -- --host 0.0.0.0 --port 8192
```

There is also a `--debug` option available to support reloading the server
automatically, see a dedicated section below for more information.

### Run the Client
In another terminal, run the test simulation and the client.

```
$ bazel run //geometry/render_gltf_client:client_demo
```
As a comparison, you can also render with the normal VTK by setting
--render_engine to vtk instead.

```
$ bazel run //geometry/render_gltf_client:client_demo -- --render_engine vtk
```

Note that if you ran your server on an alternate `--host` or `--port`, you will
need to specify that in `--server_base_url` when running `client_demo`
executable as well.

### (Optional) Run Meldis

In a separate terminal, launch Meldis.

```
$ bazel run //tools:meldis
```

## Testing of the client-server RPC pipeline
To ensure the RPC (Remote Procedure Call) infrastructure works as expected,
different layers of testing are in place.  Each file in
`//geometry/render_gltf_client` is well-covered with unit tests.  Additionally,
there are integration tests to exercise the entire pipeline and carefully
inspect the output.  It's advised to have a similar testing mechanism when
developing a new render server.

### Notes on Acceptance Test
Several example binaries are in the test folder to help exercise the pipeline.
In `acceptance_test.py`, each binary is invoked individually.  The goal of the
test is to check its high-level execution, e.g., not crashing or no import
errors.  These binaries, i.e., `server_demo` and `client_demo`, also help
demonstrate the pipeline that a Drake user can run through the command line.

### Notes on Integration Test
Furthermore, in `integration_test.py`, two end-to-end integration tests are
designed to quantitatively inspect the actual output, i.e., the rendered images
and the intermediate glTF file.

Two sets of color, depth, and label images are generated from `RenderEngineVtk`
and `RenderEngineGltfClient`.  The `RenderEngineGltfClient` engine connects to
a server (`server_demo.py`) that uses a VTK-backed rendering pipeline
(`server_vtk_backend.cc`).  Therefore, both render engines use VTK to render;
comparing their outputs directly provides evidence that the RPC infrastructure
is merely a communication tool between a client and a server but has negligible
impact on the image content.  Images produced by `RenderEngineVtk` are treated
as ground truth for pixel-by-pixel comparison.  However, it's inevitable to have
rounding errors due to subtle renderer settings; therefore, the test places a
reasonably small tolerance for the comparison.

In addition to rendering comparisons, there is another integration test focuses
on the glTF correctness.  For `RenderEngineGltfClient` to render an image, the
client converts geometries in `SceneGraph` to a glTF file and sends it over the
wire.  This test inspects the content of the intermediate glTF files.

Two good resources for understanding the glTF format:
[glTF-Tutorials](https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/README.md),
[glTF Properties Reference](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#properties-reference)

The test includes cached glTF files (`test_*_scene.gltf`) against which the
output is compared.  The comparison is not a *literal* byte-by-byte comparison
of the files' contents.  Two equivalent glTF files can appear quite different
simply by changing the ordering of elements.  This test transforms the glTF
files to eliminate this source of variability.  Furthermore, we omit some
portions of the glTF files to simplify the test in favor of focusing on the
specific expectations enumerated below.

<!-- TODO(SeanCurtis-TRI) The test needs to include a glTF file so we can
 confirm this remains true even with multiple glTFs being merged. -->
The glTF contains a single scene in the `scenes` entry (although glTF generally
allows for multiple scenes) and the `scene` value references that scene.

Each entry in `nodes` contains an added geometry in `SceneGraph`, a camera node,
or the node aggregating information pointed by `scenes`.  The `matrix` field
represents the global transformation to place the geometry/camera in the world
frame.  Note that the local camera coordinate systems of Drake and glTF are
different, so the y and z axis of the rotation matrix is flipped 180 degrees.

The extrinsics of the camera is specified in the `nodes` section.  However, the
intrinsics is placed in the `cameras` section.  Nominally, the file should have
only one camera node.

`meshes` is the crucial part of a glTF file and contains the necessary
information to specify the properties of a mesh.  Each mesh entry points to
the corresponding `accessors` entries with information, such as 3D vertex
positions, texture coordinates, and vertex indices.  It also specifies the
material property of the mesh by pointing to a `materials` entry.

The `materials` section includes the RGBA information of a mesh in the
`baseColorFactor` field.  If the material is textured, `baseColorTexture` will
be present with a number indexing to a `textures` entry.  `images` and
`samplers` are children to `textures`, and their content is fairly standard from
`vtkGLTFExporter`.

Each `buffers` entry represents a block of raw binary data, and `bufferViews`
and `accessors` provide hints on how to interpret a raw buffer to a meaningful
data structure.  As comparing the exact object texture is covered by the image
differencing test, only the `accessors` section is compared in the test.

#### Viewing the integration test outputs

There are two reasons why it would be necessary to see the outputs of the
integration test:

  - The test fails.
  - A meaningful change has been made in the underlying VTK library or the way
    we use that would change the contents of the glTF file.

If either of the tests fail, you can find the files that were assessed during
the test in:

```
bazel-testlogs/geometry/render_gltf_client/py/integration_test/test.outputs
```

#### Updating reference glTF files for the tests

If a meaningful change has been made to VTK or how Drake uses VTK, the reference
glTF files will probably no longer be valid.  In this case, the test will fail.
Manually inspect the test results (located in the path above), and confirm that
the generated glTF is what you would expect based on the underlying changes. If
so, you will derive the new reference glTF from the generated glTF file.

**Note** To avoid committing large glTF files, do not simply copy the generated
glTF file over the old reference file. Instead, be sure to delete the entire
`buffers` entry which contains texture and geometry data and bloats the file
size.  That field is not used at all in the glTF test.  It does mean you cannot
load the reference glTF in a glTF viewer, but you can always rely on the test's
output files for that.

## Prototyping your own Server
If everything is running as expected, then you can begin changing the
implementation of `render_callback` in `server_demo.py` to invoke the renderer
you desire.

### Notes on Developing with a `flask` Server
`server_demo.py` leverages `flask` to host a server.  During the development
phase, it can be helpful to add the `--debug` flag to trigger automatic server
reloading any time a file changes.

```
$ bazel run //geometry/render_gltf_client:server_demo -- --debug
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
