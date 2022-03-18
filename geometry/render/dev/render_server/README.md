# glTF Render Server API

This render server consumes a [glTF][glTF] file and produces a rendered image in
response.

[glTF]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html

## Basic Usage

### Installation

The server requirements are not currently installed via drake's `setup/` install
scripts.  To use the server you will need to install `flask`.

#### Ubuntu

Option 1: use the system package manager.

```console
# Install the prerequisites.
$ sudo apt-get install python3-flask
```

Option 2: create a virtual environment.

```console
# Install the prerequisites.
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install flask
```

#### macOS

Use `pip` or a virtual environment as shown in Option 2 for Ubuntu above
(`pip install flask`).

### Server Use

For development purposes, you may run
`bazel run //geometry/render/dev:gltf_render_server` to launch a single threaded
/ single worker flask development server.  By default this will run the server
on host `127.0.0.1` and port `8000`, you may specify `--host x.y.z.w` or
`--port XYZW` to change the host or port.  There is also a `--debug` option
available to support reloading the server automatically, see the documentation
in `gltf_render_server.py` for more information on flask debug targets.

```console
# Run on the default host and port.
$ bazel run //geometry/render/dev:gltf_render_server

# Run on custom host and port.
$ bazel run //geometry/render/dev:gltf_render_server -- --host 0.0.0.0 --port 8192
```

A single threaded worker flask server is not a good idea to run in production,
as it will not be able to handle concurrent requests.  You may use any number
of WSGI runners, in the example below we use [gunicorn](https://gunicorn.org/):

```console
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
  [See the discussion here][gunicorn_request_timeouts] for more information
  about _request timeouts_ as they pertain to `gunicorn` -- the amount of
  requests able to wait in the queue of requests to be processed while all the
  workers are tied up depend entirely on your server backend and architecture.
- `--bind` to the URL and (optional) port.  What your final choice will be
  depends on your configurations for the tool you are using to expose your
  server (e.g., nginx or apache).

[gunicorn_request_timeouts]: https://github.com/benoitc/gunicorn/issues/1492#issuecomment-294436705

For the last part, `wsgi:app`, this is assuming you have a file named `wsgi.py`
in the same directory as where you are running the command `gunicorn` with the
contents:

```py
# NOTE: you may need to update sys.path to import your server implementation
# depending on your directory structure.
from gltf_render_server import app

if __name__ == "__main__":
    app.run()
```

**Tip**: on Ubuntu, to make the `gunicorn` executable available you will want
to `sudo apt-get install gunicorn` (not `python3-gunicorn`).  If using a virtual
environment or `pip` directly, `pip install gunicorn` will make the `gunicorn`
executable available.

## Server Architecture

The server expects to first receive an input glTF scene to be provided by the
client, and subsequently a request to render the provided scene.  The file
`gltf_render_server.py` provides an implementation of the below specification.
Users who desire to have their own render server backend may copy the file
and implement the `render_callback` method to invoke the desired renderer.
**See the documentation at the top of the file**.

# Server API

A given server implementation is required to implement a "Render Endpoint" to
which the client will issue a [`POST` request][html_post] with an
[html `<form>`][html_form] and then await a rendered image response.  The `POST`
from the client will include uploading the scene file being uploaded, in
addition to a variety of other metadata attributes related to the image being
rendered (such as `width` and `height`), as well as the full specification of
the drake sensor intrinsics being rendered.

The server is **expected to block** (delay sending a response) until it is ready
to transmit the final rendered image back to the client.  This provides for an
easier implementation of the server, as well as this single endpoint is the only
form of communication the client will initiate with the server.  If the server
fails to respond with a valid image file, the client will produce an error and
halt the simulation being rendered.

**Your server is required to send a valid [HTTP response code][http_responses]
in both the event of success and failure.**  As a special note about `flask`
in particular, if you do not return the code yourself the default response is
always `200` which indicates success.  When in doubt, have your server respond
with `200` for a success, `400` for any failure related to bad requests (e.g.,
missing `min_depth` or `max_depth` for an `image_type="depth"`), and `500` for
any unhandled errors.  The HTTP response code (**only**) is what is used by the
client to determine if the interaction was successful.

[html_form]: https://developer.mozilla.org/en-US/docs/Web/HTML/Element/form
[http_responses]: https://developer.mozilla.org/en-US/docs/Web/HTTP/Status

## Additional Notes on Communicating Errors

When errors occur, the sample server implements a json response containing:
`{"error": "true", "code": "XYZ", "message": "Some error message."}`.  When an
error occurs on the server side, sending a `json` response is helpful for
indicating _why_ there was an upload or render failure.  When provided, this
information will be included in the exception message produced by the
`RenderClient`.  In the sample server, the values here represent:

- `error`: a boolean flag, `true` or `false`.
- `code`: the HTTP response code.
- `message`: an indication of why there was an error for the client to log.

At the very least, a custom server should return
`{"message": "Why there was a failure"}`.  Though this is not strictly required,
the user of the server will have no hints as to what is going wrong with the
client-server communication without some kind of response from the server.

## Render Endpoint

The render endpoint (by default: `/render`) is responsible for receiving an
uploaded scene file, rendering the scene, and transmitting an image back to the
client.  In addition to the scene file, the render endpoint is provided with the
full specification of the drake sensor's `drake::systems::sensors::CameraInfo`
object.  Depending on the choice of renderer, an application may desire to
construct its own projection matrix to match what would be utilized from within
drake.  Refer to the _implementation_ of
`RenderCameraCore::CalcProjectionMatrix` for how to use the form data to
construct the drake projection matrix.

### glTF Camera Specification

When the uploaded scene file is a [glTF scene][glTF], note that there are two
locations that describe the camera:

1. The [`"cameras"` array][glTF_cameras], which specifies the camera projection
   matrix.  The client will always produce a length one `"cameras"` array, with
   a single entry ("camera 0").  This camera will always be of
   `"type": "perspective"`, and its `"aspectRatio"`, `"yfov"`, `"zfar"`, and
   `"znear"` attributes will accurately represent the drake sensor.  However,
   note that the [glTF perspective projection definition][glTF_proj] does not
   include all of the information present in the matrix that would be obtained
   by `RenderCameraCore::CalcProjectionMatrix`.  While the two matrices will be
   similar, a given render server must decide based off its choice of render
   backend how it wishes to model the camera perspective projection
   transformation -- utilize the glTF definition, or incorporate the remainder
   of the `<form>` data to construct its own projection matrix.  A sample
   snippet from a client glTF file:

    ```json
    {
      "cameras" :
      [
        {
          "perspective" :
          {
            "aspectRatio" : 1.3333333333333333,
            "yfov" : 0.78539816339744828,
            "zfar" : 10,
            "znear" : 0.01
         },
         "type" : "perspective"
        }
      ],
    }
    ```

2. The [`"nodes"` array][glTF_nodes], which specifies the camera's global
   transformation matrix.  The `"camera": 0` entry refers to the index into the
   `"cameras"` array from (1).  Note that this is **not** the "model view
   transformation" (rather, its inverse) -- it is the camera's global
   transformation matrix which places the camera in the world just like
   any other entry in `"nodes"`.  Note that the `"matrix"` is presented in
   column-major order, as prescribed by the glTF specification.  A sample
   snippet of the camera node specification in the `"nodes"` array that has
   no rotation (the identity rotation) and a translation vector
   `[x=0.1, y=0.2, z=0.3]` would be provided as:

    ```json
    {
      "nodes" :
      [
        {
          "camera" : 0,
          "matrix" :
          [
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.1,
            0.2,
            0.3,
            1.0,
          ],
          "name" : "Camera Node"
        },
      ],
    }
    ```

[glTF_cameras]:https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-camera
[glTF_nodes]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#reference-node
[glTF_proj]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#projection-matrices

### Render Endpoint `<form>` Data

The client will `POST` a `<form>` with an `enctype=multipart/form-data` to the
server with the following field entries:

- Field `scene`:

    The scene file contents.  Sent as if from
    `<input type="file" name="scene">`.  When applicable the relevant mime type
    of the file will be provided.

     - glTF scenes will have a mime type of [`model/gltf+json`][gltf_mimetypes].

- Field `scene_sha256`:

    The sha256 hash of the `scene` file being uploaded.  The server may use this
    entry to validate that the full file was successfully uploaded.  Sent as
    form data `<input type="text" name="scene_sha256">`.

- Field `image_type`:

    The type of image being rendered.  Its value will be one of: `"color"`,
    `"depth"`, or `"label"`.  Sent as form data
    `<input type="text" name="image_type">`.

- Field `min_depth`:

    The minimum depth range as specified by a depth sensor's
    `drake::geometry::render::DepthRange::min_depth()`.  **Only provided when
    `image_type="depth"`.**  Sent as form data
    `<input type="number" name="min_depth">`.  Decimal value.

- Field `max_depth`:

    The maximum depth range as specified by a depth sensor's
    `drake::geometry::render::DepthRange::max_depth()`.  **Only provided when
    `image_type="depth"`.**  Sent as form data
    `<input type="number" name="max_depth">`.  Decimal value.

- Field `width`:

    Width of the desired rendered image in pixels.  Sent as form data
    `<input type="number" name="width">`.  Integral value.

- Field `height`:

    Height of the desired rendered image in pixels.  Sent as form data
    `<input type="number" name="height">`.  Integral value.

- Field `near`:

    The near clipping plane of the camera.  Sent as form data
    `<input type="number" name="near">`.  Decimal value.

- Field `far`:

    The far clipping plane of the camera.  Sent as form data
    `<input type="number" name="far">`.  Decimal value.

- Field `focal_x`:

    The focal length x, in pixels.  Sent as form data
    `<input type="number" name="focal_x">`.  Decimal value.

- Field `focal_y`:

    The focal length y, in pixels.  Sent as form data
    `<input type="number" name="focal_y">`.  Decimal value.

- Field `fov_x`:

    The field of view in the x-direction (in radians).  Sent as form data
    `<input type="number" name="fov_x">`.  Decimal value.

- Field `fov_y`:

    The field of view in the y-direction (in radians).  Sent as form data
    `<input type="number" name="fov_y">`.  Decimal value.

- Field `center_x`:

    The principal point's x coordinate in pixels.  Sent as form data
    `<input type="number" name="center_x">`.  Decimal value.

- Field `center_y`:

    The principal point's y coordinate in pixels.  Sent as form data
    `<input type="number" name="center_y">`.  Decimal value.

[gltf_mimetypes]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#_media_type_registrations

### Allowed Image Response Types from the Server

The client accepts the following image types from a server render:

- When `image_type="color"`, the server may return:
    - An RGB (3 channel) unsigned char PNG image.
    - An RGBA (4 channel) unsigned char PNG image.

- When `image_type="depth"`, the server may return:
    - A 16 bit or 32 bit single channel TIFF image.  The client will interpret
      this rendering as units of meters.
    - TODO(svenevs): A single channel unsigned short PNG image.  The client will
      interpret this rendering as units of millimeters and will convert to
      meters.

- When `image_type="label"`, the server may return:
    - A single channel unsigned short PNG image.
