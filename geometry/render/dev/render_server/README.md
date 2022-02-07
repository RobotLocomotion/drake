# glTF Render Server API

This render server consumes a [glTF](https://www.khronos.org/registry/glTF/)
file and produces a rendered image in response.

## Basic Usage

### Installation

The server requirements are not currently installed via drake's `setup/` install
scripts.  To use the server you will need to install `flask` and `gunicorn`.

#### Ubuntu

Option 1: use the system package manager.

```console
# Install the prerequisites.
# NOTE: python3-gunicorn does not provide the `gunicorn` executable.
$ sudo apt-get install python3-flask gunicorn

# Verify that `gunicorn` is in your $PATH.
$ which gunicorn
/usr/bin/gunicorn
```

Option 2: create a virtual environment.

```console
# Install the prerequisites.
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install flask gunicorn

# Verify that `gunicorn` is in your $PATH.
$ which gunicorn
/path/to/venv/bin/gunicorn
```

#### macOS

Use `pip` or a virtual environment as shown in Option 2 for Ubuntu above
(`pip install flask gunicorn`).  Verify that `which gunicorn` produces output.

### Server Use

For development purposes, within the `render_server` directory you may run

```console
# A flask development server, which will only have one process
$ export FLASK_APP=gltf_render_server.py
$ flask run

# Alternatively, specify an alternative host or port.
$ flask run --host=x.y.z.w --port=8932

# Alternatively, use the default host=127.0.0.1 and port=8000.
$ python gltf_render_server.py
```

A single threaded worker flask server is not a good idea to run in production,
as it will not be able to handle concurrent requests.  You may use any number
of WSGI runners, in the example below we use [gunicorn](https://gunicorn.org/):

```console
$ gunicorn --workers 8 wsgi:app
```

This will spawn 8 workers to receive client communications.  This is what
`bazel run //geometry/render/dev:server` will achieve.  While developing your
server, you may desire to use the default flask runner rather than `gunicorn`
as you will get more logging output from the flask server runner.

## Server Architecture

The server expects to first receive an input glTF scene to be provided by the
client, and subsequently a request to render the provided scene.  The file
`gltf_render_server.py` provides an implementation of the below specification.
Users who desire to have their own render server backend may copy the file
and implement the `render_callback` method to invoke the desired renderer.
**See the documentation at the top of the file**.

# Server API

A given server implementation is required to implement an "Upload Endpoint" and
a "Render Endpoint".  The client will first upload a scene file to the upload
endpoint, and then subsequently request a rendering for the now-uploaded file.

**Your server is required to send a valid [HTTP response code][http_responses]
in both the event of success and failure.**  As a special note about `flask`
in particular, if you do not return the code yourself the default response is
always `200` which indicates success.  When in doubt, have your server respond
with `200` for a success, `400` for any failure related to bad requests (e.g.,
missing `min_depth` or `max_depth` for an `image_type="depth"`), and `500` for
any unhandled errors.  The HTTP response code (**only**) is what is used by the
client to determine if the interaction was successful.

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
client-server communication.

## Upload Endpoint

The upload endpoint (by default: `/upload`) receives an input scene file, as
well as form data for `image_type` to enable the server to store the file
appropriately.  The client will transmit the following fields:

- Field `data`:

    The scene file contents.  Sent as if from
    `<input type="file" name="data">`.  Sent as `multipart/form-data`, with a
    mime type of [`model/gltf+json`][gltf_mimetypes].

- Field `image_type`:

    The type of image being rendered.  Allowed values: `color`, `depth`,
    `label`.  Sent as form data `<input type="text" name="image_type">`.

A successful upload from the sample server will return a json response:
`{"error": "false", "code": "200", "sha256": "... computed sha256 hash..."}`.
The client will validate the returned sha256 sum to ensure the correct scene was
uploaded.  **The server is required to return a json value with at least the
key-value pair `{"sha256": "... computed sha256 hash..."}`.**  Failure to
respond with the sha256 hash of the uploaded scene will result in an error on
the client side.

[gltf_mimetypes]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#_media_type_registrations

## Render Endpoint

The render endpoint (by default: `/render`) is responsible for rendering and
transmitting an image back to the client.  The server is provided with all
information about a given drake sensor is transmitted to the render endpoint
in the form data.  It is worth mentioning that the
[glTF Projection Matrices][glTF_proj] in the glTF file sent by the client will
have the following attributes:

[glTF_proj]: https://www.khronos.org/registry/glTF/specs/2.0/glTF-2.0.html#projection-matrices

```json
{
    "cameras": [
        {
            "type": "perspective",
            "perspective": {
                "aspectRatio": 1.5,
                "yfov": 0.660593,
                "zfar": 100,
                "znear": 0.01
            }
        },
    ]
}
```

While this information is usually sufficient to re-construct a similar
projection matrix, the render endpoint is provided with all attributes from a
given sensor's `drake::systems::sensors::CameraInfo` object.  In addition, when
(_and only when_) the `image_type="depth"`, the associated
`drake::geometry::render::DepthRange`'s `min_depth` and `max_depth` will be
provided.  The server may decide which attributes to use at its discretion, the
client only requires that the rendered `width` and `height` are as requested.
The client will provide the server with the following information:

- Field `id`:

    The sha256 hash of a previously uploaded glTF scene to be rendered.  Sent as
    form data `<input type="text" name="id">`.

- Field `image_type`:

    The type of image being rendered.  Allowed values: `color`, `depth`,
    `label`.  Sent as form data `<input type="text" name="image_type">`.

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
