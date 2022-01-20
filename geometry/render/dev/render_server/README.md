# glTF Render Server API
This render server consumes a [glTF](https://www.khronos.org/registry/glTF/)
file and produces a rendered image in response.

## Basic Usage
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

All methods use POST and expect `multipart/form-data`.  When errors occur, it
is expected that a json response containing
`{"error": "true", "code": "XYZ", "message": "Some error message."}` is
returned.

- `error`: a boolean flag, `true` or `false`.
- `code`: the HTTP response code.
- `message`: an indication of why there was an error for the client to log.

It is assumed that the value in `code` in the json response, and the HTTP
response are the same.

## `/upload`

The `/upload` endpoint receives an input scene file, as well as form data for
`image_type` to enable the server to store the file appropriately.  The
client will transmit the following fields:

- Field `data`:

    The glTF contents.  Should be sent as if from
    `<input type="file" name="data">`.

- Field `image_type`:

    The type of image being rendered.  Allowed values: `color`, `depth`,
    `label`.  Sent as form data `<input type="text" name="image_type">`.

A successful upload will return a json response:
`{"error": "false", "code": "200", "sha256": "... computed sha256 hash..."}`.
The client will validate the returned sha256 sum to ensure the correct scene was
uploaded.  **The server is required to return a `sha256` hash by the client.**

## `/render`

The `/render` endpoint is responsible for rendering and transmitting an image
back to the client.  The server is provided with all information about a given
drake sensor in the form data transmitted to the `/render` endpoint by the
client.  It is worth mentioning that the [glTF Projection Matrices][glTF_proj]
in the glTF file sent by the client will have the following attributes:

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
projection matrix, the `/render` endpoint is provided with all attributes from
a given sensor's `drake::systems::sensors::CameraInfo` object.  In addition,
when (and only when) the `image_type="depth"`, the associated
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

- When `image_type=color`, the server may return:
    - An RGB (3 channel) unsigned char PNG image.
    - An RGBA (4 channel) unsigned char PNG image.

- When `image_type=depth`, the server may return:
    - A 16 bit or 32 bit single channel TIFF image.  The client will interpret
      this rendering as units of meters.
    - TODO(svenevs): A single channel unsigned short PNG image.  The client will
      interpret this rendering as units of millimeters and will convert to
      meters.

- When `image_type=label`, the server may return:
    - A single channel unsigned short PNG image.
