# glTF Render Server API
This render server consumes a [glTF](https://www.khronos.org/registry/glTF/)
file and produces a rendered image in response.

[TOC]

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
`bazel run //geometry/render/dev:server` will achieve.

## Server Architecture

The server expects to first receive an input glTF scene to be provided by the
client, and subsequently a request to render the provided scene.  The file
`gltf_render_server.py` provides an implementation of the below specification.
Users who desire to have their own render server backend may copy the file
and implement the `render_callback` method to invoke the desired renderer.
**See the documentation at the top of the file**.

**Warning**: currently it is expected that an RGBA `.png` file is returned
via `/render`.

# Server API

All methods use POST and expect `multipart/form-data`.  When errors occur, it
is expected that a json response containing
`{"error": "true", "code": "XYZ", "message": "Some error message."}` is
returned.

- `error`: a boolean flag, `true` or `false`.
- `code`: the HTTP response code.
- `message`: an indication of why there was an error for the client to log.

## `/upload`

The following fields are all required by the implemented server.

- Field `data`:

    The glTF contents.  Should be sent as if from
    `<input type="file" name="data">`.

- Field `image_type`:

    The type of image being rendered.  Allowed values: `color`, `depth`,
    `label`.  Should be sent as if from `<input type="text" name="image_type">`.

A successful upload will return a json response:
`{"error": "false", "code": "200", "sha256": "... computed sha256 hash..."}`.
The client will validate the returned sha256 sum to ensure the correct scene was
uploaded.

### cURL Client Example

```console
$ curl -X POST \
    -F 'data=@path/to/example.gltf' \
    -F 'image_type=color' \
    http://127.0.0.1:8000/upload
```

## `/render`

The following fields are all required by the implemented server.

- Field `id`:

    The sha256 hash of a previously uploaded glTF scene to be rendered.  Should
    be sent as if from `<input type="text" name="id">`.

- Field `image_type`:

    The type of image being rendered.  Allowed values: `color`, `depth`,
    `label`.  Should be sent as if from `<input type="text" name="image_type">`.

- Field `width`:

    Width of the desired rendered image.  Should be sent as if from
    `<input type="number" name="width">`.

- Field `height`:

    Height of the desired rendered image.  Should be sent as if from
    `<input type="number" name="height">`.

A successful render will transmit a PNG image response to the client.

### cURL Client Example

```console
$ curl -X POST \
    -F 'id=<sha>' \
    -F 'image_type=color' \
    -F 'width=640' \
    -F 'height=480' \
    http://127.0.0.1:8000/render
```
