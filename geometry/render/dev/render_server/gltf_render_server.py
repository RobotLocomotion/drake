"""
A prototype glTF render server.  Users seeking to utilize their own renderer
as a replacement for the sample VTK based C++ executable can simply change the
implementation in the ``render_callback`` method.

The dependent package requirements for running this server::

    flask>=1.1
    gunicorn

.. note::

    If using ``apt-get`` to manage your python packages, you will want to
    ``apt-get install gunicorn`` (so that the ``gunicorn`` executable becomes
    available).

Developing this file?  See ``"__main__"`` below (``devcheck``).
"""
import atexit
import shutil
import subprocess
import sys
import tempfile

from textwrap import dedent
from typing import Any, Dict, Optional, Union, Tuple

from hashlib import sha256
from pathlib import Path

from flask import Flask, request, send_file


app = Flask(__name__)
"""The main flask application."""

this_file_dir = Path(__file__).parent.absolute()
"""The directory containing this file."""

server_cache = this_file_dir / "cache"
"""Where to store data files during the execution of the program."""
try:
    server_cache.mkdir(exist_ok=True)
except Exception as e:
    sys.stderr.write(
        f"Unable to create cache directory '{str(server_cache)}': {e}\n"
    )
    sys.exit(1)


def json_code_response(
    *, error: bool, code: int, message: Optional[str] = None, **kwargs
) -> Tuple[Dict[str, Any], int]:
    """Convenience wrapper to return json data and error codes.

    Flask will let you return a tuple ``{dictionary}, int`` and will
    automatically send a json response based off of the dictionary as well as
    set the error code.
    Args:
        error: ``True`` if success, ``False`` otherwise.  User is responsible
            for making sure that ``error`` and ``code`` agree.
        code: The HTTP response code to send.
        message: An optional text response message to provide to the client,
            e.g., what the error was.  Users are encouraged to always include
            a message when ``code != 200``.
        kwargs: Any additional key-value pairs to add to the dictionary return.
            Example::

                return json_code_response(error=False, code=200, sha256="...")

    Returns:
        This method usually just creates this return::

            return {"error": error, "code": code, "message": message}, code

        If any ``kwargs`` are provided, they will be included in the return.
        It is a convenience, so that the dictionary and code tuple does not
        need to be replicated each time.
    """
    ret: Dict[str, Any] = {"error": error, "code": code, **kwargs}
    if message is not None:
        ret["message"] = message
    return ret, code


def render_callback(
    *,
    input_scene: str,
    output_path: str,
    scene_id: str,
    image_type: str,
    width: str,
    height: str,
    near: str,
    far: str,
    focal_x: str,
    focal_y: str,
    fov_x: str,
    fov_y: str,
    center_x: str,
    center_y: str,
    min_depth: str,
    max_depth: str,
) -> Optional[Tuple[Dict[str, Any], int]]:
    """
    Invoke the renderer to produce the specified output path.

    The ``render`` method converts inputs to ``str`` for convenient use with
    ``subprocess.check_call`` or ``subprocess.run``.

    Args:
        input_scene: Path to the scene to render.
        output_path: Where the ``render`` method expects a successful render to
            store the rendered image.
        scene_id: The sha256 sum of the scene being rendered.  This input may
            or may not be useful depending on your server and render choices.
        image_type: The type of image being rendered, one of ``color``,
            ``depth``, or ``label``.  This input may or may not be useful
            depending on your server and render choices.
        width: The desired output width of the rendered image.  Guaranteed to
            be greater than zero.
        height: The desired output height of the rendered image.  Guaranteed to
            be greater than zero.
        near: The near plane of the camera.
        far: The far clipping plane of the camera.
        focal_x: The focal length x, in pixels.
        focal_y: The focal length y, in pixels.
        fov_x: The field of view in the x-direction (in radians).
        fov_y: The field of view in the y-direction (in radians).
        center_x: The principal point's x coordinate in pixels.
        center_y: The principal point's y coordinate in pixels.
        min_depth: The minimum depth range, only meaningful when the provided
            image_type is ``depth``.  ``-1.0`` for non-depth renders.
        max_depth: The maximum depth range, only meaning ful when the provided
            image_type is ``depth``.  ``-1.0`` for non-depth renders.

    Return:
        A successful render should ``return None``.  This indicates that
        ``output_path`` contains a valid image to respond to the client with.

        If there is an error that needs to be returned, a user may do for
        example::

            return json_code_response(error=True, code=500, message="why")

        for which the calling method will pass forward to the client to
        indicate failure.
    """
    # NOTE: the path to `vtk_render_server_backend` is only valid from bazel.
    # The path to the renderer executable to call.
    backend = str(
        (this_file_dir / ".." / "vtk_render_server_backend").resolve()
    )
    proc_args = [
        backend,
        "--input",
        input_scene,
        "--output",
        output_path,
        "--image_type",
        image_type,
        "--width",
        width,
        "--height",
        height,
        "--near",
        near,
        "--far",
        far,
        "--focal_x",
        focal_x,
        "--focal_y",
        focal_y,
        "--fov_x",
        fov_x,
        "--fov_y",
        fov_y,
        "--center_x",
        center_x,
        "--center_y",
        center_y,
    ]
    if image_type == "depth":
        proc_args.extend(["--min_depth", min_depth, "--max_depth", max_depth])
    print(f"$ {' '.join(proc_args)}")
    try:
        proc = subprocess.run(proc_args, capture_output=True)
        # Using subprocess.check_call will not provide any additional
        # information if the renderer produces output on stdout or stderr.
        if proc.returncode != 0:
            message = f"backend exited with code {proc.returncode}."
            stdout = proc.stdout.decode("utf-8")
            stderr = proc.stderr.decode("utf-8")
            if stdout:
                message += f"\nstdout:\n{stdout}"
            if stderr:
                message += f"\nstderr:\n{stderr}"
            raise RuntimeError(message)

        return None  # Indicates success
    except Exception as e:
        return json_code_response(
            error=True,
            code=500,
            message=f"Failed render invocation: {e}",
        )


@atexit.register
def delete_server_cache():
    """
    Delete ``server_cache`` upon exit.

    .. note::

        When developing your server it may be helpful to keep the server cache
        around, simply add a ``return None`` at the beginning of this method
        or comment the code deleting the folder out.
    """
    if server_cache.is_dir():
        shutil.rmtree(server_cache, ignore_errors=True)


def compute_hash(path: Union[str, Path], block_size: int = 1024 * 1024) -> str:
    """Incrementally compute and return the sha256 of the specified path.

    Args:
        path: The path to the file to compute the sha256 hash of.
        block_size: Size of block in bytes to read.  Default: ``1024 * 1024``.
            Assumed to be greater than 0.

    Raises:
        IOError: If the file cannot be loaded.

    Returns:
        The computed sha256 sum as a string.
    """
    h = sha256()
    with open(path, "rb") as f:
        while True:
            data = f.read(block_size)
            if not len(data):
                break

            h.update(data)

    return h.hexdigest()


def gltf_path(sha256_hash: str, image_type: str) -> Path:
    """Return the location the described file is saved.

    Args:
        sha256_hash: The identifier sha256 sum of the gltf file.
        image_type: The pipeline, one of ``color``, ``depth``, or ``label``.

    Returns:
        The path where this file should be saved.
    """
    return this_file_dir / server_cache / f"{sha256_hash}-{image_type}.gltf"


@app.route("/")
def root():
    """The main listing page, renders a simple redirect page including where
    the server cache lives for development.  This endpoint (``/``) is not
    required by the drake server-client relationship and only serves to aid
    development.
    """
    return dedent(
        f"""
        <!doctype html>
        <html>
          <body>
            <h1>Drake Render Server</h1>
            <hr>
            <ul>
              <li><a href="/upload">Upload</a> a scene.</li>
              <li><a href="/render">Render</a> a scene.</li>
            </ul>

            <h1>Server Cache</h1>
            <hr>
            <p>The server cache lives here: <tt>{server_cache}</tt></p>
          </body>
        </html>
    """
    )


@app.route("/upload", methods=["GET", "POST"])
def upload():
    if request.method == "POST":
        if "data" in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Form key `data` should be a file.",
            )
        if "data" not in request.files:
            return json_code_response(
                error=True,
                message="Expected key `data`, which should be a file.",
                code=400,
            )

        if "image_type" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message=(
                    "Expected key `image_type` (`color`, `depth`, or `label`)"
                ),
            )
        image_type = request.form["image_type"]
        if image_type not in {"color", "depth", "label"}:
            return json_code_response(
                error=True,
                code=400,
                message=(
                    f"Key image_type of {image_type} invalid.  Must be one of "
                    "`color`, `depth`, or `label`"
                ),
            )

        try:
            # This will be a werkzeug.datastructures.FileStorage, see
            # https://werkzeug.palletsprojects.com/en/2.0.x/datastructures/
            data = request.files["data"]
            _, temp_path = tempfile.mkstemp(dir=server_cache)
            data.save(temp_path)

            # Compute the hash and rename it.
            temp_path = Path(temp_path)
            sha = compute_hash(temp_path)
            gltf_out = gltf_path(sha, image_type)
            temp_path.rename(gltf_out)

            # Print to the console what was just saved, including the mime type
            # if it has been provided.  Users may choose to fail the upload in
            # the event of an unsupported mime type being provided, however in
            # general relying on accurate mime type reporting is not advisable.
            message = (
                f"==> Scene file with image_type={image_type} saved to "
                f"'{str(gltf_out)}' with a sha256 hash of '{sha}'."
            )
            mime_type = getattr(data, "content_type", None)
            if mime_type:
                message += f"  Client reported a mime type of '{mime_type}'."
            print(message)

            # Report the computed sha256 back to the client.
            return json_code_response(error=False, code=200, sha256=sha)
        except Exception as e:
            return json_code_response(
                error=True, code=500, message=f"Internal server error: {e}"
            )

    # request.method := "GET"
    return dedent(
        """\
        <!doctype html>
        <html>
          <body>
            <h1>Upload glTF</h1>
            <form method="post" enctype="multipart/form-data">
              <input type="file" name="data">
              <div>
                <label for="image_type">
                  Image Type
                  (<tt>color</tt>, <tt>depth</tt>, or <tt>label</tt>)
                </label>
                <input type="text" name="image_type">
              </div>
              <input type="submit" name="submit" value="Upload">
            </form>
          </body>
        </html>
    """
    )


@app.route("/render", methods=["GET", "POST"])
def render():
    if request.method == "POST":
        if "id" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `id` (sha256 of uploaded scene).",
            )
        scene_id = request.form["id"]

        if "image_type" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message=(
                    "Expected key `image_type` (`color`, `depth`, or `label`)"
                ),
            )
        image_type = request.form["image_type"]
        if image_type not in {"color", "depth", "label"}:
            return json_code_response(
                error=True,
                code=400,
                message=(
                    f"Key image_type of {image_type} invalid.  Must be one of "
                    "`color`, `depth`, or `label`"
                ),
            )

        gltf_in = gltf_path(request.form["id"], image_type)
        if not gltf_in.is_file():
            return json_code_response(
                error=True,
                code=400,
                message=f"Could not find {gltf_in.name}, was it uploaded?",
            )

        if "width" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `width` (width of rendered image).",
            )
        width = request.form["width"]
        try:
            width = int(width)
            if width <= 0:
                raise ValueError
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested width `{width}` is not valid.",
            )

        if "height" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `height` (height of rendered image).",
            )
        height = request.form["height"]
        try:
            height = int(height)
            if height <= 0:
                raise ValueError
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested height `{height}` is not valid.",
            )

        if "near" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `near` (near plane of camera).",
            )
        near = request.form["near"]
        try:
            near = float(near)
            if near < 0.0:
                raise ValueError
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested near plane `{near}` is not valid.",
            )

        far = request.form["far"]
        try:
            far = float(far)
            if far < 0.0:
                raise ValueError
            if near >= far:
                raise ValueError
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested far plane `{far}` is not valid.",
            )

        if "focal_x" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `focal_x` (focal length x, in pixels).",
            )
        focal_x = request.form["focal_x"]
        try:
            focal_x = float(focal_x)
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested focal_x `{focal_x}` is not valid.",
            )

        if "focal_y" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `focal_y` (focal length x, in pixels).",
            )
        focal_y = request.form["focal_y"]
        try:
            focal_y = float(focal_y)
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested focal_y `{focal_y}` is not valid.",
            )

        if "fov_x" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `fov_x` (field of view x, radians).",
            )
        fov_x = request.form["fov_x"]
        try:
            fov_x = float(fov_x)
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested fov_x `{fov_x}` is not valid.",
            )

        if "fov_y" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `fov_y` (field of view y, radians).",
            )
        fov_y = request.form["fov_y"]
        try:
            fov_y = float(fov_y)
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested fov_y `{fov_y}` is not valid.",
            )

        if "center_x" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `center_x` (principal point x, pixels).",
            )
        center_x = request.form["center_x"]
        try:
            center_x = float(center_x)
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested center_x `{center_x}` is not valid.",
            )

        if "center_y" not in request.form:
            return json_code_response(
                error=True,
                code=400,
                message="Expected key `center_y` (principal point y, pixels).",
            )
        center_y = request.form["center_y"]
        try:
            center_y = float(center_y)
        except ValueError:
            return json_code_response(
                error=True,
                code=400,
                message=f"Requested center_y `{center_y}` is not valid.",
            )

        min_depth = -1.0
        max_depth = -1.0
        if image_type == "depth":
            if "min_depth" not in request.form:
                return json_code_response(
                    error=True,
                    code=400,
                    message="Expected key `min_depth` for depth render.",
                )
            min_depth = request.form["min_depth"]
            try:
                min_depth = float(min_depth)
                if min_depth < 0.0:
                    raise ValueError
            except ValueError:
                return json_code_response(
                    error=True,
                    code=400,
                    message=f"Requested min_depth `{min_depth}` is not valid.",
                )

            if "max_depth" not in request.form:
                return json_code_response(
                    error=True,
                    code=400,
                    message="Expected key `max_depth` for depth render.",
                )
            max_depth = request.form["max_depth"]
            try:
                max_depth = float(max_depth)
                if max_depth < 0.0:
                    raise ValueError
            except ValueError:
                return json_code_response(
                    error=True,
                    code=400,
                    message=f"Requested max_depth `{max_depth}` is not valid.",
                )

            if min_depth < 0.0 or max_depth < 0.0:
                return json_code_response(
                    error=True,
                    code=400,
                    message=(
                        f"Provided min_depth={min_depth} and max_depth="
                        f"{max_depth} are invalid.  They must be positive."
                    ),
                )
            if max_depth <= min_depth:
                return json_code_response(
                    error=True,
                    code=400,
                    message=(
                        f"Provided max_depth={max_depth} must be greater than "
                        f"min_depth={min_depth}."
                    ),
                )

        # TODO(svenevs): 16U PNG depth support can be tested here.
        if image_type in {"color", "label"}:
            out_type = "png"
        else:  # image_type := "depth"
            out_type = "tiff"
        image_out = gltf_in.parent / (gltf_in.name + f".{out_type}")
        if image_out.is_file():
            image_out.unlink()

        ret = render_callback(
            input_scene=str(gltf_in),
            output_path=str(image_out),
            scene_id=scene_id,
            image_type=image_type,
            width=str(width),
            height=str(height),
            near=str(near),
            far=str(far),
            focal_x=str(focal_x),
            focal_y=str(focal_y),
            fov_x=str(fov_y),
            fov_y=str(fov_y),
            center_x=str(center_x),
            center_y=str(center_y),
            min_depth=str(min_depth),
            max_depth=str(max_depth),
        )
        if ret is not None:
            json_dict, code = ret
            return json_dict, code

        return send_file(str(image_out), mimetype=f"image/{out_type}")

    # request.method := "GET"
    min_max = 'min="16" max="65535"'
    return dedent(
        f"""\
        <!doctype html>
        <html>
          <body>
            <h1>Render Scene</h1>
            <form method="post" enctype="multipart/form-data">
              <table>
                <tr>
                  <td>sha256 Hash Identifier</td>
                  <td>
                    <input type="text" name="id">
                  </td>
                </tr>
                <tr>
                  <td>
                    Image Type
                    (<tt>color</tt>, <tt>depth</tt>, or <tt>label</tt>)
                  </td>
                  <td>
                    <input type="text" name="image_type">
                  </td>
                <tr>
                  <td>Image Width</td>
                  <td>
                    <input type="number" name="width" {min_max}>
                  </td>
                </tr>
                <tr>
                  <td>Image Height</td>
                  <td>
                    <input type="number" name="height" {min_max}>
                  </td>
                </tr>
                <tr>
                  <td>Near Plane of Camera</td>
                  <td>
                    <input type="number" name="near">
                  </td>
                </tr>
                <tr>
                  <td>Far Plane of Camera</td>
                  <td>
                    <input type="number" name="far">
                  </td>
                </tr>
                <tr>
                  <td>Focal Length X in Pixels</td>
                  <td>
                    <input type="number" name="focal_x">
                  </td>
                </tr>
                <tr>
                  <td>Focal Length Y in Pixels</td>
                  <td>
                    <input type="number" name="focal_y">
                  </td>
                </tr>
                <tr>
                  <td>Field of View X in Radians</td>
                  <td>
                    <input type="number" name="focal_x">
                  </td>
                </tr>
                <tr>
                  <td>Field of View Y in Radians</td>
                  <td>
                    <input type="number" name="focal_y">
                  </td>
                </tr>
                <tr>
                  <td>Principal Point X in Pixels</td>
                  <td>
                    <input type="number" name="center_x">
                  </td>
                </tr>
                <tr>
                  <td>Principal Point Y in Pixels</td>
                  <td>
                    <input type="number" name="center_y">
                  </td>
                </tr>
                <tr>
                  <td>Depth range min_depth</td>
                  <td>
                    <input type="number" name="min_depth" value="-1.0">
                  </td>
                </tr>
                <tr>
                  <td>Depth range max_depth</td>
                  <td>
                    <input type="number" name="max_depth" value="-1.0">
                  </td>
                </tr>
              </table>
              <input type="submit" name="submit" value="Render">
            </form>
          </body>
        </html>
    """
    )


if __name__ == "__main__":
    host = "127.0.0.1"
    port = 8000
    # NOTE: see geometry/render/dev/BUILD.bazel, this is a convenience bypass.
    if len(sys.argv) > 1:
        if sys.argv[1] == "proxy_run_gunicorn":
            subprocess.run(
                [
                    "gunicorn",
                    "--workers",
                    "8",
                    "--bind",
                    f"{host}:{port}",
                    "wsgi:app",
                ],
                cwd=this_file_dir,
            )
        # NOTE: this just runs the formatting and linting checks on this file.
        elif sys.argv[1] == "devcheck":
            # PyPI packages to install:
            #     black
            #     flake8
            #     mypy
            #
            # You may need to `pip install -U importlib_metadata` to satisfy
            # mypy, or ignore warnings about importlib_metadata types.
            # NOTE: use `run` not `check_call` so that all run.
            def log_and_run(*args):
                print(f"$ {' '.join(args)}")
                subprocess.run(args)

            this_file = str(Path(__file__).absolute())
            # NOTE: drake uses 79 via flake8 checks.
            log_and_run("black", "--line-length", "79", this_file)
            log_and_run("mypy", this_file)
            log_and_run("flake8", this_file)
    else:
        app.run(host=host, port=port)
