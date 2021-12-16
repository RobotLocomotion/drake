"""
A prototype glTF render server.  Users seeking to utilize their own renderer
as a replacement for the sample VTK based C++ executable can simply change the
implementation in the ``render_callback`` method.  Additionally, you will
likely want to modify the ``root`` that returns a rendered ``README.md`` (to
return your own string for the ``/`` index if e.g., you do not have a
``README.md`` in the same directory).

The dependent package requirements for running this server::

    flask>=1.1
    markdown
    gunicorn

.. note::

    If using ``apt-get`` to manage your python packages, you will want to
    ``apt-get install gunicorn`` (so that the ``gunicorn`` executable becomes
    available).

Developing this file: see ``"__main__"`` below (``devcheck``).
"""
import subprocess
import sys
import tempfile

from textwrap import dedent
from typing import Any, Dict, Optional, Union, Tuple

from hashlib import sha256
from pathlib import Path

from flask import Flask, request, send_file

import markdown
from markdown.extensions.codehilite import CodeHiliteExtension

app = Flask(__name__)
"""The main flask application."""

this_file_dir = Path(__file__).parent.absolute()
"""The directory containing this file."""

server_cache = this_file_dir / "cache"
"""Where to store data files during the execution of the program."""
# TODO(svenevs): Right location do delete server cache?  Global variables are a
# problem with flask in general (multiple processes with gunicorn).
try:
    server_cache.mkdir(exist_ok=True)
except Exception as e:
    sys.stderr.write(
        f"Unable to create cache directory '{str(server_cache)}': {e}\n"
    )
    sys.exit(1)


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


@app.route("/")
def root():
    """The main listing page, renders README.md in this directory."""
    try:
        with open(this_file_dir / "README.md") as readme:
            return markdown.markdown(
                readme.read()
                + f"# Server Cache\nThe cache is here: `{server_cache}`.",
                # https://python-markdown.github.io/extensions/#officially-supported-extensions
                extensions=[
                    "def_list",
                    "extra",
                    "fenced_code",
                    "tables",
                    CodeHiliteExtension(noclasses=True),
                    "toc",
                ],
            )
    except Exception:
        return "<p>Nothing to see here...</p>"


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
            _, tmp_path = tempfile.mkstemp(dir=server_cache)
            data.save(tmp_path)

            # Compute the hash and rename it.
            tmp_path = Path(tmp_path)
            sha = compute_hash(tmp_path)
            gltf_out = gltf_path(sha, image_type)
            tmp_path.rename(gltf_out)

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


def render_callback(
    *,
    input_scene: str,
    output_path: str,
    scene_id: str,
    image_type: str,
    width: str,
    height: str,
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

    Return:
        A successful render should ``return None``.  This indicates that
        ``output_path`` contains a valid image to respond to the client with.

        If there is an error that needs to be returned, a user may do for
        example::

            return json_code_response(error=True, code=500, message="why")

        for which the calling method will pass forward to the client to
        indicate failure.
    """
    # TODO(svenevs): make this configurable and pass the path via bazel?
    # The path to the renderer executable to call.
    backend = str((this_file_dir / ".." / "render_server_backend").resolve())
    proc_args = [
        backend,
        "--input",
        input_scene,
        "--output",
        output_path,
        "--width",
        width,
        "--height",
        height,
    ]
    print(f"$ {' '.join(proc_args)}")
    try:
        subprocess.check_call(proc_args)
        return None
    except subprocess.CalledProcessError as cpe:
        return json_code_response(
            error=True,
            code=500,
            message=f"Failed render invocation: {cpe}",
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

        # TODO(svenevs): get this in a helper method so it's not duplicated.
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

        png_out = gltf_in.parent / (gltf_in.name + ".png")
        # TODO(svenevs): Renderer errors out if the file exists (intentional),
        # instead of deleting the whole cache we will re-render.  A smarter
        # implementation would just skip rendering and send the file if it
        # exists!  Since the renderer here is under development, though, it is
        # better to re-render (since our renderer is very fast).
        if png_out.is_file():
            png_out.unlink()

        ret = render_callback(
            input_scene=str(gltf_in),
            output_path=str(png_out),
            scene_id=scene_id,
            image_type=image_type,
            width=str(width),
            height=str(height),
        )
        if ret is not None:
            json_dict, code = ret
            return json_dict, code

        return send_file(str(png_out), mimetype="image/png")

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
              </table>
              <input type="submit" name="submit" value="Render">
            </form>
          </body>
        </html>
    """
    )


if __name__ == "__main__":
    # TODO(svenevs): see geometry/render/dev/BUILD.bazel py_binary call,
    # replace this with an official way of using bazel to call gunicorn?
    host = "127.0.0.1"
    port = 8000
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
        # TODO(svenevs): we can delete this, these were just to assist with
        # development, finding dead code, etc.
        # NOTE: this just runs the formatting and linting checks on this file.
        elif sys.argv[1] == "devcheck":
            # PyPI packages to install:
            #     black
            #     flake8
            #     mypy
            #     types-Markdown
            # NOTE: use `run` not `check_call` so that all run.
            def log_and_run(*args):
                print(f"$ {' '.join(args)}")
                subprocess.run(args)

            this_file = str(Path(__file__).absolute())
            # NOTE: drake uses 79 via flake8 checks.
            log_and_run("black", "--line-length", "79", this_file)
            log_and_run("flake8", this_file)
            log_and_run("mypy", this_file)
    else:
        app.run(host=host, port=port)
