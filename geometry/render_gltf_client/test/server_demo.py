"""
A prototype glTF render server that receives HTTP requests and dispatches to a
renderer backend to render images.  Users seeking to replace the sample
VTK-based backend can simply change the implementation in `render_callback()`.

Check the README page for more details:
https://github.com/RobotLocomotion/drake/blob/master/geometry/render_gltf_client/test/README.md
"""

import argparse
import atexit
import datetime
from enum import Enum
from hashlib import sha256
from io import BytesIO
import itertools
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from textwrap import dedent
from typing import Dict, Union

from bazel_tools.tools.python.runfiles.runfiles import Create as CreateRunfiles
from flask import Flask, request, send_file

"""The main flask application."""
app = Flask(__name__)


"""Where the client must upload files to and wait for an image response from.
If the drake client transmits files to an alternative endpoint, make sure to
update this value accordingly.

Warning:
    If you change this value to `/`, then you must delete or comment out the
    `root()` function below.
"""
RENDER_ENDPOINT = "/render"


"""Whether or not the server should cleanup its local cache. For debugging
purposes, set this to `False` to prevent the deletion of scene files / rendered
images.  See also: `delete_server_cache()`.
"""
CLEANUP = True


"""Where to store data files during the execution of the program."""
TMP_DIR = Path(os.environ.get("TEST_TMPDIR", "/tmp")) / "server_demo"
TMP_DIR.mkdir(exist_ok=True)


###############################################################################
# Helper classes and functions.
###############################################################################
def compute_hash(path: Path) -> str:
    """Returns the sha256 given a file path."""
    h = sha256()
    # Use a fixed block size to read files.
    BLOCK_SIZE = 1024 * 1024
    with open(path, "rb") as f:
        while True:
            data = f.read(BLOCK_SIZE)
            if not len(data):
                break
            h.update(data)
    return h.hexdigest()


@atexit.register
def delete_server_cache():
    """Deletes `TMP_DIR` upon exit, e.g., `ctrl+C`, when CLEANUP is True."""
    if CLEANUP:
        if TMP_DIR.is_dir():
            shutil.rmtree(TMP_DIR, ignore_errors=True)


class RenderError(Exception):
    """An exception class used for signaling the caller the error message and
    the HTTP response code.  Any exceptions raised in `render_callback()`
    should use this class to populate an informative `message` and
    `error_code`.

    Args:
        message (str): The exception message to communicate to the client.
        error_code (int): The HTTP response code to indicate the type of the
            failure. If not provided, it will be `400`.  If a value of less
            than `400` is provided, a `500` error code (internal server error)
            will be sent instead as this exception should only be used to
            indicate failure.
    """

    def __init__(self, message: str, error_code: int = 400):
        super().__init__(message)
        # Do not allow successful HTTP codes to be used, this is an Error.
        assert (
            error_code >= 400
        ), f"RenderError: provided error code {error_code} is less than 400"
        self.message = message
        self.error_code = error_code

    def flask_json_code_response(self, **kwargs):
        """Returns a formatted response consistent with what Flask supports,
        i.e., (json_dict, http_return_code).  Users may provide additional
        `**kwargs` if needed. However, generally speaking, the relevant
        information to communicate to the client should be provided in
        `message` when an exception is raised.
        """
        return (
            {
                "error": True,
                "message": self.message,
                "code": self.error_code,
                **kwargs,
            },
            self.error_code,
        )


class FieldType(Enum):
    """Describes the type expected in a given `<form>` field entry."""

    File = 0
    String = 1
    Int = 2
    Float = 3


class RenderRequest:
    """A `RenderRequest` wrapped around a `flask.request` that validates the
    entries in a client's `<form>`.  Users of this class can assume all the
    fields are sensible, and should only access those fields through
    `get_field()` function.

    Note:
        When using functions such as `subprocess.run()` or
        `subprocess.check_call()`, be sure to convert any non-string attributes
        to a string in the list of command line arguments.  They are kept as
        numeric types in the attributes to facilitate a user performing
        any additional logic they need with the provided parameters before
        issuing the call to their renderer (e.g., creating a perspective
        projection matrix).
    """

    # A class constant listing all the expected fields and their types from a
    # request <form>. It's used for validation and should be kept consistent
    # with Drake's documentation on `Render Endpoint <form> Data` section.
    # See also: https://drake.mit.edu/doxygen_cxx/group__render__engine__gltf__client__server__api.html  # noqa
    EXPECTED_FORM_FIELDS: Dict[str, FieldType] = {
        "scene": FieldType.File,
        "scene_sha256": FieldType.String,
        "image_type": FieldType.String,
        "min_depth": FieldType.Float,
        "max_depth": FieldType.Float,
        "width": FieldType.Int,
        "height": FieldType.Int,
        "near": FieldType.Float,
        "far": FieldType.Float,
        "focal_x": FieldType.Float,
        "focal_y": FieldType.Float,
        "fov_x": FieldType.Float,
        "fov_y": FieldType.Float,
        "center_x": FieldType.Float,
        "center_y": FieldType.Float,
        # NOTE: This input is not actually validated.
        "submit": FieldType.String,
    }

    def __init__(self, flask_request: request, verbose: bool = True):
        """
        Args:
            flask_request (`flask.request`): The flask request instance
                provided to `render_endpoint()`.
            verbose (bool): Whether to log messages.  Default: True.
        """
        self.request = flask_request
        self.verbose = verbose

        # Validate the fields from the request match `EXPECTED_FORM_FIELDS`.
        self._check_form_entries()

        # An internal dictionary to store validated form fields.
        self._fields_map = {}

        # Validate fields in `flask.request.form` first.
        for field_name in self.request.form:
            if field_name == "scene":
                raise RenderError(
                    "Form field 'scene' should be in the files section."
                )
            field_type = self.EXPECTED_FORM_FIELDS[field_name]

            if field_type in [FieldType.Int, FieldType.Float]:
                self._fields_map[field_name] = self._parse_numeric(
                    field_name, field_type
                )
            else:  # field_type == FieldType.String:
                self._fields_map[field_name] = self.request.form[field_name]

        # Validate `image_type` field.
        image_type = self._fields_map["image_type"]
        if image_type not in ["color", "depth", "label"]:
            raise RenderError(
                f"Field image_type='{image_type}' is not valid, must be "
                "either 'color', 'depth', or 'label'."
            )

        # Validate `min_depth` and `max_depth` fields.
        if image_type == "depth":
            min_depth = self._fields_map["min_depth"]
            max_depth = self._fields_map["max_depth"]
            if min_depth >= max_depth:
                raise RenderError(
                    f"Form fields min_depth='{min_depth}' and max_depth="
                    f"'{max_depth}' are not valid. max_depth must be greater "
                    "than min_depth."
                )

        # Validate `near` and `far` fields.
        near = self._fields_map["near"]
        far = self._fields_map["far"]
        if near >= far:
            raise RenderError(
                f"Form fields near='{near}' and far='{far}' not valid, the "
                "near plane must be in front of the far plane."
            )

        # Validate `center_x` and `center_y` fields.
        center_x = self._fields_map["center_x"]
        center_y = self._fields_map["center_y"]
        width = self._fields_map["width"]
        height = self._fields_map["height"]
        if center_x >= width or center_y >= height:
            raise RenderError(
                f"Form fields center_x='{center_x}' and center_y='{center_y}' "
                f"are not valid.  center_x must be less than width='{width}', "
                f"and center_y must be less than the height='{height}'."
            )

        # Validate the only field, i.e., `scene`, in `flask.request.files`.
        if "scene" not in self.request.files and self.request.files.len() != 1:
            raise RenderError(
                "The files section should contain a single field: 'scene'."
            )
        self._fields_map["scene"] = self._parse_scene("scene")

    def get_field(self, field_name: str) -> Union[int, float, str]:
        """Queries the value of a field in the form. This function should be
        the **only** function called outside this class."""
        return self._fields_map[field_name]

    def _check_form_entries(self):
        """Checks if there are any extra fields from the request."""
        expected_fields = set(self.EXPECTED_FORM_FIELDS.keys())
        provided_fields = set(
            itertools.chain(self.request.files, self.request.form)
        )

        # All the fields provided should be a subset of `expected_fields`. Some
        # of the fields are optional, i.e., min_depth and max_depth, and thus
        # the two sets may not match exactly.
        extras = provided_fields.difference(expected_fields)
        if extras:
            raise RenderError(
                f"Extra field(s) {extras} in the request <form>."
            )

    def _parse_numeric(
        self, field_name: str, field_type: FieldType
    ) -> Union[int, float]:
        """Checks if the raw string value can be converted to the expected type
        and the numeric value is greater than 0 (as an indication of being
        sensible)."""
        try:
            value = self.request.form[field_name]
            if field_type == FieldType.Int:
                numeric_value = int(value)
            else:  # field_type == FieldType.Float
                numeric_value = float(value)

            # Ensure the value is positive and finite (not nan).
            if numeric_value <= 0 or not math.isfinite(numeric_value):
                raise ValueError
            return numeric_value
        except ValueError:
            raise RenderError(
                f"Form field {field_name} with value '{value}' is not a valid "
                "number."
            )
        except Exception as e:
            raise RenderError(
                f"Internal server error processing field '{field_name}': {e}",
                error_code=500,
            )

    def _parse_scene(self, field_name: str) -> Path:
        """Validates the uploaded scene file and returns the path to where it
        was saved in the `TMP_DIR`.

        Warning:
            It is assumed that `_fields_map` contains both `scene_sha256` and
            `image_type` fields prior to calling this function.

        Raises:
            RenderError: In the event that the provided sha256 hash is not the
            same as what is computed from the uploaded file, or any other
            errors that occur in trying to move the file to the `TMP_DIR`.
        """
        try:
            # This will be a werkzeug.datastructures.FileStorage, see
            # https://werkzeug.palletsprojects.com/en/2.0.x/datastructures/
            scene_data = self.request.files[field_name]
            _, temp_path_str = tempfile.mkstemp(dir=TMP_DIR)
            scene_data.save(temp_path_str)

            # Compute and validate the sha256 hash of the uploaded scene.
            temp_path = Path(temp_path_str)
            sha256 = compute_hash(temp_path)
            expected_sha256 = self._fields_map["scene_sha256"]
            if sha256 != expected_sha256:
                # Delete the file before erroring out.
                if CLEANUP:
                    try:
                        temp_path.unlink()
                    except Exception:
                        pass
                raise RenderError(
                    f"Provided scene_sha256='{expected_sha256}' does not "
                    f"match the computed sha256 of '{sha256}'.",
                    error_code=500,
                )

            # Create a timestamp for saving the file to avoid collisions.
            timestamp = datetime.datetime.now().strftime(
                "%Y-%m-%d_%H-%M-%S-%f"
            )
            scene_path = TMP_DIR / f"{timestamp}.gltf"
            temp_path.rename(scene_path)

            if self.verbose:
                print(f"Saving scene file: {str(scene_path)}")
            return scene_path
        except RenderError as re:
            raise re from None  # Forward the exception.
        except Exception as e:
            raise RenderError(f"Internal server error: {e}", error_code=500)


###############################################################################
# The main rendering call, replace with your own desired implementation.
###############################################################################
def render_callback(render_request: RenderRequest) -> str:
    """Invokes the renderer to produce and return an output image path.

    The data fields in `render_request` have already been validated based off
    the original flask request.  This function is responsible for determining
    the final output image path and file extension.

    Args:
        render_request (RenderRequest): The validated flask request wrapped in
            a `RenderRequest` instance.

    Returns:
        A path to the final rendered image file.

    Raises:
        RenderError: In the event that anything goes wrong during the call to
            the renderer, a `RenderError` should be raised.  The caller of this
            function should look for this specific kind of exception first, and
            forward the corresponding error response to the client.  Any other
            exceptions raised will result in an internal server error response
            message to the client.
    """
    # Locate the binary of the renderer.
    runfiles = CreateRunfiles()
    backend_bin = runfiles.Rlocation(
        "drake/geometry/render_gltf_client/server_vtk_backend"
    )

    # Determine the extension and the file path of the rendering.
    # NOTE: When CLEANUP=True, this image path is deleted by the caller.
    output_path = str(render_request.get_field("scene"))
    image_type = render_request.get_field("image_type")
    if image_type in ["color", "label"]:
        output_path += ".png"
    else:  # image_type == "depth"
        output_path += ".tiff"

    # Create the command-line arguments to pass to the render backend.
    # NOTE: Make sure you convert entries in the list to `str`.
    proc_args = [
        backend_bin,
        "--input_path",
        str(render_request.get_field("scene")),
        "--output_path",
        output_path,
        "--image_type",
        str(render_request.get_field("image_type")),
        "--width",
        str(render_request.get_field("width")),
        "--height",
        str(render_request.get_field("height")),
        "--near",
        str(render_request.get_field("near")),
        "--far",
        str(render_request.get_field("far")),
        "--focal_x",
        str(render_request.get_field("focal_x")),
        "--focal_y",
        str(render_request.get_field("focal_y")),
        "--fov_x",
        str(render_request.get_field("fov_x")),
        "--fov_y",
        str(render_request.get_field("fov_y")),
        "--center_x",
        str(render_request.get_field("center_x")),
        "--center_y",
        str(render_request.get_field("center_y")),
    ]
    if image_type == "depth":
        proc_args.extend(
            [
                "--min_depth",
                str(render_request.get_field("min_depth")),
                "--max_depth",
                str(render_request.get_field("max_depth")),
            ]
        )

    # Call the render backend, including capturing any errors.
    try:
        proc = subprocess.run(proc_args, capture_output=True)
        if proc.returncode != 0:
            message = f"backend exited with code {proc.returncode}."
            stdout = proc.stdout.decode("utf-8")
            stderr = proc.stderr.decode("utf-8")
            if stdout:
                message += f"\nstdout:\n{stdout}"
            if stderr:
                message += f"\nstderr:\n{stderr}"
            raise RuntimeError(message)

        # Inform the caller where the final rendering resides.
        return output_path
    except Exception as e:
        raise RenderError(f"Failed render invocation: {e}", error_code=500)


###############################################################################
# Flask endpoint implementations.
###############################################################################
@app.route("/")
def root():
    """The main listing page renders a simple redirect page including where
    the server cache lives for development.  This endpoint (`/`) is not
    required by the drake server-client relationship and only serves to aid
    development.
    """
    html_prefix = dedent(
        f"""\
        <!doctype html>
        <html>
          <body>
            <h1>Drake Render Server</h1>
            <hr>
            <p>
              <a href="{RENDER_ENDPOINT}">Render</a> a scene.
            </p>
        """
    )
    html_suffix = dedent(
        """\
          </body>
        </html>
        """
    )
    # Inform the developer of where the server cache lives in development mode.
    if app.config["ENV"].lower() == "development":
        indent = "  " * 2
        html_interior = (
            f"{indent}<h1>Server Cache</h1>\n"
            f"{indent}<hr>\n"
            f"{indent}<p>\n"
            f"{indent}  This is a development server.  The server cache\n"
            f"{indent}  lives here: <tt>{str(TMP_DIR)}</tt>\n"
            f"{indent}</p>\n"
        )
    else:
        html_interior = ""

    return f"{html_prefix}{html_interior}{html_suffix}"


@app.route(RENDER_ENDPOINT, methods=["GET", "POST"])
def render_endpoint():
    """The main rendering endpoint for the client to communicate with.

    A client's request will be validated and provided to `render_callback()`,
    with the resultant image transmitted back to the client.
    """
    if request.method == "POST":
        try:
            # Validate the request and render the image.
            render_request = RenderRequest(request)
            output_image = Path(render_callback(render_request))
            if render_request.verbose:
                print(f"Rendering image: {str(output_image)}")

            # Now that the image is rendered, it is safe to delete the scene.
            if CLEANUP:
                if render_request.verbose:
                    scene_str = str(render_request.get_field("scene"))
                    print(f"Deleting scene file: {scene_str}")
                render_request.get_field("scene").unlink(missing_ok=True)

            # The mime type response is only populated based off the file
            # extension, implementation of render_callback is responsible for
            # invalid file extensions / mime types being reported.
            #
            # NOTE: the client does *NOT* rely on the mime type.
            mime_type = None
            image_extension = output_image.suffix.lower()
            if image_extension == ".png":
                mime_type = "image/png"
            elif image_extension in {".tif", ".tiff"}:
                mime_type = "image/tiff"
            else:
                raise RuntimeError(
                    f"Missing mime_type for '{image_extension}'"
                )
            # If we are not deleting the file, use the file path directly for
            # simplicity.  Otherwise, we load the image into RAM, delete it,
            # and then send that.  See the documentation:
            #
            # https://flask.palletsprojects.com/en/2.0.x/api/#flask.send_file
            #
            # There is a small concern in loading into RAM when sending files
            # in the sense that the potential wrapping frameworks such as nginx
            # or apache are optimized for sending files (not python buffers).
            #
            # However, in practice, the size of the image file will be on the
            # order of megabytes and it is preferable to delete the image file
            # now rather than implement a cleanup scheme on the next request
            # (you cannot execute code after send_file).
            if not CLEANUP:
                return send_file(output_image, mimetype=mime_type)
            else:
                buffer = None
                with open(output_image, "rb") as f:
                    buffer = BytesIO(f.read())
                if render_request.verbose:
                    print(f"Deleting rendered image: {str(output_image)}")
                output_image.unlink(missing_ok=True)
                return send_file(buffer, mimetype=mime_type)
        except RenderError as re:
            return re.flask_json_code_response()
        except Exception as e:
            # Similar to RenderError.flask_json_code_response(), see docs.
            return (
                {
                    "error": True,
                    "message": f"Internal server error: {e}",
                    "code": 500,
                },
                500,
            )

    # request.method := "GET"
    html_prefix = dedent(
        """
        <!doctype html>
        <html>
          <body>
            <h1>Render Scene</h1>
            <form method="post" enctype="multipart/form-data">
              <table>
    """
    )
    html_suffix = dedent(
        """\
              </table>
              <input type="submit" name="submit" value="Render">
            </form>
          </body>
        </html>
    """
    )

    # A nested function to convert a FieldType to a string.
    def html_input_type(field_type: FieldType) -> str:
        if field_type == FieldType.File:
            return "file"
        elif field_type in {FieldType.Int, FieldType.Float}:
            return "number"
        else:  # field_type == FieldType.String
            return "text"

    # Generate the <table> entries indented properly.
    tr_indent = "  " * 4
    td_indent = "  " * 5
    table_rows = []
    for field_name, field_type in RenderRequest.EXPECTED_FORM_FIELDS.items():
        row = f"{tr_indent}<tr>\n"
        input_type = html_input_type(field_type)
        row += (
            f'{td_indent}<td><input type="{input_type}" name="{field_name}">\n'
        )
        row += f"{tr_indent}</tr>\n"
        table_rows.append(row)

    return f"{html_prefix}{''.join(table_rows)}{html_suffix}"


def main():
    parser = argparse.ArgumentParser(description=__doc__,)
    parser.add_argument(
        "--host",
        type=str,
        required=False,
        default="127.0.0.1",
        help="URL to host on, default: 127.0.0.1.",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=False,
        default=8000,
        help="Port to host on, default: 8000.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Whether to run in debug mode in which flask reloads the server "
        "automatically when file changes.",
    )
    parser.add_argument(
        "--acceptance_test",
        action="store_true",
        default=False,
        help="Whether to run as an acceptance test that constructs a Flask app"
        " but doesn't run it.",
    )

    args = parser.parse_args()
    if args.acceptance_test:
        return
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
