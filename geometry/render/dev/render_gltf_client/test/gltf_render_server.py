"""
A prototype glTF render server.  Users seeking to utilize their own renderer
as a replacement for the sample VTK based C++ executable can simply change the
implementation in the :func:`render_callback` method.  Briefly refer to the
class documentation of :class:`RenderRequest` to familiarize yourself with what
attributes are parsed and made available for you.  Then replace the
implementation of :func:`render_callback` with your own implementation to call
your renderer of choice.

The dependent package requirements for running this server::

    flask>=1.1

Tip:
    While developing your server, it can be helpful to provide the ``--debug``
    flag which will result in the server reloading any time the file changes.
    See also the documentation on the ``FLASK_ENV`` environment variable.  Its
    default value is ``"production"``, but setting it to ``"development"`` can
    be helpful in iteratively developing the file.  This server does not use
    any additional flask plugins, so the ``--debug`` flag will work reliably.
    However, the environment variable is to be set **before** launching the
    server, and any extensions you may be using could have already initialized
    before ``app.run(..., debug=True)`` so you may get unreliable behavior
    and should instead prefer using the environment variable.  Examples can be
    found `in the flask documentation <flask_env_>`_.

    .. _flask_env:
        https://flask.palletsprojects.com/en/2.0.x/config/#environment-and-debug-features

    Lastly, the server implemented here provides a brief html page rendering
    when a user issues a ``GET /`` (e.g., you visit ``localhost:8000`` in your
    browser).  When ``FLASK_ENV="development"``, this server will include the
    path to the :data:`SERVER_CACHE` so you may easily navigate there in your
    terminal if desired.  Information about the server directory structure
    should not be revealed in production.

Warning:
    There are some global variables declared below, e.g., ``RENDER_ENDPOINT``.
    It is acceptable to use simple "readonly" global variables like strings and
    booleans, but in general global variables cannot be changed or have their
    state shared between processes.  Do not attempt to overwrite their values
    during server execution -- halt the server, change the values, and restart.

    For example, avoid creating a list or dictionary at module scope and
    modifying it from within one of the methods below to save any kind state.
    This will only work in the single threaded development server provided by
    flask, but will not be coherent when using a wsgi wrapper such as
    ``gunicorn``.  If your needs require shared state between server processes,
    you will want to use a proper database or implement a scheme using
    multiprocessing.
"""
import argparse
import atexit
import datetime
from enum import Enum
from hashlib import sha256
from pathlib import Path
from io import BytesIO
import itertools
import shutil
import subprocess
import sys
import tempfile
from textwrap import dedent
from typing import Dict, Union, Tuple, TYPE_CHECKING

from flask import Flask, send_file

if TYPE_CHECKING:
    from flask import Request as request
else:
    from flask import request


app = Flask(__name__)
"""The main flask application."""

RENDER_ENDPOINT = "/render"
"""Where the client will upload files to / wait for an image response from.

If on the drake side of your application you seek to transmit files to an
alternative location, make sure to update this value accordingly.

Warning:
    If you change this value to ``/``, then you must delete or comment out the
    :func:`root` method below.
"""

CLEANUP = True
"""Whether or not the server should cleanup its local cache.

For debugging purposes, set this to ``True`` to prevent the deletion of scene
files / rendered images.  See also: :func:`delete_server_cache`.
"""

THIS_FILE_DIR = Path(__file__).parent.absolute()
"""The directory containing this file."""

SERVER_CACHE = THIS_FILE_DIR / "cache"
"""Where to store data files during the execution of the program."""
try:
    SERVER_CACHE.mkdir(exist_ok=True)
except Exception as e:
    sys.stderr.write(
        f"Unable to create cache directory '{str(SERVER_CACHE)}': {e}\n"
    )
    sys.exit(1)


###############################################################################
# Helper classes and methods.
###############################################################################
def compute_hash(path: Union[str, Path], block_size: int = 1024 * 1024) -> str:
    """Incrementally compute and return the sha256 of the specified ``path``.

    Args:
        path: The file to compute the sha256 hash of.
        block_size: Size of block in bytes to read.  Default: ``1024 * 1024``.

    Raises:
        ValueError: If the ``block_size`` is not greater than 0.
        IOError: If the file cannot be loaded.

    Returns:
        The computed sha256 sum as a string.
    """
    if block_size <= 0:
        raise ValueError(
            f"compute_hash: block_size={block_size} not valid, must be "
            "greater than 0."
        )

    h = sha256()
    with open(path, "rb") as f:
        while True:
            data = f.read(block_size)
            if not len(data):
                break

            h.update(data)

    return h.hexdigest()


@atexit.register
def delete_server_cache():
    """Delete the :data:`SERVER_CACHE` folder upon exit (e.g., ``ctrl+C``).
    See also: :data:`CLEANUP`.
    """
    if CLEANUP:
        if SERVER_CACHE.is_dir():
            shutil.rmtree(SERVER_CACHE, ignore_errors=True)


class RenderError(Exception):
    """An exception class used for signaling to calling methods what type of
    message and HTTP response code should be sent back to the client.  Any
    issues that arise in the :func:`render_callback` should raise this type of
    error populating with an informative ``message`` and ``code``.  The calling
    code will communicate this message and code back to the client.

    Attributes:
        message (str): The exception message to communicate to the client.
        code (int): The HTTP response code to communicate to the client.  If
            not provided, it will be ``400``.  If a value of less than ``400``
            is provided, it will be set to ``500`` (internal server error).
            This exception is only to be used to indicate failure.
    """

    def __init__(self, message: str, code: int = 400):
        super().__init__(message)
        self.message = message
        self.code = code
        # Do not allow successful HTTP codes to be used, this is an Error.
        if self.code < 400:
            self.code = 500

    def flask_json_code_response(self, **kwargs):
        """Return a ``(json_dict, http_return_code)`` response populated with::

            (
                {
                    error=True,
                    message=self.message,
                    code=self.code,
                    **kwargs
                },
                self.code
            )

        Flask supports returning a (dictionary, integer http code) from a
        request, this method simply creates this return.  Users may provide
        additional ``**kwargs`` if needed, however generally speaking the
        relevant information to communicate to the client should be provided
        in ``self.message`` at the time the exception was raised.
        """
        return (
            {
                "error": True,
                "message": self.message,
                "code": self.code,
                **kwargs,
            },
            self.code,
        )


class FieldType(Enum):
    """A simple enumeration to symbolically describe the type expected in a
    given ``<form>`` field entry.
    """

    File = 0
    """Represents ``<input type="file" ...>``."""
    String = 1
    """Represents ``<input type="text" ...>``."""
    Int = 2
    """Represents ``<input type="number" ...>``."""
    Float = 3
    """Represents ``<input type="number" ...>``."""


def html_input_type(field_type: FieldType) -> str:
    """Return the value to use for ``type`` in an html ``<input>`` tag."""
    if field_type == FieldType.File:
        return "file"
    elif field_type in {FieldType.Int, FieldType.Float}:
        return "number"
    else:  # field_type := FieldType.String
        return "text"


class RenderRequest:
    """A ``RenderRequest`` instance wraps around the ``flask.request`` provided
    to :func:`render_endpoint` and validates all of the data provided by the
    client on the anticipated ``<form>`` entries.  In the
    :func:`render_callback` method, a user may assume that the following
    attributes have all been validated and their values are sensible.

    Note:
        When using functions such as :func:`python:subprocess.run` or
        :func:`python:subprocess.check_call`, be sure to convert any non-string
        attributes to a string in the list of command line arguments.  They are
        kept as numeric types in the attributes to facilitate a user performing
        any additional logic they need with the provided parameters before
        issuing the call to their renderer (e.g., creating a perspective
        projection matrix).

    Args:
        flask_request (``flask.request``): The flask request instance provided
            to the :func:`render_endpoint` method.
        log_to_console: Whether or not there should be any ``print``
            statements to the server console, e.g., to where the file uploaded
            by the client has been saved.  Default: ``True`` (useful for
            debugging).

    Attributes:
        request (``flask.request``): The input ``flask_request`` saved for
            availability in the event a user desires to perform additional
            validation or manipulation.
        log_to_console (bool): The input ``log_to_console``.
        scene_path (pathlib.Path): The path to where the uploaded file has
            been saved.
        scene_sha256 (str): The sha256 hash of the scene file stored at
            ``self.scene_path``.  This value is validated after loading the
            scene file from the ``<form>``.
        image_type (str): The type of scene being rendered.  One of ``color``,
            ``depth``, or ``label``.
        min_depth (float): The minimum depth range of the sensor.  Default is
            ``-1.0``, its value is only populated when
            :attr:`~RenderRequest.image_type` is `"depth"`.
        max_depth (float): The maximum depth range of the sensor.  Default is
            ``-1.0``, its value is only populated when
            :attr:`~RenderRequest.image_type` is `"depth"`.
        width (int): The width of the desired rendered image in pixels.
        height (int): The height of the desired rendered image in pixels.
        near (float): The near clipping plane of the camera.
        far (float): The far clipping plane of the camera.
        focal_x (float): The focal length x, in pixels.
        focal_y (float): The focal length y, in pixels.
        fov_x (float): The field of view in the x-direction (in radians).
        fov_y (float): The field of view in the y-direction (in radians).
        center_x (float): The principal point's x coordinate in pixels.
        center_y (float): The principal point's y coordinate in pixels.

    Raises:
        RenderError: If anything goes wrong during construction from the
            provided request, such as missing ``<form>`` entries, or invalid
            values such as ``image_type="depth"`` but no ``min_depth`` or
            ``max_depth`` provided.  The method constructing instances of this
            class (:func:`render_endpoint`) should check for exceptions raised
            and provide the specified error response to the client.
    """

    FORM_FIELD_TO_DESCRIPTION: Dict[str, Tuple[FieldType, str]] = {
        "scene": (
            FieldType.File,
            "the scene file to download from the client and render",
        ),
        "scene_sha256": (
            FieldType.String,
            "the sha256 hash of the file uploaded in form field `scene`",
        ),
        "image_type": (
            FieldType.String,
            "the type of scene being rendered, one of `color`, `depth`, or "
            "`label`",
        ),
        "min_depth": (
            FieldType.Float,
            "the minimum depth range of the sensor, only meaningful when "
            "image_type=depth",
        ),
        "max_depth": (
            FieldType.Float,
            "the maximum depth range of the sensor, only meaningful when "
            "image_type=depth",
        ),
        "width": (FieldType.Int, "the desired width of the rendered image"),
        "height": (FieldType.Int, "the desired height of the rendered image"),
        "near": (
            FieldType.Float,
            "the near clipping plane of the render camera",
        ),
        "far": (
            FieldType.Float,
            "the far clipping plane of the render camera",
        ),
        "focal_x": (FieldType.Float, "the focal length x, in pixels"),
        "focal_y": (FieldType.Float, "the focal length y, in pixels"),
        "fov_x": (
            FieldType.Float,
            "the field of view in the x-direction, in radians",
        ),
        "fov_y": (
            FieldType.Float,
            "the field of view in the y-direction, in radians",
        ),
        "center_x": (
            FieldType.Float,
            "the principal point's x coordinate, in pixels",
        ),
        "center_y": (
            FieldType.Float,
            "the principal point's y coordinate, in pixels",
        ),
        # NOTE: this input is not actually validated.
        "submit": (FieldType.String, "submit button"),
    }
    """A mapping of string form field keys to (:class:`FieldType`, string
    descriptions) of the field.  Used in the ``_parse_*`` methods to produce
    meaningful errors, as well as in the :func:`render_endpoint` to produce the
    ``GET`` listing of expected entries in the ``<form>``
    """

    def __init__(self, flask_request: request, log_to_console: bool = True):
        self.request = flask_request
        self.log_to_console = log_to_console

        self._error_on_extra_form_entries()

        # Parse and validate the scene related form field entries.
        self.scene_sha256 = self._parse_string("scene_sha256")
        self.image_type = self._parse_string("image_type")
        if self.image_type not in {"color", "depth", "label"}:
            raise RenderError(
                f"Field image_type='{self.image_type}' is not valid, must be "
                "one of 'color', 'depth', or 'label'."
            )

        # Parse and validate the uploaded scene file.
        self.scene_path = self._parse_scene()

        # Parse and validate the provided depth range.
        if self.image_type == "depth":
            self.min_depth = self._parse_positive_number("min_depth")
            self.max_depth = self._parse_positive_number("max_depth")
            if self.min_depth >= self.max_depth:
                raise RenderError(
                    f"Form fields min_depth='{self.min_depth}' and max_depth="
                    f"'{self.max_depth}' are not valid, min_depth must be "
                    "less than max_depth."
                )
        else:
            # Always make these attributes available.
            self.min_depth = -1.0
            self.max_depth = -1.0

        # Parse and validate the provided image dimensions to render.
        self.width = self._parse_positive_number("width")
        self.height = self._parse_positive_number("height")

        # Parse and validate the provided render camera clipping planes.
        self.near = self._parse_positive_number("near")
        self.far = self._parse_positive_number("far")
        if self.near >= self.far:
            raise RenderError(
                f"Form fields near='{self.near}' and far='{self.far}' not"
                "valid, the near plane must be in front of the far plane."
            )

        # Parse and validate the camera intrinsics.  Generally, the only check
        # available is positivity (other than center_x and center_y) since the
        # meaning of the value cannot be determined as "reasonable" here.
        self.focal_x = self._parse_positive_number("focal_x")
        self.focal_y = self._parse_positive_number("focal_y")
        self.fov_x = self._parse_positive_number("fov_x")
        self.fov_y = self._parse_positive_number("fov_y")
        self.center_x = self._parse_positive_number("center_x")
        self.center_y = self._parse_positive_number("center_y")
        # NOTE: same validation as in systems/sensors/camera_info.cc.
        if self.center_x >= self.width or self.center_y >= self.height:
            raise RenderError(
                f"Form fields center_x='{self.center_x}' and center_y="
                f"'{self.center_y}' are not coherent.  center_x must be less "
                f"than the provided width='{self.width}', and center_y must "
                f"be less than the provided height='{self.height}'."
            )

    def _error_on_extra_form_entries(self):
        """Error if extra form entries have been provided.  Should be done
        immediately to help a consumer identify e.g., mis-spelled entries.

        Raises:
            RenderError: If any extra ``<form>`` entries are found.
        """
        expected_fields = set(
            [field for field in self.FORM_FIELD_TO_DESCRIPTION]
        )
        provided_fields = set(
            [
                field
                for field in itertools.chain(
                    self.request.files, self.request.form
                )
            ]
        )
        extras = provided_fields - expected_fields
        if extras:
            s = "s" if len(extras) > 1 else ""
            raise RenderError(
                f"Extra <form> field{s} provided: {', '.join(extras)}"
            )

    def _check_type(
        self,
        field: str,
        field_type: FieldType,
        *allowed_field_types: FieldType,
    ):
        """Validate if the provided ``field_type`` is in the provided
        ``*allowed_field_types``.

        Args:
            field: the field being parsed, included for a better error message.
            field_type: the reported FieldType of ``field``
            allowed_field_types: the list of field types supported by the
                 calling method trying to parse this ``field``.

        Raises:
            ValueError: if ``field_type not in allowed_field_types``.
        """
        if field_type not in allowed_field_types:
            raise ValueError(
                f"Unable to parse field {field} with type {field_type} as one "
                f"of {','.join(str(aft) for aft in allowed_field_types)}."
            )

    def _extract_field_from_form(self, field: str, description: str) -> str:
        """Return the field from ``self.request.form[field]`` if found.  Helper
        method for ``_parse*`` methods to report consistent error messages.

        Args:
            field: The field to extract from the ``<form>``.
            description: The string description of the ``field`` used in
                reporting error messages.

        Raises:
            RenderError: If the ``field`` is not found in the form.
        """
        if field not in self.request.form:
            raise RenderError(
                f"Expected form field '{field}' ({description})."
            )
        return self.request.form[field]

    def _parse_string(self, field: str) -> str:
        """Parse and return the requested ``field`` from ``self.request.form``
        as a string.

        Args:
            field: The field name to extract from ``self.request.form``.

        Raises:
            RenderError: In the event that ``field`` is not provided, not able
                to be parsed, or any other issue arises from trying to obtain
                the value from the ``<form>``.
        """
        try:
            field_type, description = self.FORM_FIELD_TO_DESCRIPTION[field]
            self._check_type(field, field_type, FieldType.String)
            value = self._extract_field_from_form(field, description)
            return value
        except RenderError as re:
            raise re from None  # forward the exception as is
        except Exception as e:
            raise RenderError(
                f"Internal server error processing field '{field}': {e}",
                code=500,
            )

    def _parse_positive_number(self, field: str) -> Union[int, float]:
        """Parse and return the requested ``field`` from ``self.request.form``
        as a positive number.

        Args:
            field: The field name to extract from ``self.request.form``.

        Raises:
            RenderError: In the event that ``field`` is not provided, not able
                to be parsed, its value is not greater than 0, or any other
                issue arises from trying to obtain the value from the
                ``<form>``.
        """
        try:
            field_type, description = self.FORM_FIELD_TO_DESCRIPTION[field]
            self._check_type(field, field_type, FieldType.Int, FieldType.Float)
            value = self._extract_field_from_form(field, description)
            try:
                if field_type == FieldType.Int:
                    numeric_value: Union[int, float] = int(value)
                else:  # field_type := FieldType.Float
                    numeric_value = float(value)
                if not numeric_value > 0:
                    raise ValueError
                return numeric_value
            except ValueError:
                raise RenderError(
                    f"Form field {field} ({description}) with value `{value}` "
                    "is not valid."
                )
        except RenderError as re:
            raise re from None  # forward the exception as is
        except Exception as e:
            raise RenderError(
                f"Internal server error processing field '{field}': {e}",
                code=500,
            )

    def _parse_scene(self) -> Path:
        """Parse and validate the uploaded scene file, returning the path to
        where it was saved in the :data:`SERVER_CACHE`.

        Warning:
            It is assumed that :attr:`~RenderRequest.scene_sha256` and
            :attr:`~RenderRequest.image_type` have already been parsed and
            validated.

        Raises:
            RenderError: In the event that any of the expected entries are not
                provided by the ``<form>``, the sha256 hash of provided is not
                the same as what is computed from the uploaded file, or any
                other errors that occur in trying to move the file to the
                :data:`SERVER_CACHE`.
        """
        if "scene" in self.request.form:
            raise RenderError("Form field `scene` should be a file.")
        if "scene" not in self.request.files:
            raise RenderError(
                "Expected field `scene`, which should be a file."
            )

        try:
            # This will be a werkzeug.datastructures.FileStorage, see
            # https://werkzeug.palletsprojects.com/en/2.0.x/datastructures/
            scene_data = self.request.files["scene"]
            _, temp_path_str = tempfile.mkstemp(dir=SERVER_CACHE)
            scene_data.save(temp_path_str)

            # Compute the and validate the sha256 hash of the uploaded scene.
            temp_path = Path(temp_path_str)
            sha = compute_hash(temp_path)
            if sha != self.scene_sha256:
                # Delete the file before erroring out.
                if CLEANUP:
                    try:
                        temp_path.unlink()
                    except Exception:
                        pass
                raise RenderError(
                    f"Provided scene_sha256='{self.scene_sha256}' does not "
                    f"match the computed sha256 of '{sha}'.  Possible upload "
                    "failure.",
                    code=500,
                )

            # Create a timestamp for saving the file to avoid collisions.
            curr_time = datetime.datetime.now()
            curr_time_str = curr_time.strftime("%Y-%m-%d_%H-%M-%S-%f")
            scene_description = f"{self.image_type}-{self.scene_sha256}"
            scene_path = (
                SERVER_CACHE / f"{curr_time_str}-{scene_description}.gltf"
            )
            temp_path.rename(scene_path)

            # Print to the console what was just saved, including the mime type
            # if it has been provided.  Users may choose to fail the upload in
            # the event of an unsupported mime type being provided, however in
            # general relying on accurate mime type reporting is not advisable.
            if self.log_to_console:
                message = (
                    f"==> Scene file with image_type={self.image_type} saved "
                    f"to '{str(scene_path)}' (sha256={sha})."
                )
                mime_type = getattr(scene_data, "content_type", None)
                if mime_type is not None:
                    message += (
                        f"  Client reported a mime type of '{mime_type}'."
                    )

            return scene_path
        except RenderError as re:
            raise re from None  # forward the error
        except Exception as e:
            raise RenderError(f"Internal server error: {e}", code=500)


###############################################################################
# The main rendering call, replace with your own desired implementation.
###############################################################################
def render_callback(render_request: RenderRequest) -> Union[Path, str]:
    """Invoke the renderer to produce and return an output image path.

    The ``render_request`` parameter's attributes have already been validated
    based off of the current flask request.  This method is responsible for
    determining its final output image path and file extension.

    Args:
        render_request (RenderRequest): The validated flask request wrapped in
            a :class:`RenderRequest` instance.

    Returns:
        A successful call should return the path to the final output rendered
        image file.

    Raises:
        RenderError: In the event that anything goes wrong during the call to
            the renderer, a :class:`RenderError` should be raised.  The method
            calling this render callback will look for this specific kind of
            exception first, and forward the corresponding error response to
            the client.  Any other exceptions raised will result in an internal
            server error response message to the client.
    """
    # NOTE: the path to `gltf_render_server_backend` is only valid from bazel.
    # The path to the renderer executable to call.
    backend = str(
        (THIS_FILE_DIR / ".." / "gltf_render_server_backend").resolve()
    )
    # Determine where to store this rendering.  The render_request.scene_path
    # will already live within the SERVER_CACHE, appending a file extension
    # to render to is acceptable.
    #
    # NOTE: when CLEANUP=True, this image path is deleted by the caller.
    output_path = str(render_request.scene_path)
    if render_request.image_type in {"color", "label"}:
        output_path += ".png"
    else:  # render_request.image_type := "depth"
        output_path += ".tiff"

    # Create the command-line arguments to pass to the render backend.
    # NOTE: make sure you convert entries in the list to `str`.
    proc_args = [
        backend,
        "--input",
        str(render_request.scene_path),
        "--output",
        output_path,
        "--image_type",
        str(render_request.image_type),
        "--width",
        str(render_request.width),
        "--height",
        str(render_request.height),
        "--near",
        str(render_request.near),
        "--far",
        str(render_request.far),
        "--focal_x",
        str(render_request.focal_x),
        "--focal_y",
        str(render_request.focal_y),
        "--fov_x",
        str(render_request.fov_x),
        "--fov_y",
        str(render_request.fov_y),
        "--center_x",
        str(render_request.center_x),
        "--center_y",
        str(render_request.center_y),
    ]
    if render_request.image_type == "depth":
        proc_args.extend(
            [
                "--min_depth",
                str(render_request.min_depth),
                "--max_depth",
                str(render_request.max_depth),
            ]
        )

    # Log to the console what command is about to be run for debugging.
    if render_request.log_to_console:
        print(f"$ {' '.join(proc_args)}")

    # Call the render backend, including capturing any errors.
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

        # Inform the calling method where the final rendering resides.
        return output_path
    except Exception as e:
        raise RenderError(f"Failed render invocation: {e}", code=500)


###############################################################################
# Flask endpoint implementations.
###############################################################################
@app.route("/")
def root():
    """The main listing page, renders a simple redirect page including where
    the server cache lives for development.  This endpoint (``/``) is not
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
            f"{indent}  lives here: <tt>{str(SERVER_CACHE)}</tt>\n"
            f"{indent}</p>\n"
        )
    else:
        html_interior = ""

    return f"{html_prefix}{html_interior}{html_suffix}"


@app.route(RENDER_ENDPOINT, methods=["GET", "POST"])
def render_endpoint():
    """The main rendering endpoint for the client to communicate with.

    A client request ``<form>`` is validated and provided to
    :func:`render_callback`, with the resultant image transmitted back to the
    client.
    """
    if request.method == "POST":
        try:
            # Validate the request and render the image.
            render_request = RenderRequest(request)
            output_image = Path(render_callback(render_request))
            if render_request.log_to_console:
                print(f"==> Rendered image: {str(output_image)}")

            # Now that the image is rendered, it is safe to delete the scene.
            if CLEANUP:
                if render_request.log_to_console:
                    scene_str = str(render_request.scene_path)
                    print(f"==> Deleting scene file {scene_str}.")
                render_request.scene_path.unlink(missing_ok=True)

            # The mime type response is only populated based off the file
            # extension, implementation of render_callback is responsible for
            # invalid file extensions / mime types being reported.
            #
            # NOTE: the client does *NOT* rely on the mime type.
            mime_type = None
            image_extension = output_image.suffix.lower()
            if image_extension == "png":
                mime_type = "image/png"
            elif image_extension in {"tif", "tiff"}:
                mime_type = "image/tiff"

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
                if render_request.log_to_console:
                    print(f"==> Deleting rendering {str(output_image)}.")
                output_image.unlink(missing_ok=True)
                return send_file(
                    buffer,
                    mimetype=mime_type,
                    # NOTE: do not reveal internal details of the server.
                    attachment_filename=f"rendering.{image_extension}",
                )
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
    # Generate the <table> entries indented properly.
    tr_indent = "  " * 4
    td_indent = "  " * 5
    table_rows = []
    for field, (
        field_type,
        description,
    ) in RenderRequest.FORM_FIELD_TO_DESCRIPTION.items():
        row = f"{tr_indent}<tr>\n"
        row += f"{td_indent}<td>{description}</td>\n"
        input_type = html_input_type(field_type)
        row += f'{td_indent}<td><input type="{input_type}" name="{field}">\n'
        row += f"{tr_indent}</tr>\n"
        table_rows.append(row)

    return f"{html_prefix}{''.join(table_rows)}{html_suffix}"


def main():
    parser = argparse.ArgumentParser(
        description="Prototype glTF Render Server"
    )
    parser.add_argument(
        "--host",
        type=str,
        required=False,
        default="127.0.0.1",
        help="URL to host on, default: 127.0.0.1.  Input not validated.",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=False,
        default=8000,
        help="Port to host on, default: 8000.  Input not validated.",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Run in debug mode, flask reloads the server when file changes.",
    )

    args = parser.parse_args()
    app.run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
