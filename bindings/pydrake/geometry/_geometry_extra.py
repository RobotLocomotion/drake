# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

# ruff: noqa: F821 (undefined-name). This file is only a fragment.

import os
import socket
import subprocess
import sys


def _is_listening(port):
    """Returns True iff the port number (on localhost) is listening for
    connections.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return sock.connect_ex(("127.0.0.1", port)) == 0
    finally:
        sock.close()


def _restart_nginx_service():
    """(Re)starts Drake Deepnote's nginx proxy."""
    print("(Re)starting NginX server for MeshCat on Deepnote...")
    proc = subprocess.run(
        ["service", "nginx", "restart"],
        encoding="utf-8",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if proc.returncode == 0:
        return
    print(proc.stdout, file=sys.stderr, end="")
    proc.check_returncode()


def _start_meshcat_deepnote(*, params=None, restart_nginx=False):
    """Returns a Meshcat object suitable for use on Deepnote's cloud.
    The optional arguments are not available to end users but might be helpful
    when debugging this function.
    """
    from IPython.display import HTML, display

    host = os.environ["DEEPNOTE_PROJECT_ID"]
    if params is None:
        params = MeshcatParams()
    params.web_url_pattern = f"https://{host}.deepnoteproject.com/{{port}}/"
    if restart_nginx or not _is_listening(8080):
        _restart_nginx_service()
    meshcat = Meshcat(params=params)
    url = meshcat.web_url()
    display(HTML(f"Meshcat URL: <a href='{url}' target='_blank'>{url}</a>"))
    return meshcat


def StartMeshcat():
    """
    Constructs a Meshcat instance, with extra support for Deepnote.

    On most platforms, this function is equivalent to simply constructing a
    ``pydrake.geometry.Meshcat`` object with default arguments.

    On Deepnote, however, this does extra work to expose Meshcat to the public
    internet by setting up a reverse proxy for the single available network
    port. To access it, you must enable "Allow incoming connections" in the
    Environment settings pane and use an Init notebook with NginX similar to
    https://github.com/RobotLocomotion/drake/blob/v1.53.0/doc/_pages/release_playbook.md?plain=1#L234-L279

    (Drake no longer uses Deepnote, but we've kept the code for it here intact
    for any downstream users who are still using it.)
    """
    if "DEEPNOTE_PROJECT_ID" in os.environ:
        return _start_meshcat_deepnote()
    return Meshcat()


def _add_extraneous_repr_functions():
    """Defines repr functions for various classes in common where defining it
    in python is simply more convenient.
    """

    def in_memory_mesh_repr(mesh):
        # If defined, the supporting string has to provide the preceding comma,
        # and space, along with the parameter name and its value.
        supporting_string = ""
        if mesh.supporting_files:
            supporting_string = f", supporting_files={mesh.supporting_files!r}"
        return f"InMemoryMesh(mesh_file={mesh.mesh_file!r}{supporting_string})"

    InMemoryMesh.__repr__ = in_memory_mesh_repr

    def mesh_source_repr(source):
        if source.is_path():
            param_str = f"path={str(source.path())!r}"
        else:
            param_str = f"mesh={source.in_memory()!r}"
        return f"MeshSource({param_str})"

    MeshSource.__repr__ = mesh_source_repr

    def mesh_or_convex_repr(mesh, type_name):
        if mesh.source().is_path():
            data_param = f"filename={str(mesh.source().path())!r}"
        else:
            data_param = f"mesh_data={mesh.source().in_memory()!r}"
        # Convert array to list to ease converting repr to mesh type.
        return f"{type_name}({data_param}, scale3={mesh.scale3().tolist()!r})"

    def mesh_repr(mesh):
        return mesh_or_convex_repr(mesh, "Mesh")

    def convex_repr(convex):
        return mesh_or_convex_repr(convex, "Convex")

    Mesh.__repr__ = mesh_repr
    Convex.__repr__ = convex_repr


_add_extraneous_repr_functions()
