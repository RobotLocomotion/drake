# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import os
import socket
import subprocess
import sys

from pydrake.common import FindResourceOrThrow


def _is_listening(port):
    """Returns True iff the port number (on localhost) is listening for
    connections.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return sock.connect_ex(("127.0.0.1", port)) == 0
    finally:
        sock.close()


def _install_deepnote_nginx():
    """Uses Ubuntu to install the NginX web server and configures it to serve
    as a reverse proxy for MeshCat on Deepnote. The server will proxy
    https://DEEPNOTE_PROJECT_ID:8080/PORT/ to http://127.0.0.1:PORT/ so
    that multiple notebooks can all be served via Deepnote's only open port.
    """
    print("Installing NginX server for MeshCat on Deepnote...")
    install_nginx = FindResourceOrThrow(
        "drake/setup/deepnote/install_nginx")
    proc = subprocess.run(
        [install_nginx], encoding="utf-8", stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    if proc.returncode == 0:
        return
    print(proc.stdout, file=sys.stderr, end="")
    proc.check_returncode()


def _start_meshcat_deepnote(*, params=None, restart_nginx=False):
    """Returns a Meshcat object suitable for use on Deepnote's cloud.
    The optional arguments are not available to end users but might be helpful
    when debugging this function.
    """
    from IPython.display import display, HTML
    host = os.environ["DEEPNOTE_PROJECT_ID"]
    if params is None:
        params = MeshcatParams()
    params.web_url_pattern = f"https://{host}.deepnoteproject.com/{{port}}/"
    if restart_nginx or not _is_listening(8080):
        _install_deepnote_nginx()
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
    Environment settings pane.
    """
    if "DEEPNOTE_PROJECT_ID" in os.environ:
        return _start_meshcat_deepnote()
    return Meshcat()
