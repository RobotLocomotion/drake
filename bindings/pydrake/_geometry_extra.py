# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import os
import socket
import subprocess
import sys

from pydrake.common import FindResourceOrThrow, set_log_level


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
    Constructs a Meshcat instance, with support for Deepnote and Google Colab.

    On most platforms, this method is equivalent to calling `meshcat =
    Meshcat()`. On Deepnote or Google Colab, this provides extra functionality,
    setting Meshcat to use the appropriate ports and to use ngrok if necessary.

    Note that the free/unregistered version of ngrok only allows two
    connections. On Google Colab, this means one can only have two viable
    Meshcat instances running per provisioned machine. On Deepnote, we have one
    additional port available (port 8080) which does not require ngrok; you
    must "Allow incoming connections" in the environment settings to use it.

    If you run out of available ports, you can reset the notebook to free any
    ports that are currently used by the notebook.

    Warning:
     Drake's support for Colab is deprecated.
     New releases after 2022-04-01 will no longer be compatible with Colab.
     See `drake#13391 <https://github.com/RobotLocomotion/drake/issues/13391>`_
     or `colab#1880 <https://github.com/googlecolab/colabtools/issues/1880>`_
     for details.
    """
    prev_log_level = set_log_level("warn")
    use_ngrok = False
    if ("DEEPNOTE_PROJECT_ID" in os.environ):
        # Deepnote exposes port 8080 (only).  If we need multiple meshcats,
        # then we fall back to ngrok.
        # TODO(jwnimmer-tri) Use _start_meshcat_deepnote() instead of ngrok,
        # once we have a bit more experience with it.
        try:
            meshcat = Meshcat(8080)
        except RuntimeError:
            use_ngrok = True
        else:
            set_log_level(prev_log_level)
            web_url = f"https://{os.environ['DEEPNOTE_PROJECT_ID']}.deepnoteproject.com"  # noqa
            print(f'Meshcat is now available at {web_url}')
            return meshcat

    if 'google.colab' in sys.modules:
        use_ngrok = True

    meshcat = Meshcat()
    web_url = meshcat.web_url()
    if use_ngrok:
        from pyngrok import ngrok
        http_tunnel = ngrok.connect(meshcat.port(), bind_tls=False)
        web_url = http_tunnel.public_url

    set_log_level(prev_log_level)
    print(f'Meshcat is now available at {web_url}')

    return meshcat
