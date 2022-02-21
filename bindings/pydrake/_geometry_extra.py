# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import os
import socket
import subprocess
import sys

from pydrake.common import FindResourceOrThrow, set_log_level


def _is_listening(port):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return sock.connect_ex(("127.0.0.1", port)) == 0
    finally:
        sock.close()


def _install_deepnote_nginx():
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


def _start_meshcat_deepnote(*, port=None, restart_nginx=False):
    host = os.environ["DEEPNOTE_PROJECT_ID"]
    if restart_nginx or not _is_listening(8080):
        _install_deepnote_nginx()
    params = MeshcatParams(
        port=port,
        web_url_pattern=f"https://{host}.deepnoteproject.com/{{port}}/")
    return Meshcat(params=params)


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
    """
    if "DEEPNOTE_PROJECT_ID" in os.environ:
        return _start_meshcat_deepnote()

    prev_log_level = set_log_level("warn")
    use_ngrok = False
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
