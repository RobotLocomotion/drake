# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import os
import subprocess
import sys

from pydrake.common import set_log_level

# Proxy http://DEEPNOTE_PROJECT_ID:8080/PORT/ to http://127.0.0.1:PORT/ so
# that multiple notebooks can all be served via Deepnote's only open port.
#
# For conf documentation, see https://www.nginx.com/resources/wiki/start/.
_NGINX_CONF = """
# Deepnote MeshCat proxy server configuration.
server {
  listen 8080 default_server;
  listen [::]:8080 default_server;
  root /var/www/html;
  server_name _;
  location ~ /(7[0-9][0-9][0-9])/(.*) {
    proxy_pass http://127.0.0.1:$1/$2;
  }
}
"""


def _run(args):
    proc = subprocess.run(
        args, encoding="utf-8", stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    if proc.returncode == 0:
        return
    print(proc.stdout, file=sys.stderr, end="")
    proc.check_returncode()


def _setup_deepnote_nginix():
    print("Setting up Deepnote MeshCat NginX server...")
    _run(["apt-get", "update"])
    _run(["apt-get", "install", "--no-install-recommends", "nginx-light"])
    _run(["rm", "-f", "/etc/nginx/sites-enabled/default"])
    conf_filename = "/etc/nginx/sites-available/deepnote-meshcat-proxy"
    with open(conf_filename, "w") as conf:
        conf.write(_NGINX_CONF)
    _run(["ln", "-sf", conf_filename, "/etc/nginx/sites-enabled/"])
    _run(["service", "nginx", "start"])


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
    prev_log_level = set_log_level("warn")
    use_ngrok = False
    if ("DEEPNOTE_PROJECT_ID" in os.environ):
        # Deepnote exposes port 8080 (only).  If we need multiple meshcats,
        # then we fall back to ngrok.
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
