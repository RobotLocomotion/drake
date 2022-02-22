# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import os
import sys
from pydrake.common import set_log_level


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
