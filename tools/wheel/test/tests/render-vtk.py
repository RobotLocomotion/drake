#!/usr/bin/env python3

import platform
import subprocess

# N.B. It should be safe to import pydrake without the EGL library. We shouldn't
# be dlopen-ing libEGL until the first time a renderer is constructed.
from pydrake.geometry import (
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
    SceneGraph,
)

if platform.system() == "Linux":
    os_release = platform.freedesktop_os_release()["ID"]
    if os_release == "ubuntu":
        subprocess.check_call("apt-get install -y libegl1", shell=True)
    elif os_release == "amzn":
        subprocess.check_call("dnf install -y libglvnd-egl", shell=True)
    else:
        raise NotImplementedError(os_release)

# Check that setting up rendering doesn't crash. (Our rendering code uses tricky
# linking and dlopen tactics, so is especially brittle.)
scene_graph = SceneGraph()
params = RenderEngineVtkParams()
renderer = scene_graph.AddRenderer("vtk", MakeRenderEngineVtk(params))
assert scene_graph.RendererCount() == 1
