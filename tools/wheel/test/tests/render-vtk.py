#!/usr/bin/env python3

from pathlib import Path
import platform
import subprocess

from pydrake.geometry import (
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
    SceneGraph,
)

if platform.system() == "Linux":
    os_release = Path("/etc/os-release").read_text().splitlines()
    if "ID=ubuntu" in os_release:
        subprocess.check_call("apt-get install libegl1 -y", shell=True)
    elif 'ID="amzn"' in os_release:
        subprocess.check_call("dnf install libglvnd-egl -y", shell=True)
    else:
        raise NotImplementedError(repr(os_release))

scene_graph = SceneGraph()
params = RenderEngineVtkParams()
renderer = scene_graph.AddRenderer("vtk", MakeRenderEngineVtk(params))
assert scene_graph.RendererCount() == 1
