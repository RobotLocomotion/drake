#!/usr/bin/env python3

from pathlib import Path
import subprocess
import sys

from pydrake.geometry import (
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
    SceneGraph,
)

platform_details = Path("/etc/os-release").read_text().splitlines()
if "ID=ubuntu" not in platform_details:
    print("Skipping render-vtk test on non-Ubuntu")
    sys.exit(0)

subprocess.check_call(["/usr/bin/apt-get install -y libegl1"], shell=True)

scene_graph = SceneGraph()
params = RenderEngineVtkParams()
renderer = scene_graph.AddRenderer("vtk", MakeRenderEngineVtk(params))
assert scene_graph.RendererCount() == 1
