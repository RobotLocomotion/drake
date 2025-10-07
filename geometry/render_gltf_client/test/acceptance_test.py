"""Performs minimal smoke tests to run binaries in this test folder."""

import json
import os
import subprocess
import unittest

from python.runfiles import Create as CreateRunfiles

# A minimal glTF file contains zero meshes.
# The `buffers` entry is required for vtkGLTFImporter and taken from:
# https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/gltfTutorial_003_MinimalGltfFile.md

MINIMAL_GLTF = """\
{
  "scene": 0,
  "cameras" : [
    {
      "perspective" :
      {
        "aspectRatio" : 1.3333333333333333,
        "yfov" : 0.78539816339744828,
        "zfar" : 10,
        "znear" : 0.01
      },
      "type" : "perspective"
    }
  ],
  "buffers" : [
    {
      "uri" : "data:application/octet-stream;base64,AAABAAIAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAAAAAACAPwAAAAA=",  # noqa
      "byteLength" : 44
    }
  ],
  "asset" : {
    "version" : "2.0"
  }
}
""".replace("# noqa", "")


class TestGltfRenderBinary(unittest.TestCase):
    def setUp(self):
        self.runfiles = CreateRunfiles()
        self.maxDiff = None

    def _check_call(self, *args):
        """Runs a subprocess and checks that it returns exitcode zero."""
        result = subprocess.run(
            *args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            encoding="utf-8",
        )
        if result.returncode != 0:
            # This is just to print the stdout conveniently.
            self.assertEqual(result.stdout, "")
        result.check_returncode()

    def test_server_vtk_backend(self):
        """Renders each type of images by invoking the vtk backend with test
        arguments."""

        server_vtk_backend = self.runfiles.Rlocation(
            "drake/geometry/render_gltf_client/server_vtk_backend"
        )

        # Fail-fast in case MINIMAL_GLTF has typos.
        json.loads(MINIMAL_GLTF)

        # Note: `tmp_dir` will be cleaned up by bazel after running the test.
        tmp_dir = os.path.join(
            os.environ.get("TEST_TMPDIR", "/tmp"), "server_vtk_backend"
        )
        os.makedirs(tmp_dir, exist_ok=True)
        gltf_input_path = os.path.join(tmp_dir, "test.gltf")
        with open(gltf_input_path, "w") as f:
            f.write(MINIMAL_GLTF)

        # Render each image type and ensure the return code is zero.
        for image_type, ext in zip(
            ["color", "depth", "label"], ["png", "tiff", "png"]
        ):
            image_output_path = os.path.join(tmp_dir, f"{image_type}.{ext}")
            proc_args = [
                server_vtk_backend,
                "--input_path",
                gltf_input_path,
                "--output_path",
                image_output_path,
                "--image_type",
                image_type,
                "--width",
                "64",
                "--height",
                "48",
                "--near",
                "0.01",
                "--far",
                "10.0",
                "--focal_x",
                "0.052227",
                "--focal_y",
                "0.052227",
                "--fov_x",
                "0.785398",  # 45 degrees FOV.
                "--fov_y",
                "0.785398",
                "--center_x",
                "32",
                "--center_y",
                "24",
            ]

            with self.subTest(image=f"{image_type}.{ext}"):
                self._check_call(proc_args)

    def test_server_demo(self):
        """A minimal smoke test to ensure running the server demo won't crash,
        e.g., the imports are correct.  It doesn't currently test receiving a
        flask request and exercising `render_callback()`."""

        server_demo = self.runfiles.Rlocation(
            "drake/geometry/render_gltf_client/server_demo"
        )
        subprocess.run([server_demo, "--acceptance_test"])

    def test_client_demo_client(self):
        """A minimal smoke test to run the client demo that launches a Drake
        simulation and a RenderEngineGltfClient. Since there is no server,
        we can only instantiate the simulator, not advance it."""
        client_demo = self.runfiles.Rlocation(
            "drake/geometry/render_gltf_client/client_demo"
        )
        proc_args = [
            client_demo,
            "--render_engine=client",
            "--simulation_time=-1",  # N.B. Negative means don't call AdvanceTo.
        ]
        self._check_call(proc_args)

    def test_client_demo_vtk(self):
        """A minimal smoke test to run the client demo that launches a Drake
        simulation and a RenderEngineVtk for a short period of time."""
        client_demo = self.runfiles.Rlocation(
            "drake/geometry/render_gltf_client/client_demo"
        )
        proc_args = [
            client_demo,
            "--render_engine=vtk",
            "--simulation_time=0.1",
        ]
        self._check_call(proc_args)
