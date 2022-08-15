"""Performs minimal smoke tests to run binaries in this test folder."""

import os
import subprocess
import unittest

from bazel_tools.tools.python.runfiles.runfiles import Create as CreateRunfiles

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
}
"""


class TestGltfRenderServer(unittest.TestCase):
    def setUp(self):
        self.runfiles = CreateRunfiles()

    def test_gltf_render_server_py(self):
        """A minimal smoke test to ensure running the binary won't crash, e.g.,
        the imports are correct.  It doesn't currently test receiving a flask
        request and exercising `render_callback()`."""

        server_py_binary = self.runfiles.Rlocation(
            "drake/geometry/render_gltf_client/gltf_render_server"
        )
        subprocess.run([server_py_binary, "--test"])

    def test_gltf_render_server_backend(self):
        """Renders each type of images by invoking the backend binary with
        test arguments."""

        server_backend_binary = self.runfiles.Rlocation(
            "drake/geometry/render_gltf_client/gltf_render_server_backend"
        )

        # Note: `tmp_dir` will be cleaned up by bazel after running the test.
        tmp_dir = os.path.join(
            os.environ.get("TEST_TMPDIR", "/tmp"), "gltf_render_server_test"
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
                server_backend_binary,
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

            proc = subprocess.run(proc_args, capture_output=True)
            self.assertTrue(proc.returncode == 0)
