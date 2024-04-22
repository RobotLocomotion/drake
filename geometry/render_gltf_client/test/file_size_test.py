import glob
import os
import unittest

from python.runfiles import Create as CreateRunfiles

MAXIMUM_FILE_SIZE = 50000  # 50KB.


class TestFileSize(unittest.TestCase):
    """Ensures the file size of the test resources, e.g., PNG or glTF files, in
    the test folder is reasonable. This test guards the developers from
    accidentally committing large files when updating them.
    """

    def setUp(self):
        runfiles = CreateRunfiles()
        self.test_folder = runfiles.Rlocation(
            "drake/geometry/render_gltf_client/test"
        )

    def test_gltf_file_size(self):
        gltf_files = glob.glob(os.path.join(self.test_folder, "*.gltf"))
        for gltf in gltf_files:
            with self.subTest(filename=os.path.basename(gltf)):
                self.assertLess(os.path.getsize(gltf), MAXIMUM_FILE_SIZE)

    def test_image_file_size(self):
        image_files = glob.glob(
            os.path.join(self.test_folder, "*.png")
        ) + glob.glob(os.path.join(self.test_folder, "*.tiff"))
        for image in image_files:
            with self.subTest(filename=os.path.basename(image)):
                self.assertLess(os.path.getsize(image), MAXIMUM_FILE_SIZE)
