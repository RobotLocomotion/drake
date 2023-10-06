from pathlib import Path
import subprocess
import unittest

from pydrake.common import (
    FindResourceOrThrow,
    temp_directory,
)


class TestRefineMesh(unittest.TestCase):

    def assert_files_different(self, a_path, b_path):
        with open(a_path) as a, open(b_path) as b:
            self.assertNotEqual(a.read(), b.read())

    def test_refinement(self):
        refiner = FindResourceOrThrow(
            "drake/geometry/proximity/refine_mesh")
        mesh_file = FindResourceOrThrow(
            "drake/geometry/test/one_tetrahedron.vtk")

        # Refining one tetrahedron should yield a different mesh.
        tmpdir = Path(temp_directory())
        direct_result_file = tmpdir / "refined.vtk"
        subprocess.run(
            [refiner, mesh_file, direct_result_file], check=True)
        self.assert_files_different(mesh_file, direct_result_file)

        # Refining an already refined mesh should be a no-op, and not write the
        # output file.
        repeat_result_file = tmpdir / "repeat.vtk"
        subprocess.run(
            [refiner, direct_result_file, repeat_result_file], check=True)
        self.assertFalse(repeat_result_file.exists())
