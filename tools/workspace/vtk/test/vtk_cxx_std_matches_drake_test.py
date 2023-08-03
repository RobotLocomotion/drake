import re
import sys
import unittest
from pathlib import Path

from bazel_tools.tools.python.runfiles import runfiles

sys.path.insert(0, str(Path(__file__).parent.parent / "image"))
from vtk_cmake_configure_args import cxx_std  # noqa: E402


class TestVtkCxxStdMatchesDrake(unittest.TestCase):
    def test_all_supported_platforms(self):
        codenames = [
            "focal",
            "jammy",
            "mac",
        ]
        for codename in codenames:
            with self.subTest(codename=codename):
                self._check(codename)

    def _check(self, codename):
        if codename == "mac":
            resource = "drake/tools/macos.bazelrc"
        else:
            resource = f"drake/tools/ubuntu-{codename}.bazelrc"
        bazelrc_path = Path(runfiles.Create().Rlocation(resource)).resolve()
        with self.subTest(bazelrc_path=bazelrc_path):
            drake_cxx_std = self._parse_cxx_std_from_bazelrc(bazelrc_path)
            vtk_cxx_std = cxx_std(codename)
            self.assertEqual(vtk_cxx_std, drake_cxx_std)

    def _parse_cxx_std_from_bazelrc(self, bazelrc_path: Path):
        """Returns Drake's C++ standard based on the given bazelrc file."""
        with open(bazelrc_path) as f:
            for line in f:
                match = re.match(r"^build --cxxopt=-std=c\+\+(\S*)", line)
                if match is not None:
                    return match.groups()[0]
        self.fail("Could not find C++ std in bazelrc")
