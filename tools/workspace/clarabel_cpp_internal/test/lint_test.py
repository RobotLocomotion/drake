import unittest
from pathlib import Path


class ClarabelCppInternalLintTest(unittest.TestCase):

    def _read(self, filename):
        """Returns the contents of the given filename."""
        with open(filename, encoding="utf-8") as f:
            return f.read()

    def test_serialize_header(self):
        """Checks that serialize.h matches the desired gen_serialize.h."""
        mydir = Path("tools/workspace/clarabel_cpp_internal")
        actual = self._read(mydir / "serialize.h")
        expected = self._read(mydir / "gen_serialize.h")
        self.maxDiff = None
        self.assertMultiLineEqual(expected, actual)
