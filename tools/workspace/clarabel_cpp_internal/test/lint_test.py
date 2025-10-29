from pathlib import Path
import unittest


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
        # If the following check fails, run this command (from the root of a
        # Drake checkout) to re-generate the header file:
        #
        #  bazel run //tools/workspace/clarabel_cpp_internal:gen_serialize -- \
        #    --output=$(pwd)/tools/workspace/clarabel_cpp_internal/serialize.h
        #
        self.assertMultiLineEqual(expected, actual)
