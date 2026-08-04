import re
import unittest

from python import runfiles


class TestEnum(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_enum_cross_check(self):
        """Checks that the Drake-created flavor of nlopt.cpp (via a patch file)
        is consistent with the upstream-generated flavor of same (via CMake).

        If this test fails during an NLopt version pin upgrade, you will need
        to update patches/gen_enums.patch with the reported differences.
        """
        # Load both input files.
        # "actual" refers to the the Drake-created flavor (via a patch file).
        # "expected" refers to the upstream-generated flavor (via CMake).
        manifest = runfiles.Create()
        actual_file = manifest.Rlocation("nlopt_internal/genrule/nlopt.hpp")
        with open(actual_file) as f:
            actual = f.read()
        expected_file = manifest.Rlocation(
            "drake/tools/workspace/nlopt_internal/test/nlopt-upstream.hpp"
        )
        with open(expected_file) as f:
            expected = f.read()

        # When CMake is processing the header file, it removes blank lines.
        # We will do the same to our actual file to prep for comparison.
        actual = actual.replace("\n\n\n", "\n")
        actual = actual.replace("\n\n", "\n")

        # CMake also does something inexplicable to tab-spaced macro line
        # endings. Canonicalize those in both files for comparison.
        actual = re.sub(r"\s+\\", r" \\", actual)
        expected = re.sub(r"\s+\\", r" \\", expected)

        # Compare.
        self.assertMultiLineEqual(expected, actual)
