"""Validate Drake's Python Interface files (.pyi)."""

import os
import unittest

from bazel_tools.tools.python.runfiles import runfiles


class TestStubgen(unittest.TestCase):
    # TODO(mwoehlke-kitware): test the already-generated files instead.
    def test_contents(self):
        """Ensure that .pyi files contain 'reasonable' contents.

        For now, this is more or less just a smoke test, with a very cursory
        check on the contents.
        """
        manifest = runfiles.Create()

        # Get the base directory where our data files can be found.
        output_dir = manifest.Rlocation('drake/bindings/pydrake')

        # Find some of the expected output and look for an expected function.
        expected = os.path.join(output_dir, 'pydrake', '__init__.pyi')
        found_expected_decl = False
        for line in open(expected, 'r'):
            if line.startswith('def getDrakePath():'):
                found_expected_decl = True
                break
        self.assertTrue(found_expected_decl)
