"""Tests generation of Drake's Python Interface files (.pyi)."""

import os
import unittest

from mypy import stubgen

# Mypy can time out if importing takes an inordinate length of time. Try to
# avoid this by importing ourselves up front when the import isn't being run
# under a timeout.
import pydrake.all


class TestStubgen(unittest.TestCase):
    # TODO(mwoehlke-kitware): test the already-generated files instead.
    def test_generation(self):
        """Ensure that stubgen runs and generates output.

        For now, this is more or less just a smoke test, with a very cursory
        check that the output is 'reasonable'.
        """
        output_dir = os.environ['TEST_TMPDIR']
        args = ['--package', 'pydrake', '--output', output_dir]

        # Generate stubs.
        result = stubgen.main(args)
        self.assertTrue(result is None or result == 0)

        # Find some of the expected output and look for an expected function.
        expected = os.path.join(output_dir, 'pydrake', '__init__.pyi')
        found_expected_decl = False
        for line in open(expected, 'r'):
            if line.startswith('def getDrakePath():'):
                found_expected_decl = True
                break
        self.assertTrue(found_expected_decl)
