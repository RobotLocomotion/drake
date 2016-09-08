#!/usr/bin/python

"""
spatial_inertia_solid_cuboid_test.py -- Unit test for
spatial_inertia_solid_cuboid.py.
"""

import os
import subprocess
import sys
import unittest


class TestSpatialInertiaCalculatorCuboid(unittest.TestCase):
    def get_absolute_filename(self, relative_filename):
        """Return an absolute filename, relative to this test program."""
        mydir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(mydir, relative_filename)

    def run_and_expect(self, args, expected_exitcode, expected_regexps=None):
        try:
            calculator = self.get_absolute_filename(
                "../spatial_inertia_solid_cuboid.py")
            output = subprocess.check_output(
                [sys.executable, calculator] + args,
                stderr=subprocess.STDOUT)
            returncode = 0
        except subprocess.CalledProcessError as e:
            output = e.output
            returncode = e.returncode
        self.assertEqual(returncode, expected_exitcode,
                         "output was:\n" + output)
        if expected_regexps is not None:
            for item in expected_regexps:
                self.assertRegexpMatches(output, item)

    def test_help(self):
        """
        Verifies that a help statement is printed when the calculator is passed
        a --help or -h command line flag.
        """
        self.run_and_expect(
            ['--help'],
            0,
            ["usage: spatial_inertia_solid_cuboid.py*"])

        self.run_and_expect(
            ['-h'],
            0,
            ["usage: spatial_inertia_solid_cuboid.py*"])

    def test_gazebo_cinder_block_spatial_inertia(self):
        """
        Verifies that the calculator outputs the expected values. This is tested
        against Gazebo's cinder-block's spatial inertia specification.

        See: http://models.gazebosim.org/cinder_block/model.sdf
        """
        expected_output = \
            "Computing spatial inertia of solid cuboid with properties:\n" \
            "  - x axis length = 0.37\n" \
            "  - y axis length = 0.17\n" \
            "  - z axis length = 0.2\n" \
            "  - mass = 5.0\n" \
            "Results\n" \
            "  - ixx = 0.0287083333333\n" \
            "  - iyy = 0.0737083333333\n" \
            "  - izz = 0.0690833333333\n"

        self.run_and_expect(
            ['0.37', '0.17', '0.2', '5'],
            0,
            [expected_output])


if __name__ == '__main__':
    unittest.main()
