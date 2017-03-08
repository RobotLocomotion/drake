#!/usr/bin/env python

"""Attempts to generate a Wavefront OBJ file from each non-blacklisted
*.yaml file from parent directory, returning a non-zero exit code if any
file fails.
"""

import glob
import subprocess
import os
import unittest
import sys

_THIS_FILE = os.path.abspath(__file__)
_THIS_DIR = os.path.dirname(_THIS_FILE)


class TestYamlObjing(unittest.TestCase):
    YAML_TO_OBJ = "yaml_to_obj"

    def test_yaml_files(self):
        this_dir = os.path.dirname(_THIS_DIR)
        yaml_dir = os.path.join(this_dir, '../monolane')

        self.assertTrue(os.path.exists(self.YAML_TO_OBJ),
                        self.YAML_TO_OBJ + " not found")

        yaml_files = glob.glob(os.path.join(yaml_dir, '*.yaml'))
        # NB:  Blacklist is empty now, but still here in case it is needed
        # again in the future.
        blacklist = []
        test_yaml_files = [f for f in yaml_files
                           if not any([b in f for b in blacklist])]
        self.assertTrue(len(test_yaml_files) > 0)

        for yf in test_yaml_files:
            subprocess.check_call([
                self.YAML_TO_OBJ,
                "-yaml_file", yf,
                "-obj_file", "/dev/null",
            ])


if __name__ == '__main__':
    TestYamlObjing.YAML_TO_OBJ = sys.argv.pop()
    unittest.main()
