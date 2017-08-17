#!/usr/bin/env python

"""Attempts to load each non-blacklisted *.yaml file from parent directory,
returning a non-zero exit code if any file fails.
"""

import glob
import subprocess
import os
import unittest
import sys

_THIS_FILE = os.path.abspath(__file__)
_THIS_DIR = os.path.dirname(_THIS_FILE)


class TestYamlLoading(unittest.TestCase):
    YAML_LOAD = "yaml_load"

    def test_yaml_files(self):
        yaml_dir = os.path.dirname(_THIS_DIR)

        self.assertTrue(os.path.exists(self.YAML_LOAD),
                        self.YAML_LOAD + " not found")

        yaml_files = glob.glob(os.path.join(yaml_dir, '*.yaml'))
        blacklist = []
        test_yaml_files = [f for f in yaml_files
                           if not any([b in f for b in blacklist])]
        self.assertTrue(len(test_yaml_files) > 0)

        for yf in test_yaml_files:
            subprocess.check_call([self.YAML_LOAD, "-yaml_file", yf])


if __name__ == '__main__':
    TestYamlLoading.YAML_LOAD = sys.argv.pop()
    unittest.main()
