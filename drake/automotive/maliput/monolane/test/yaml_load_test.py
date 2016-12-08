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
_DRAKE_PATHS_DIR = os.path.dirname(os.path.dirname(os.path.dirname(_THIS_DIR)))

sys.path.append(_DRAKE_PATHS_DIR)

from drake_paths import DRAKE_DRAKE_BUILD_DIR


class TestYamlLoading(unittest.TestCase):
    def test_yaml_files(self):
        yaml_dir = os.path.dirname(_THIS_DIR)
        yaml_files = glob.glob(os.path.join(yaml_dir, '*.yaml'))
        blacklist = []
        test_yaml_files = [f for f in yaml_files
                           if not any([b in f for b in blacklist])]
        assert len(test_yaml_files) > 0
        for yf in test_yaml_files:
            subprocess.check_call([
                os.path.join(DRAKE_DRAKE_BUILD_DIR,
                             "automotive/maliput/monolane/test/yaml_load"),
                "-yaml_file", yf,
            ])


if __name__ == '__main__':
    unittest.main()
