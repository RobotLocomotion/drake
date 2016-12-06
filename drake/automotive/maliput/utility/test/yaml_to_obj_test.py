#!/usr/bin/env python

"""
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


class TestYamlObjing(unittest.TestCase):
    def test_yaml_files(self):
        this_dir = os.path.dirname(_THIS_DIR)
        yaml_files = glob.glob(os.path.join(this_dir, '../monolane/*.yaml'))
        blacklist = []
        test_yaml_files = [f for f in yaml_files
                           if not any([b in f for b in blacklist])]
        assert len(test_yaml_files) > 0
        for yf in test_yaml_files:
            subprocess.check_call([
                os.path.join(DRAKE_DRAKE_BUILD_DIR,
                             "automotive/maliput/utility/yaml_to_obj"),
                "-yaml_file", yf,
                "-obj_file", "/dev/null"
            ])


if __name__ == '__main__':
    unittest.main()
