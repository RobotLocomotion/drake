import os
import re
import subprocess
import unittest

import install_test_helper


class TestBinaryInstall(unittest.TestCase):
    def assertNotBazelGeneratedScript(self, bin_file):
        with open(bin_file) as f:
            text = f.read()
        self.assertNotIn("GetWindowsPathWithUNCPrefix", text, bin_file)

    def test_python_binaries(self):
        """Ensure that we do not install in Bazel-generated Python files."""
        install_dir = install_test_helper.get_install_dir()
        bin_list = subprocess.check_output([
            "find", install_dir, "-type", "f", "-executable"
            ]).decode("utf8").split()
        py_file_checks = subprocess.check_output([
            "file"] + bin_list).decode("utf8").splitlines()
        py_bin_match = re.compile(r"(.*):.*Python.*ASCII.*")
        num_matches = 0
        for line in sorted(py_file_checks):
            m = py_bin_match.match(line)
            if m is not None:
                num_matches += 1
                self.assertNotBazelGeneratedScript(m.group(1))
        # Sanity check.
        self.assertNotEqual(num_matches, 0)


if __name__ == '__main__':
    unittest.main()
