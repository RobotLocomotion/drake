import copy
import os
from pathlib import Path
import subprocess
import unittest


class TestGurobiSolverLicenseRetention(unittest.TestCase):
    def _subprocess_license_use_count(self, license_file_content):
        """Sets GRB_LICENSE_FILE to a temp file with the given content, runs
        our test helper, and then returns the license pointer use_count.
        """
        # Create a dummy license file. Note that the license filename is magic.
        # The License code in gurobi_solver.cc treats this filename specially.
        tmpdir = Path(os.environ["TEST_TMPDIR"])
        license_file = tmpdir / "DRAKE_UNIT_TEST_NO_LICENSE.lic"
        with open(license_file, "w", encoding="utf-8") as f:
            f.write(license_file_content)

        # Override the built-in license file.
        env = copy.copy(os.environ)
        env["GRB_LICENSE_FILE"] = str(license_file)

        # Run the helper and return the pointer use_count.
        output = subprocess.check_output(
            ["solvers/gurobi_solver_license_retention_test_helper"]
        )
        return int(output)

    def test_local_license(self):
        """When the file named by GRB_LICENSE_FILE contains 'HOSTID', the
        license object is held in two places: the test helper main(), and
        a global variable within GurobiSolver::AcquireLicense.
        """
        content = "HOSTID=foobar\n"
        self.assertEqual(self._subprocess_license_use_count(content), 2)

    def test_nonlocal_license(self):
        """When the file named by GRB_LICENSE_FILE doesn't contain 'HOSTID',
        the license object is only held by main(), not any global variable.
        """
        content = "TOKENSERVER=foobar.invalid.\n"
        self.assertEqual(self._subprocess_license_use_count(content), 1)
