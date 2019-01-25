import os
import subprocess
import sys
import unittest

import install_test_helper


class SnoptVisibilityTest(unittest.TestCase):

    def test_visibility(self):
        """Confirm that SNOPT's symbols are not exported in the installed
        libdrake.so.  See comments in //tools/workspace/snopt/... for details.
        """

        # The shared library to be tested.
        libdrake = os.path.join(
            install_test_helper.get_install_dir(),
            "lib/libdrake.so"
            )
        self.assertTrue(os.path.exists(libdrake))

        # Dump the symbol names to an output string.
        if sys.platform == "linux2":
            command = ["readelf", "--wide", "--symbols"]
            undefined_marker = " UND "
        else:
            command = ["nm", "-a"]
            undefined_marker = " U "
        output = subprocess.check_output(command + [libdrake]).decode("utf8")
        for line in output.splitlines():
            if undefined_marker in line:
                # Ignore undefined references (like snprintf).
                continue
            # Most of snopt's symbols are 'snfoo' or 'f_snfoo' or '_snfoo'.  If
            # we don't find any of those, it seems unlikely that we would find
            # any others like the s08foo ones, etc.
            self.assertNotIn(" sn", line)
            self.assertNotIn(" f_sn", line)
            self.assertNotIn(" _sn", line)


if __name__ == '__main__':
    unittest.main()
