import os
import re
import subprocess
import sys
import unittest

import install_test_helper
import otool


def has_prefix(path, prefixes):
    for p in prefixes:
        if path.startswith(p):
            return True
    return False


class RpathTest(unittest.TestCase):

    def test_rpaths(self):
        """Confirms that libdrake.so does not link to any non-system libraries
        that are not correctly RPATH'd.
        """

        # The shared library to be tested.
        libdrake = os.path.join(
            install_test_helper.get_install_dir(),
            "lib/libdrake.so"
            )
        self.assertTrue(os.path.exists(libdrake))

        # Inspect the linked libraries.
        if sys.platform == "darwin":
            allowed_prefixes = [
                "@rpath/"
                "@loader_path/",
                "/usr/lib/",
                "/usr/local/opt/",
                "/opt/homebrew/opt/",
                "/System/Library/Frameworks/",
            ]
            for lib in otool.linked_libraries(libdrake):
                self.assertTrue(has_prefix(lib.path, allowed_prefixes),
                                msg=f"{lib.path} has a disallowed prefix")
        else:
            allowed_prefixes = [
                "/lib/",
                "/lib64/",
                "/usr/lib/",
                "/usr/lib64/",
                os.path.dirname(libdrake),
            ]
            output = subprocess.check_output(
                ['ldd', libdrake], encoding="utf8")

            for line in output.splitlines():
                m = re.match('.* => (.*) [(]0x[0-9a-f]+[)]$', line.strip())
                if m is not None:
                    (resolved,) = m.groups()
                    self.assertNotEqual(resolved, "not found",
                                        msg=line.strip())
                    self.assertTrue(has_prefix(resolved, allowed_prefixes),
                                    msg=f"{resolved} has a disallowed prefix")


if __name__ == '__main__':
    unittest.main()
