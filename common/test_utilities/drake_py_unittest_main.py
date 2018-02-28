"""This is Drake's default main() for unittest-based tests.  It is intended for
use by the drake_py_unittest macro defined in //tools/skylark:drake_py.bzl and
should NOT be called directly by anything else.
"""

import sys
import unittest

# TODO(jwnimmer-tri) This helper should fail-fast when the test code has a
# __main__ routine or a shebang line.

if __name__ == '__main__':
    test_filename = sys.argv.pop(1)
    tests = unittest.TestLoader().discover(".", pattern=test_filename)
    if tests.countTestCases() == 0:
        raise RuntimeError("No tests found in {}!".format(
            test_filename))
    result = unittest.runner.TextTestRunner().run(tests)
    if result.testsRun == 0:
        raise RuntimeError("Tests found in {} but none run!".format(
            test_filename))
    if not result.wasSuccessful():
        sys.exit(1)
