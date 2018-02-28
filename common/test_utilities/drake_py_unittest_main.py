"""This is Drake's default main() for unittest-based tests.  It is intended for
use by the drake_py_unittest macro defined in //tools/skylark:drake_py.bzl and
should NOT be called directly by anything else.
"""

import os
import sys
import unittest

if __name__ == '__main__':
    test_filename = sys.argv.pop(1)

    # Find the test_filename and check it for a (misleading) __main__.
    found_filename = None
    for dirpath, dirs, files in os.walk("."):
        if test_filename in files:
            assert not found_filename
            found_filename = os.path.join(dirpath, test_filename)
    if not found_filename:
        raise RuntimeError("No such file found {}!".format(
            test_filename))
    with open(found_filename, "r") as infile:
        for line in infile.readlines():
            if line.startswith("if __name__ =="):
                print("error: " + test_filename + " appears to have a main " +
                      "function (checks 'if __name__ == ') but also uses " +
                      "drake_py_unittest; when using drake_py_unittest, "
                      "the boilerplate main function should not be used; " +
                      "if this test is not based on unittest, declare it " +
                      "as drake_py_test instead of drake_py_unittest and " +
                      "keep the main function intact")
                sys.exit(1)

    # Have unittest find the test_filename and load its tests.
    tests = unittest.TestLoader().discover(".", pattern=test_filename)
    if tests.countTestCases() == 0:
        raise RuntimeError("No tests found in {}!".format(
            test_filename))

    # Run everything
    result = unittest.runner.TextTestRunner().run(tests)
    if result.testsRun == 0:
        raise RuntimeError("Tests found in {} but none run!".format(
            test_filename))
    if not result.wasSuccessful():
        sys.exit(1)
