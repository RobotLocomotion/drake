"""This is Drake's default main() for unittest-based tests.  It is intended for
use by the drake_py_unittest macro defined in //tools/skylark:drake_py.bzl and
should NOT be called directly by anything else.
"""

import argparse
import imp
import io
import os
import re
import sys
import trace
import unittest
import warnings

try:
    from pydrake.common.deprecation import DrakeDeprecationWarning
    has_pydrake = True
except ImportError:
    has_pydrake = False


if __name__ == '__main__':
    # Obtain the full path for this test case; it looks a bit like this:
    # .../execroot/.../foo_test.runfiles/.../drake_py_unittest_main.py
    main_py = sys.argv[0]

    # Parse the test case name out of the runfiles directory name.
    match = re.search("^(.*bin/(.*?/)?(py/)?([^/]*_test).runfiles/)", main_py)
    if not match:
        print("error: no test name match in {}".format(main_py))
        sys.exit(1)
    runfiles, test_package, _, test_name, = match.groups()
    test_basename = test_name + ".py"

    # Check the test's source code for a (misleading) __main__.
    runfiles_test_filename = (
        runfiles + "drake/" + test_package + "test/" + test_basename)
    if not os.path.exists(runfiles_test_filename):
        raise RuntimeError("Could not find {} at {}".format(
            test_basename, runfiles_test_filename))
    realpath_test_filename = os.path.realpath(runfiles_test_filename)
    with io.open(realpath_test_filename, "r", encoding="utf8") as infile:
        for line in infile.readlines():
            if any([line.startswith("if __name__ =="),
                    line.strip().startswith("unittest.main")]):
                print("error: " + test_basename + " appears to have a main " +
                      "function (checks 'if __name__ == ') or call the main " +
                      "function of unittest ('unittest.main') but also uses " +
                      "drake_py_unittest; when using drake_py_unittest, " +
                      "the boilerplate main function should not be used; " +
                      "if this test is not based on unittest, declare it " +
                      "as drake_py_test instead of drake_py_unittest and " +
                      "keep the main function intact")
                sys.exit(1)
    if os.access(realpath_test_filename, os.X_OK):
        print("error: " + test_basename + " uses drake_py_unittest but is " +
              "marked executable in the filesystem; fix this via chmod a-x " +
              test_basename)
        sys.exit(1)
    module = imp.load_source(test_name, runfiles_test_filename)

    # Figure out which arguments are for unittest and which are for the module
    # under test.
    unittest_argv = sys.argv[:1]
    known_unittest_args = [
        "-h", "--help",
        "-v", "--verbose",
        "-q", "--quiet",
        "-f", "--failfast",
        "-c", "--catch",
        "-b", "--buffer",
    ]
    test_class_guesses = [
        x for x in dir(module)
        if x.startswith("Test")
    ]
    index = 1
    while index < len(sys.argv):
        arg = sys.argv[index]
        if arg in known_unittest_args or any([
                arg.startswith(clazz) for clazz in test_class_guesses]):
            unittest_argv.append(arg)
            sys.argv.pop(index)
            continue
        index += 1

    # Custom flags.
    parser = argparse.ArgumentParser(description="Drake-specific arguments")
    parser.add_argument(
        "--trace", type=str, choices=["none", "user", "sys"], default="none",
        help="Enable source tracing. `none` implies no tracing, `user` " +
             "implies tracing user code, and `sys` implies tracing all " +
             "code. Default is `none`.")
    parser.add_argument(
        "--nostdout_to_stderr", action="store_true",
        help="Do not pipe stdout to stderr. When running from the Bazel " +
             "client (non-batch), output may be mixed, so piping makes " +
             "the output more readable.")
    parser.add_argument(
        "--deprecation_action", type=str, default="once",
        help="Action for any deprecation warnings. See " +
             "`warnings.simplefilter()`.")
    parser.add_argument(
        "--drake_deprecation_action", type=str, default="error",
        help="Action for Drake deprecation warnings. Applied after " +
             "--deprecation_action.")
    args, remaining = parser.parse_known_args()
    sys.argv = sys.argv[:1] + remaining

    def run():
        # Ensure we print out help.
        if "-h" in unittest_argv or "--help" in unittest_argv:
            parser.print_help()
            print("\n`unittest` specific arguments")

        # Delegate the rest to unittest.
        unittest.main(module=test_name, argv=unittest_argv)

    if not args.nostdout_to_stderr:
        sys.stdout.flush()
        sys.stdout = sys.stderr

    # Ensure deprecation warnings are always shown at least once.
    warnings.simplefilter(args.deprecation_action, DeprecationWarning)
    # Handle Drake-specific deprecations.
    if has_pydrake:
        warnings.simplefilter(
            args.drake_deprecation_action, DrakeDeprecationWarning)

    if args.trace != "none":
        if args.trace == "user":
            # Add `sys.prefix` here, just in case we're debugging with a
            # virtualenv.
            ignoredirs = ["/usr", sys.prefix]
        else:
            ignoredirs = []
        tracer = trace.Trace(trace=1, count=0, ignoredirs=ignoredirs)
        tracer.run('run()')
    else:
        run()
