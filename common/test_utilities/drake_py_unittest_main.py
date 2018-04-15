"""This is Drake's default main() for unittest-based tests.  It is intended for
use by the drake_py_unittest macro defined in //tools/skylark:drake_py.bzl and
should NOT be called directly by anything else.
"""

import argparse
import imp
import os
import re
import sys
import trace
import unittest

if __name__ == '__main__':
    # Obtain the full path for this test case; it looks a bit like this:
    # .../execroot/.../foo_test.runfiles/.../drake_py_unittest_main.py
    main_py = sys.argv[0]

    # Parse the test case name out of the runfiles directory name.
    match = re.search("/([^/]*_test).runfiles/", main_py)
    if not match:
        print("error: no test name match in {}".format(main_py))
        sys.exit(1)
    test_name, = match.groups()
    test_filename = test_name + ".py"

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
            if any([line.startswith("if __name__ =="),
                    line.strip().startswith("unittest.main")]):
                print("error: " + test_filename + " appears to have a main " +
                      "function (checks 'if __name__ == ') or call the main " +
                      "function of unittest ('unittest.main') but also uses " +
                      "drake_py_unittest; when using drake_py_unittest, " +
                      "the boilerplate main function should not be used; " +
                      "if this test is not based on unittest, declare it " +
                      "as drake_py_test instead of drake_py_unittest and " +
                      "keep the main function intact")
                sys.exit(1)
    if os.access(found_filename, os.X_OK):
        print("error: " + test_filename + " uses drake_py_unittest but is " +
              "marked executable in the filesystem; fix this via chmod a-x " +
              test_filename)
        sys.exit(1)
    module = imp.load_source(test_name, found_filename)

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

    if args.trace != "none":
        if args.trace == "user":
            ignoredirs = sys.path
        else:
            ignoredirs = []
        tracer = trace.Trace(trace=1, count=0, ignoredirs=ignoredirs)
        tracer.run('run()')
    else:
        run()
