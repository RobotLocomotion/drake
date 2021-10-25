"""This is Drake's default main() for unittest-based tests.  It is intended for
use by the drake_py_unittest macro defined in //tools/skylark:drake_py.bzl and
should NOT be called directly by anything else.
"""

import argparse
from importlib.machinery import SourceFileLoader
import io
import os
import re
import sys
import trace
import unittest
import warnings
import xmlrunner

try:
    from pydrake.common.deprecation import DrakeDeprecationWarning
    has_pydrake = True
except ImportError:
    has_pydrake = False


def main():
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
                print(f"error: {test_basename} appears to have a main "
                      "function (checks 'if __name__ == ') or call the main "
                      "function of unittest ('unittest.main') but also uses "
                      "drake_py_unittest; when using drake_py_unittest, "
                      "the boilerplate main function should not be used; "
                      "if this test is not based on unittest, declare it "
                      "as drake_py_test instead of drake_py_unittest and "
                      "keep the main function intact")
                sys.exit(1)
    if os.access(realpath_test_filename, os.X_OK):
        print(f"error: {test_basename} uses drake_py_unittest but is "
              "marked executable in the filesystem; fix this via "
              f"chmod a-x {test_basename}")
        sys.exit(1)

    # On import, force all drake deprecation warnings to trigger an error.
    if has_pydrake:
        warnings.simplefilter("error", DrakeDeprecationWarning)

    module = SourceFileLoader(test_name, runfiles_test_filename).load_module(
        test_name)

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
        help="Enable source tracing. `none` implies no tracing, `user` "
             "implies tracing user code, and `sys` implies tracing all "
             "code. Default is `none`.")
    parser.add_argument(
        "--nostdout_to_stderr", action="store_true",
        help="Do not reexec to get unbuffered output. When running from the "
             "Bazel client (non-batch), stdout and stderr ordering may not "
             "flush at convenient times, making errors less readable. Having "
             "the output be unbuffered makes it more readable.")
    parser.add_argument(
        "--deprecation_action", type=str, default="once",
        help="Action for any deprecation warnings. See "
             "`warnings.simplefilter()`.")
    parser.add_argument(
        "--drake_deprecation_action", type=str, default="error",
        help="Action for Drake deprecation warnings. Applied after "
             "--deprecation_action.")
    args, remaining = parser.parse_known_args()
    sys.argv = sys.argv[:1] + remaining

    def run():
        # Ensure we print out help.
        if "-h" in unittest_argv or "--help" in unittest_argv:
            parser.print_help()
            print("\n`unittest` specific arguments")

        # Delegate the rest to unittest.
        #
        # Use `warnings=False` to tell unittest to keep its hands off of our
        # warnings settings, exploting a loophole where it checks "if warnings
        # is None" to check if the user passed a kwarg, but "if warning" to
        # actually apply the user's kwarg.
        # N.B. Do not use the runner when `--trace={user,sys}` is enabled.
        if "XML_OUTPUT_FILE" in os.environ and args.trace == "none":
            with open(os.environ["XML_OUTPUT_FILE"], "wb") as output:
                unittest.main(
                    module=test_name, argv=unittest_argv, warnings=False,
                    testRunner=xmlrunner.XMLTestRunner(output=output))
        else:
            unittest.main(module=test_name, argv=unittest_argv, warnings=False)

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
        run = traced(run, ignoredirs=ignoredirs)
    run()


# N.B. `reexecute_if_unbuffered` and `traced` should be kept in exact sync with
# `doc/python_bindings.rst`. It should be easy to copy and paste this in other
# scripts.


def reexecute_if_unbuffered():
    """Ensures that output is immediately flushed (e.g. for segfaults).
    ONLY use this at your entrypoint. Otherwise, you may have code be
    re-executed that will clutter your console."""
    import os
    import shlex
    import sys
    if os.environ.get("PYTHONUNBUFFERED") in (None, ""):
        os.environ["PYTHONUNBUFFERED"] = "1"
        argv = list(sys.argv)
        if argv[0] != sys.executable:
            argv.insert(0, sys.executable)
        cmd = " ".join([shlex.quote(arg) for arg in argv])
        sys.stdout.flush()
        os.execv(argv[0], argv)


def traced(func, ignoredirs=None):
    """Decorates func such that its execution is traced, but filters out any
     Python code outside of the system prefix."""
    import functools
    import sys
    import trace
    if ignoredirs is None:
        ignoredirs = ["/usr", sys.prefix]
    tracer = trace.Trace(trace=1, count=0, ignoredirs=ignoredirs)

    @functools.wraps(func)
    def wrapped(*args, **kwargs):
        return tracer.runfunc(func, *args, **kwargs)

    return wrapped


if __name__ == '__main__':
    # TODO(eric.cousineau): Move this into `main()` to leverage argparse if we
    # can simplify the custom parsing logic (e.g. not have to import source
    # modules).
    if "--nostdout_to_stderr" not in sys.argv:
        reexecute_if_unbuffered()
        # N.B. If execv is called by `reexecute_if_unbuffered`, then `main`
        # will not be called by this current process image; instead, it will be
        # called in the *new* process image.
    main()
