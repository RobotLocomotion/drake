# This file is intended for use by the drake_py_test macro defined in
# //tools/skylark:drake_py.bzl and should NOT be used by anything else.

"""A drake_py_test should not `import unittest`.  In most cases, your
BUILD.bazel file should use `drake_py_unittest` to declare such tests, which
provides an appropriate main() routine (and will disable this warning).

In the unlikely event that you actually have a unittest and need to write your
own main, set `allow_import_unittest = True` in the drake_py_test rule.
"""

raise RuntimeError(__doc__)
