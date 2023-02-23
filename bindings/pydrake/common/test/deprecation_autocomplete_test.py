"""
Tests autocompletion for deprecation API.

See `deprecation_utility_test.py` for a unittest on higher-level API. See
`deprecation_test.py` for a general unittest on low-level API. This test is
separate because the `rlcompleter` in Python 3.10 mucks around with the
module's reference count, causing the `tearDown` test in `deprecation_test.py`
to fail if this test is part of that suite.
"""

import rlcompleter
import sys
import unittest


def get_completion_suffixes(namespace, prefix, max_count=1000):
    # Gets all completions for a given namespace and prefix, stripping the
    # prefix from the results. Also strip a trailing ')' if present, as
    # different versions of Python are not consistent whether or not this is
    # present.
    completer = rlcompleter.Completer(namespace)
    suffixes = []
    for i in range(max_count):
        candidate = completer.complete(prefix, i)
        if candidate is None:
            break
        assert candidate.startswith(prefix), (prefix, candidate)
        suffix = candidate[len(prefix):]
        if suffix.endswith("()"):
            suffix = suffix[:-1]
        suffixes.append(suffix)
    else:
        raise RuntimeError("Exceeded max count!")
    return suffixes


class TestDeprecation(unittest.TestCase):
    """Tests module shim functionality. """
    def test_module_autocomplete(self):
        # Ensure that we can autocomplete with our example module.
        # Without `__dir__` being implemented, it'll only return `install` as a
        # non-private autocomplete candidate.
        import deprecation_example
        suffixes = get_completion_suffixes(
            locals(), prefix="deprecation_example.")
        suffixes_expected = [
            # Injection from `Completer.attr_matches`, via `get_class_members`.
            "__class__(",
            "__delattr__(",
            "__dict__",
            "__dir__(",
            "__doc__",
            "__format__(",
            "__getattr__(",
            "__getattribute__(",
            "__hash__(",
            "__init__(",
            "__module__",
            "__new__(",
            "__reduce__(",
            "__reduce_ex__(",
            "__repr__(",
            "__setattr__(",
            "__sizeof__(",
            "__str__(",
            "__subclasshook__(",
            "__weakref__",
            "_install(",
            # Intended completions via `__all__`.
            "sub_module",
            "value",
        ]
        # Python 3.11 adds a default implementation of __getstate__(), see:
        # https://docs.python.org/3/library/pickle.html#object.__getstate__
        # It does not exist in python <=3.10.
        if sys.version_info[0:2] >= (3, 11):
            suffixes_expected.append("__getstate__(")
        suffixes_expected += [
            "__ge__(",
            "__eq__(",
            "__le__(",
            "__lt__(",
            "__gt__(",
            "__ne__(",
        ]
        if hasattr(deprecation_example, "__init_subclass__"):
            suffixes_expected.append("__init_subclass__(")
        under = get_completion_suffixes(
            locals(), prefix="deprecation_example._")
        suffixes += ["_" + s for s in under]
        dunder = get_completion_suffixes(
            locals(), prefix="deprecation_example.__")
        suffixes += ["__" + s for s in dunder]
        self.assertSetEqual(set(suffixes), set(suffixes_expected))
