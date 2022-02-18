import collections
import functools
import re

# TODO(jwnimmer-tri) It's probable that ValueParameterizedTest could be
# re-implemented using a class decorator, instead of a metaclass. That
# would likely be easier for developers to understand and maintain. We
# should explore that next time we revisit any new features here.


# A helper for ValueParameterizedTest that uses a subTest to display the
# exact kwargs upon a failure.
def _run_subtest(self, *, old_method, **kwargs):
    with self.subTest(**kwargs):
        old_method(self, **kwargs)


class ValueParameterizedTest(type):
    """A metaclass to repeat a test case over different argument values.

    This can also be accomplished with a unittest.subTest, but that runs all
    values via a single test case, which is awkward when we have large numbers
    of sub-tests.  In contrast, this metaclass will replicate a prototype
    method into many standalone tests, so that unittest will automatically run
    them one by one, allowing the developer to run them individually.

    A value-paramaterized test case looks like this:

      from module_under_test import dut
      class MyTest(unittest.TestCase, metaclass=ValueParameterizedTest):
          def _foo_values():
              return [22, 33]
          @run_with_multiple_values([dict(foo=foo) for foo in _foo_values()])
          def test_dut(self, *, foo):
              self.assertGreater(dut(foo), 0)

    The effect of the metaclass is as-if the developer wrote this instead:

      from module_under_test import dut
      class MyTest(unittest.TestCase):
          def test_dut_22(self):
              foo = 22
              self.assertGreater(dut(foo), 0)
          def test_dut_33(self):
              foo = 33
              self.assertGreater(dut(foo), 0)
    """
    def __new__(metacls, name, bases, namespace):
        # Find all of the unittest methods.
        test_methods = [
            x for x in namespace.keys()
            if x.startswith("test")
        ]
        assert len(test_methods) > 0

        # Find the unittest methods that used our decorator.
        parameterized_methods = [
            x for x in test_methods
            if hasattr(namespace[x], "_value_parameterized_test_pairs")
        ]
        if not parameterized_methods:
            raise RuntimeError("ValueParameterizedTest was used without any"
                               " @run_with_multiple_values decorators")

        # Multiply the decorated methods into real test cases.
        for method_name in parameterized_methods:
            # Use pop on the decorated method so it isn't run unparameterized.
            old_method = namespace.pop(method_name)

            for suffix, kwargs in old_method._value_parameterized_test_pairs:
                # Conjure the new method name, and sanity check it.
                new_name = method_name + "_" + suffix
                assert new_name.startswith("test_"), new_name
                assert new_name not in namespace, new_name

                # Create a new method with bound kwargs.
                new_method = functools.partialmethod(
                    _run_subtest,
                    old_method=old_method,
                    **kwargs)

                # Keep the same docstring.
                new_method.__doc__ = old_method.__doc__

                # Inject the new method under the new name.
                namespace[new_name] = new_method

        # Continue class construction as usual.
        return type.__new__(metacls, name, bases, namespace)


def _choose_test_suffix(kwargs):
    """Summarizes kwargs as a string, ensuring that the string is a valid
    method name suffix.
    """
    result = "_".join([str(x) for x in kwargs.values()])
    result = re.sub("[^0-9a-zA-Z]+", "_", result)
    result = result.strip("_").lower()
    return result


def _make_test_pairs(values):
    """Returns a list of (test_suffix, kwargs) pairs for the given list of
    kwargs values, by calculating a unique test_suffix summary of the kwargs.
    """
    pairs = [
        [_choose_test_suffix(kwargs), kwargs]
        for kwargs in values
    ]
    # Uniquify any duplicate (or missing) suffix names.
    counter = collections.Counter([x for x, _ in pairs])
    bad_names = set([x for x, count in counter.items() if count > 1 or not x])
    return [
        (x if x not in bad_names else f"{x}_iter{i}", kwargs)
        for i, (x, kwargs) in enumerate(pairs)
    ]


def run_with_multiple_values(values):
    """Decorator for use with the ValueParameterizedTest metaclass to specify
    the set of parameterized values.
    """
    # Convert any iterables into a concrete list and choose method suffixes.
    pairs = _make_test_pairs(values)

    # Attach the list of values to the test function.
    def wrap(check_func):
        check_func._value_parameterized_test_pairs = pairs
        return check_func
    return wrap
