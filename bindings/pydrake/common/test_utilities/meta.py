
class ValueParameterizedTest(type):
    """A metaclass to repeat a test case over different argument values.

    This can also be accomplished with a unittest.subTest, but that runs all
    values via a single test case, which is awkward when we have large numbers
    of sub-tests.  In contrast, this metaclass will replicate a prototype
    method into many standalone tests, so that unittest will automatically run
    them one by one, allowing the developer to run them individually.

    The parameterized test method names must start with "value_test", as in:

      def value_test_dut(self, *, foo):
          self.assertGreater(dut(foo), 0)

    and there must be at least one method that follows that naming convention.
    The metaclass is then instantiated over a values dictionary (iterable of
    pairs) to multiply the test cases. The key (first item in the pair) is the
    suffix to append to the test case name to uniquely name this set of kwargs;
    The value is a dictionary of kwargs to pass to the prototype method. For
    example, values=[(alice, {"foo": 22}), (bob, {"foo": 33})] will create two
    new test methods:

      def test_dut_alice(self):
          self.value_test_dut(foo=22)
      def test_dut_bob(self):
          self.value_test_dut(foo=33)

    Putting it all together, a value-paramaterized test looks like this:

      from some_module import dut
      def make_my_values():
          yield ("alice", dict(foo=22))
          yield ("bob", dict(foo=33))
      class MyTest(unittest.TestCase, metaclass=ValueParameterizedTest,
                   values=make_my_values()):
          def value_test_dut(self, *, foo):
              self.assertGreater(dut(foo), 0)
    """
    def __new__(metacls, name, bases, namespace, *, values):
        values = list(values)
        assert len(values) > 0
        test_methods = [
            x for x in namespace.keys()
            if x.startswith("value_test")
        ]
        assert len(test_methods) > 0, list(namespace.keys())
        for method_name in test_methods:
            old_method = namespace[method_name]
            for suffix, kwargs in values:
                new_name = method_name[6:] + "_" + suffix
                assert new_name.startswith("test_"), new_name
                assert new_name not in namespace, new_name

                def new_method(self):
                    with self.subTest(**kwargs):
                        old_method(self, **kwargs)
                new_method.__doc__ = old_method.__doc__

                namespace[new_name] = new_method
        return type.__new__(metacls, name, bases, namespace)
