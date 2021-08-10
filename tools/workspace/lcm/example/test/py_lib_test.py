import unittest

from package1.package2 import lcmt_foo


class TestFoo(unittest.TestCase):

    def test_foo(self):
        foo = lcmt_foo()
        foo.bar.value = 5
        self.assertIsNotNone(foo.encode())
