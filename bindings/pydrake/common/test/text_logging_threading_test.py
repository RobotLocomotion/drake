import logging
import unittest

from pydrake.common import use_native_cpp_logging
from pydrake.common.test.text_logging_test_helpers import do_log_test_async

# If we comment out this line, then the test will fail (by hanging indefinitely
# until Bazel's test timeout kills it).
use_native_cpp_logging()


class TestTextLoggingThreading(unittest.TestCase):

    def test_python_threaded_logging(self):
        """Exercises Drake's native C++ logging sink in the case where Python
        code calls a bound C++ function that logs on a different C++ thread.
        """
        do_log_test_async()
