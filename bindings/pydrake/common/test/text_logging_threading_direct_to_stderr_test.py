import logging
import unittest

from pydrake.common import use_native_cpp_logging
from pydrake.common.test.text_logging_test_helpers import do_log_test_async

# If we comment out this line, then the test will fail (by hanging indefinitely
# until Bazel's test timeout kills it).
use_native_cpp_logging()


class TestTextLoggingThreadingDirectToStderr(unittest.TestCase):
    """Exercises Drake's native C++ logging sink in the case where Python code
    calls a bound C++ function that logs on a different C++ thread.

    Nominally, the C++ to Python logging redirection would mean taking the GIL
    to write the message; that would usually deadlock because the bound C++
    function call would hold the GIL during the C++ code on the main thread.

    By calling `use_native_cpp_logging` to disable the log redirection, we can
    avoid the deadlock.
    """

    def test_logging(self):
        do_log_test_async()
