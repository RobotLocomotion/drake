import logging
import unittest

from pydrake.common.test.text_logging_test_helpers import (
    do_log_test_async_with_gil_release,
)


class TestTextLoggingThreadingWithGilRelease(unittest.TestCase):
    """Exercises Drake's native C++ logging sink in the case where Python code
    calls a bound C++ function that logs on a different C++ thread.

    The C++ to Python logging redirection takes the GIL to write the message;
    that would usually deadlock because the bound C++ function call would hold
    the GIL during the C++ code on the main thread.

    By binding the C++ function to release the GIL before running the C++ code,
    we can avoid the deadlock.
    """

    def test_logging(self):
        do_log_test_async_with_gil_release()
