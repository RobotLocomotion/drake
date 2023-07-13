import logging
import time
import unittest

from pydrake.common import configure_logging
from pydrake.common.test.text_logging_test_helpers import (
    Worker,
    do_log_test,
)

configure_logging()


class TestTextLoggingGil(unittest.TestCase):
    def test_multithreaded_logging(self):
        """Exercises Drake's Python logging sink in the case where the log
        messages are coming from multiple kinds of sources concurrently:
        from pure Python, from the Python main thread calling into pybind11'd
        C++ code, and from a C++ worker thread.

        Prior to the bugfix merged along with this test case, the code would
        segfault (when using the prior implementation), or deadlock and timeout
        (if the implementation still used spdlog mutexes). A successful test
        does not make any unittest assertions; the fact that is runs without
        faults is a sufficient proof of safety.
        """
        logging.info("Starting Worker")
        worker = Worker()
        worker.Start()

        for i in range(100):
            logging.info(f"Looping {i}")
            do_log_test()
            time.sleep(0.01)

        logging.info("Shutting down")
        worker.Stop()
