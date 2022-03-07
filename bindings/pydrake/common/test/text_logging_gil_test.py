import logging
import time
import unittest

from pydrake.common import configure_logging
from pydrake.common.test.text_logging_test import (
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
