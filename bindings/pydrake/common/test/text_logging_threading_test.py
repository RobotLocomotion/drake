import logging
import threading
import unittest

from pydrake.common import use_native_cpp_logging
from pydrake.common.test.text_logging_test_helpers import do_log_test

# With this commented out, the test should fail (block).
# use_native_cpp_logging()


class TestTextLoggingThreading(unittest.TestCase):

    @staticmethod
    def _log_from_cpp_a_whole_lot():
        for _ in range(10):
            do_log_test()

    def test_python_threaded_logging(self):
        """Exercises Drake's C++ logging sink in the case where Python code
        is launching multiple threads that all generate C++ log messages.
        """
        # This constant must be kept in sync with our BUILD file.
        num_cpu = 4

        # Repeat the thread start + stop multiple times, to try to catch races.
        target = self._log_from_cpp_a_whole_lot
        num_tries = 100
        for i in range(num_tries):
            logging.info("Running attempt {i+1}/{num_tries} ...")
            threads = []
            for j in range(num_cpu):
                threads.append(threading.Thread(target=target))
            for x in threads:
                x.start()
            for x in threads:
                x.join()
