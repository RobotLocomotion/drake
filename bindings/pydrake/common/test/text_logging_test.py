import logging
import unittest

import pydrake.common as mut


class TestHandler(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.messages = []

    def emit(self, record):
        # Save formatted message
        self.messages.append(self.format(record))


class TestLogging(unittest.TestCase):
    def setUp(self):
        self.assertTrue(mut._module_py._HAVE_SPDLOG)

        # Get the current logger and handler
        self.logger = logging.getLogger("drakelog")
        self.default_handler = self.logger.handlers[0]
        # Remove the default handler
        self.logger.removeHandler(self.default_handler)

        # Create a new test handler to capture output
        self.test_handler = TestHandler()
        # Set the format the same as the default handler
        formatter = logging.Formatter(self.default_handler.formatter._fmt)
        self.test_handler.setFormatter(formatter)
        # Add test handler to the logger
        self.logger.addHandler(self.test_handler)

    def tearDown(self):
        # Remove test handler from the logger
        self.logger.removeHandler(self.test_handler)
        # Restore the default handler
        self.logger.addHandler(self.default_handler)

    def expected_message(self, type):
        # Expected message format:
        # [<date> <time>] [console] [<type>] <message>
        return "".join([
                  r"^\[[0-9,\-,\s,:,\.]*\] \[console\] \[", type, r"\] ",
                  type, " message", "\n$"])

    def do_test(self, log_level, expected_messages):
        mut.set_log_level(log_level)

        mut.log_debug("debug message")
        mut.log_info("info message")
        mut.log_warn("warning message")
        mut.log_error("error message")
        mut.log_critical("critical message")

        messages = self.test_handler.messages

        self.assertEqual(len(messages), len(expected_messages))

        for message, type in zip(messages, expected_messages):
            self.assertRegex(message,
                             self.expected_message(type))

    def test_debug_logging(self):
        self.do_test("debug",
                     ["debug", "info", "warning", "error", "critical"])

    def test_info_logging(self):
        self.do_test("info", ["info", "warning", "error", "critical"])

    def test_warning_logging(self):
        self.do_test("warn", ["warning", "error", "critical"])

    def test_error_logging(self):
        self.do_test("err", ["error", "critical"])

    def test_critical_logging(self):
        self.do_test("critical", ["critical"])

    def test_no_logging(self):
        self.do_test("off", [])
