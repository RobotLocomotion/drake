import re
import subprocess
import unittest


class TestTextLogging(unittest.TestCase):
    def expected_message(self, spdlog_level):
        # Expected message format:
        # [<date> <time>] [console] [<level>] <message>

        # See bindings/pydrake/test/text_logging_test_py.cc
        expected_messages = {"debug": "Test Debug message",
                             "info": "Test Info message",
                             "warn": "Test Warn message",
                             "error": "Test Error message",
                             "critical": "Test Critical message"}
        level_strings = {"debug": "debug",
                         "info": "info",
                         "warn": "warning",
                         "error": "error",
                         "critical": "critical"}

        message = expected_messages[spdlog_level]
        level = level_strings[spdlog_level]
        return fr"\[[0-9,\-,\s,:,\.]*\] \[console\] \[{level}\] {message}"

    def do_test(self, spdlog_level, expected_spdlog_levels):
        output = subprocess.check_output(
            ["bindings/pydrake/text_logging_example", spdlog_level],
            stderr=subprocess.STDOUT).decode("utf8")
        expected_output = ""
        for level in expected_spdlog_levels:
            expected_output += self.expected_message(level) + "\n"
        if not expected_output:
            self.assertEqual(output, expected_output)
        else:
            self.assertRegex(output, expected_output)

    def test_debug_logging(self):
        self.do_test("debug",
                     ["debug", "info", "warn", "error", "critical"])

    def test_info_logging(self):
        self.do_test("info", ["info", "warn", "error", "critical"])

    def test_warning_logging(self):
        self.do_test("warn", ["warn", "error", "critical"])

    def test_error_logging(self):
        self.do_test("err", ["error", "critical"])

    def test_critical_logging(self):
        self.do_test("critical", ["critical"])

    def test_no_logging(self):
        self.do_test("off", [])
