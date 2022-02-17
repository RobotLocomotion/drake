import functools
from logging import (
    CRITICAL,
    DEBUG,
    ERROR,
    INFO,
    NOTSET,
    WARNING,
)
import re
import subprocess
import sys
import unittest

from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)

# As a Drake convention, we use log level 5 for "TRACE" to match spdlog's
# trace() function.  (Level 0 is "NOTSET" and Level 10 is "DEBUG".)
DRAKE_TRACE = 5


class TestTextLoggingExample(unittest.TestCase,
                             metaclass=ValueParameterizedTest):
    """Proves that Python logging is in effect, by checking all combinations of
    log level settings for both C++ and Python.

    https://drake.mit.edu/doxygen_cxx/text__logging_8h.html
    https://docs.python.org/3/library/logging.html#levels
    """

    @staticmethod
    def _python_level_name(level):
        """Given a Python logging integer level code, returns the corresponding
        Python string name.
        """
        return dict((
            (NOTSET, "NOTSET"),
            (DRAKE_TRACE, "Level 5"),
            (DEBUG, "DEBUG"),
            (INFO, "INFO"),
            (WARNING, "WARNING"),
            (ERROR, "ERROR"),
            (CRITICAL, "CRITICAL"),
        ))[level]

    @staticmethod
    def _spdlog_level_name(level):
        """Given a Python logging integer level code, returns the corresponding
        spdlog::level enum name (that can be passed to pydrake' set_log_level).
        """
        return dict((
            (NOTSET, "unchanged"),
            (DRAKE_TRACE, "trace"),
            (DEBUG, "debug"),
            (INFO, "info"),
            (WARNING, "warn"),
            (ERROR, "err"),
            (CRITICAL, "critical"),
        ))[level]

    def _all_permutations():
        """Enumerates all permutations of logging configurations to test."""
        all_levels = [
            NOTSET,
            DRAKE_TRACE,
            DEBUG,
            INFO,
            WARNING,
            ERROR,
            CRITICAL,
        ]
        for use_nice_format in [False, True]:
            for python_level in all_levels:
                if use_nice_format and python_level == NOTSET:
                    # There is no such thing as "nice format" when we're not
                    # configuring the Python logging in the first place.
                    continue
                for spdlog_level in all_levels:
                    yield dict(
                        use_nice_format=use_nice_format,
                        python_level=python_level,
                        spdlog_level=spdlog_level)

    @run_with_multiple_values(_all_permutations())
    def test_example(self, *, use_nice_format, python_level, spdlog_level):
        """Runs the text_logging_example and checks its output."""
        # Invoke the test program.
        spdlog_level_name = self._spdlog_level_name(spdlog_level)
        try:
            output = subprocess.check_output(
                ["bindings/pydrake/common/text_logging_example",
                 f"--python_level={python_level}",
                 f"--spdlog_level_name={spdlog_level_name}",
                 f"--use_nice_format={int(use_nice_format)}"],
                stderr=subprocess.STDOUT, encoding="utf8")
        except subprocess.CalledProcessError as e:
            print(e.output, file=sys.stderr, flush=True)
            raise

        # Parse all of its output lines, e.g., the nice format:
        #  [2022-02-11 14:49:17,952] [drake] [error] Test Error message
        # or the basic format:
        #  ERROR:drake:Test Error message
        found = []
        if use_nice_format:
            pattern = r"\[.{23}\] \[drake\] \[(.*?)\] (.*)"
        else:
            pattern = r"(.*?):drake:(.*)"
        matcher = re.compile(pattern)
        for line in output.splitlines():
            match = matcher.match(line)
            self.assertTrue(match, f"{line!r} does not match {pattern}")
            found.append(match.groups())

        # Confirm the message bodies vs the reported level.
        expected_messages = {
            "Level 5": "Test Trace message",
            "DEBUG": "Test Debug message",
            "INFO": "Test Info message",
            "WARNING": "Test Warn message",
            "ERROR": "Test Error message",
            "CRITICAL": "Test Critical message",
        }
        for spdlog_level_token, message in found:
            self.assertEqual(message, expected_messages[spdlog_level_token])

        # Confirm that the correct levels came out.
        found_levels = set([k for k, _ in found])
        expected_level = max(python_level or INFO, spdlog_level or INFO)
        expected_levels = set()
        if expected_level <= DRAKE_TRACE:
            expected_levels.add("Level 5")
        if expected_level <= DEBUG:
            expected_levels.add("DEBUG")
        if expected_level <= INFO:
            expected_levels.add("INFO")
        if expected_level <= WARNING:
            expected_levels.add("WARNING")
        if expected_level <= ERROR:
            expected_levels.add("ERROR")
        if expected_level <= CRITICAL:
            expected_levels.add("CRITICAL")
        self.assertSetEqual(found_levels, expected_levels)
