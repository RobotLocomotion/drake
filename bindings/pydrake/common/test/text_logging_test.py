import functools
from logging import (
    CRITICAL,
    DEBUG,
    ERROR,
    INFO,
    NOTSET,
    WARNING,
)
import os
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

    def _all_permutations():
        """Enumerates all permutations of logging configurations to test."""
        all_levels = [
            -1,  # The test program uses "-1" to denote leaving the level at
                 # its default value, i.e., not calling setLevel().
            NOTSET,
            DRAKE_TRACE,
            DEBUG,
            INFO,
            WARNING,
            ERROR,
            CRITICAL,
        ]
        for use_nice_format in [False, True]:
            for use_native_cpp_logging in [False, True]:
                for root_level in all_levels:
                    for drake_level in all_levels:
                        yield dict(
                            use_nice_format=use_nice_format,
                            use_native_cpp_logging=use_native_cpp_logging,
                            root_level=root_level,
                            drake_level=drake_level)

    @run_with_multiple_values(_all_permutations())
    def test_example(self, *, use_nice_format, use_native_cpp_logging,
                     root_level, drake_level):
        """Runs the text_logging_example and checks its output."""
        # Invoke the test program.
        try:
            output = subprocess.check_output(
                ["bindings/pydrake/common/text_logging_example",
                 f"--use_nice_format={int(use_nice_format)}",
                 f"--use_native_cpp_logging={int(use_native_cpp_logging)}",
                 f"--root_level={root_level}",
                 f"--drake_level={drake_level}"],
                stderr=subprocess.STDOUT, encoding="utf8")
        except subprocess.CalledProcessError as e:
            print(e.output, file=sys.stderr, flush=True)
            raise

        # Parse all of its output lines, e.g., the nice format:
        #  [2022-02-11 14:49:17,952] [drake] [ERROR] Test Error message
        # or the basic format:
        #  ERROR:drake:Test Error message:
        # or the C++ native format:
        #  [2023-07-19 16:29:50.914] [console] [error] Test Error message
        found = []
        if use_native_cpp_logging:
            pattern = r"\[.{23}\] \[console\] \[([a-z]*?)\] (.*)"
        elif use_nice_format:
            pattern = r"\[.{23}\] \[drake\] \[([LevelA-Z0-9]*?)\] (.*)"
        else:
            pattern = r"(.*?):drake:(.*)"
        matcher = re.compile(pattern)
        for line in output.splitlines():
            match = matcher.match(line)
            self.assertTrue(match, f"{line!r} does not match {pattern}")
            level, message = match.groups()
            if use_native_cpp_logging:
                self.assertTrue(level.islower(), msg=level)
                level = level.upper()
            found.append((level, message))

        # Confirm the message bodies vs the reported level.
        trace_name = "TRACE" if use_nice_format else f"Level {DRAKE_TRACE}"
        expected_messages = {
            trace_name: "Test Trace message",
            "DEBUG": "Test Debug message",
            "INFO": "Test Info message",
            "WARNING": "Test Warn message",
            "ERROR": "Test Error message",
            "CRITICAL": "Test Critical message",
        }
        for spdlog_level_token, message in found:
            self.assertEqual(message, expected_messages[spdlog_level_token])

        # Compute what effective level was emitted.
        observed_spdlog_tokens = set()
        observed_levels = set()
        token_to_level = {
            trace_name: DRAKE_TRACE,
            "DEBUG": DEBUG,
            "INFO": INFO,
            "WARNING": WARNING,
            "ERROR": ERROR,
            "CRITICAL": CRITICAL,
        }
        for spdlog_level_token, _ in found:
            observed_spdlog_tokens.add(spdlog_level_token)
            level = token_to_level[spdlog_level_token]
            observed_levels.add(level)
        effective_level = min(observed_levels)

        # Confirm that all messages above that level were output, as well as
        # the converse (that messages below that level were not).
        for one_level_token in expected_messages:
            level = token_to_level[one_level_token]
            if level >= effective_level:
                self.assertIn(one_level_token, observed_spdlog_tokens)
            else:
                self.assertNotIn(one_level_token, observed_spdlog_tokens)

        # Compute what message level we should expect to have seen.
        if root_level < 0:
            # We did not directly set the root level; what was its default?
            if use_nice_format:
                # The nice format defaults to "INFO".
                root_level = INFO
            else:
                # Otherwise, Python defaults to "WARNING"
                root_level = WARNING
        if drake_level < 0:
            # We did not directly set the Drake level; what was its default?
            if use_nice_format:
                # The nice format defaults to "NOTSET".
                drake_level = NOTSET
            else:
                # The native format default to "INFO"
                drake_level = INFO
        assert drake_level >= 0
        assert root_level >= 0
        expected_level = drake_level or root_level or DRAKE_TRACE
        if use_native_cpp_logging:
            expected_level = INFO

        # Confirm that the correct levels came out.
        self.assertEqual(effective_level, expected_level)

    def test_disabled_via_env(self):
        """When the magic environment variable is set, C++ logging is not
        redirected to Python; it continues to write to stderr directly, using
        its own independent formatter and log level threshold.

        (1) We set the env var so that Python logging and C++ logging are
        independent.
        (2) We do not specifically change anything about C++ logging level.
        (3) We set the Python logging level to "no logging at all".
        (4) The do_log_test function invokes a bunch of C++ log statements.
        The test predicate is that the C++ log statements made it out to
        stderr. If step (1) didn't happen, then step (3) would mean that no
        messages would ever appear, so the test predicate would fail (the
        output would be empty).
        """
        # Disable the code that injects the pylogging_sink.
        env = dict(os.environ)
        env["DRAKE_PYTHON_LOGGING"] = "0"
        # Configure the Python logging to not print anything.
        python_level = CRITICAL + 1
        try:
            output = subprocess.check_output(
                ["bindings/pydrake/common/text_logging_example",
                    "--use_nice_format=1",
                    "--use_native_cpp_logging=0",
                    f"--root_level={python_level}",
                    f"--drake_level={python_level}"],
                stderr=subprocess.STDOUT,
                encoding="utf8",
                env=env)
        except subprocess.CalledProcessError as e:
            print(e.output, file=sys.stderr, flush=True)
            raise
        # The C++ logger should still have printed (INFO and higher)
        self.assertIn("Test Info message", output)
