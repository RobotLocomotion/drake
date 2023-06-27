import logging

from pydrake.common import MaybeUndoRedirectPythonLogging
from pydrake.common.test.text_logging_test import do_log_info


def main():
    do_log_info(f"Redirected")

    # Takes effect.
    assert MaybeUndoRedirectPythonLogging()
    # Does not take effect twice in a row.
    assert not MaybeUndoRedirectPythonLogging()

    do_log_info(f"Not redirected")

    drake_logger = logging.getLogger("drake")
    assert not drake_logger._tied_to_spdlog

    # TODO: Verify monkey patches are reverted / no longer have an effect?


if __name__ == "__main__":
    main()
