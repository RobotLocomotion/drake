from pydrake.common import (
  MaybeRedirectPythonLogging,
  MaybeUndoRedirectPythonLogging,
)
from pydrake.common.test.text_logging_test import do_log_info


def main():
    for i in range(3):
        do_log_info(f"Redirected {i}")

        # Takes effect.
        assert MaybeUndoRedirectPythonLogging()
        # Does not take effect twice in a row.
        assert not MaybeUndoRedirectPythonLogging()

        do_log_info(f"Not redirected {i}")

        # Takes effect.
        assert MaybeRedirectPythonLogging()
        # Does not take effect twice in a row.
        assert not MaybeRedirectPythonLogging()


if __name__ == "__main__":
    main()
