import argparse
import logging

from pydrake.common import configure_logging, set_log_level
from pydrake.common.test.text_logging_test import do_log_test


def main():
    # Parse our arguments.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--python_level", metavar="INT", type=int, required=True,
        help="Set Python Drake logger to this level, or use 0 as a no-op.")
    parser.add_argument(
        "--spdlog_level_name", metavar="STR", type=str, required=True,
        help="Set C++ Drake to this log level, or use 'unchanged' as a no-op."
             " Refer to set_log_level in drake/common for valid level names.")
    parser.add_argument(
        "--use_nice_format", metavar="1|0", type=int, required=True,
        help="Switch on (or off) nicely formatted Python output.")
    args = parser.parse_args()

    # Configure Python logging (if requested).
    if args.python_level != logging.NOTSET:
        if args.use_nice_format:
            configure_logging()
            logging.getLogger().setLevel(args.python_level)
        else:
            logging.basicConfig(level=args.python_level)
            logging.getLogger("drake").setLevel(logging.NOTSET)
    else:
        # Trying to use the "nice format" makes no sense when we're not even
        # configuring Python logging in the first place.
        assert args.use_nice_format == 0

    # Configure C++ logging (if requested).
    if args.spdlog_level_name != "unchanged":
        set_log_level(args.spdlog_level_name)

    # Emit a bunch of log messages from C++. Depending on the level filtering,
    # not all of them will be printed.
    do_log_test()


if __name__ == "__main__":
    main()
