import argparse
import logging
import os

from text_logging_test_helpers import do_log_test, set_log_level
if "_TEST_SPDLOG_LEVEL" in os.environ:
    # Configure logging before we import pydrake so that we can intercept debug
    # messages while handling the redirect.
    set_log_level(os.environ["_TEST_SPDLOG_LEVEL"])

from pydrake.common import configure_logging


def main():
    # Parse our arguments.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--use_nice_format", metavar="1|0", type=int, required=True,
        help="Switch on (or off) nicely formatted Python output.")
    parser.add_argument(
        "--root_level", metavar="INT", type=int, required=True,
        help="Set Python root logger to this level, or use -1 for a no-op.")
    parser.add_argument(
        "--drake_level", metavar="INT", type=int, required=True,
        help="Set Python Drake logger to this level, or use -1 for a no-op.")
    args = parser.parse_args()

    # Configure logging as instructed.
    if args.use_nice_format:
        configure_logging()
    if args.root_level >= 0:
        logging.getLogger().setLevel(args.root_level)
    if args.drake_level >= 0:
        logging.getLogger("drake").setLevel(args.drake_level)

    # Emit a bunch of log messages from C++. Depending on the level filtering,
    # not all of them will be printed.
    do_log_test()


if __name__ == "__main__":
    main()
