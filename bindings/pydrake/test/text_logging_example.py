import sys

from pydrake.test.text_logging_test import do_log_test
from pydrake.common import set_log_level


def main():
    spdlog_level = sys.argv[1]
    set_log_level(spdlog_level)
    do_log_test()


if __name__ == "__main__":
    main()
