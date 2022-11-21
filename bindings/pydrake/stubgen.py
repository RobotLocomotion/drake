"""Command-line tool to generate Drake's Python Interface files (.pyi)."""

import sys

from mypy import stubgen

# Mypy can time out if importing takes an inordinate length of time. Try to
# avoid this by importing ourselves up front when the import isn't being run
# under a timeout.
import pydrake.all

if __name__ == "__main__":
    sys.exit(stubgen.main())
