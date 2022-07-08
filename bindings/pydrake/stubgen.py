"""Command-line tool to generate Drake's Python Interface files (.pyi)."""

import sys

from mypy import stubgen

if __name__ == "__main__":
    sys.exit(stubgen.main())
