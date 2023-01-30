"""Command-line tool to generate Drake's Python Interface files (.pyi)."""

import sys
import warnings

from mypy import stubgen

from pydrake.common.deprecation import DrakeDeprecationWarning

if __name__ == "__main__":
    warnings.simplefilter('ignore', DrakeDeprecationWarning)

    # Mypy can time out if importing takes an inordinate length of time.
    # Try to avoid this by importing ourselves up front when the import
    # isn't being run under a timeout.
    import pydrake.all

    sys.exit(stubgen.main())
