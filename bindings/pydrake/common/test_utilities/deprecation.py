"""
Utilities for unit testing deprecation.
"""

import contextlib
import warnings

from pydrake.common.deprecation import DrakeDeprecationWarning


def _check_expected(caught, expected_count):
    if expected_count is None:
        return
    if len(caught) == expected_count:
        return
    raise ValueError(
        "Expected {} deprecation warnings but got {} instead:\n".format(
            expected_count, len(caught)) + "\n".join([str(x) for x in caught]))


@contextlib.contextmanager
def catch_drake_warnings(action="always", expected_count=None):
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter(action, DrakeDeprecationWarning)
        yield caught
    # N.B. If an error occurs in the contained context, this will not get
    # executed. This is intended behavior; see #10924 for more information.
    _check_expected(caught, expected_count)
