# This is a mock version of torch for use with `rtld_global_warning_test`,
# simulating the following line:
# https://github.com/pytorch/pytorch/blob/v1.0.0/torch/__init__.py#L75

import os as _dl_flags
import sys


# Make the check in `pydrake/__init__.py` pass, but then undo the change.
_old_flags = sys.getdlopenflags()
sys.setdlopenflags(_dl_flags.RTLD_GLOBAL)
sys.setdlopenflags(_old_flags)
