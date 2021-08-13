# This file is only used by the installed copy of Drake, to provide a
# compatiblity alias for "from drake import lcmt_foo".

from pydrake.lcmtypes import *

# Warn the user about the pending removal.
import warnings as _warnings
_warnings.warn(
    "Drake's lcmtypes for Python are now named pydrake.lcmtypes.lcmt_foo"
    " instead of drake.lcmt_foo. The 'drake' module name is deprecated."
    " Please adjust your import statements to match the new name, and"
    " remove the BUILD dependency on @drake//lcmtypes:lcmtypes_drake_py."
    " The old module will be removed from Drake on or after 2021-12-01.",
    category=DeprecationWarning)
