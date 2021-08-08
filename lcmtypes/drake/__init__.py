# This file is only used by the installed copy of Drake, to provide a
# compatiblity alias for "from drake import lcmt_foo".

from pydrake.common.deprecation import _warn_deprecated
from pydrake.lcmtypes import *

_warn_deprecated(
    "Drake's lcmtypes for Python are now named pydrake.lcmtypes.lcmt_foo"
    " instead of drake.lcmt_foo. The 'drake' package name is deprecated."
    " Please adjust your import statements to match the new name.",
    date="2022-12-01")
