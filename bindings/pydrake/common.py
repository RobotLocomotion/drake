from __future__ import absolute_import, division, print_function

from ._pydrake_common import *

# When running from python, turn DRAKE_ASSERT and DRAKE_DEMAND failures into
# SystemExit, instead of process aborts.  See RobotLocomotion/drake#5268.
set_assertion_failure_to_throw_exception()
