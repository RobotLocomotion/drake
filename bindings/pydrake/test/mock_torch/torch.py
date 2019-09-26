# This is a mock version of torch for use with `rtld_global_warning_test`.

import os

# Naively use the symbol.
assert os.RTLD_GLOBAL is not None
