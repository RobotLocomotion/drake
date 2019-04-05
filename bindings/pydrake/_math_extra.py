import operator

import numpy as np

# Add generic logical operators as ufuncs so that we may do comparisons on
# arrays of any scalar type, without restriction on the output type.
# These are added solely to work around #8315, where arrays of Expression can't
# use direct logical operators (e.g. `<=`) since the output type is not bool.
# N.B. Defined in order listed in Python documentation:
# https://docs.python.org/3.6/library/operator.html
lt = np.vectorize(operator.lt)
le = np.vectorize(operator.le)
eq = np.vectorize(operator.eq)
ne = np.vectorize(operator.ne)
ge = np.vectorize(operator.ge)
gt = np.vectorize(operator.gt)
