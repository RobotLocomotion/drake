import operator

import numpy as np

# Add common scalar binary ufuncs for use with any type.

# N.B. These enable logical comparison for arrays of Expression (dtype=object),
# where the output of the expression is not a bool (workaround for #8315).
# N.B. Defined in order listed in Python documentation:
# https://docs.python.org/3.6/library/operator.html
lt = np.vectorize(operator.lt)
le = np.vectorize(operator.le)
eq = np.vectorize(operator.eq)
ne = np.vectorize(operator.ne)
ge = np.vectorize(operator.ge)
gt = np.vectorize(operator.gt)
