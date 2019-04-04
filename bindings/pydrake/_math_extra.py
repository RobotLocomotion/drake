import operator

import numpy as np

# Add common scalar binary ufuncs for use with symbolic scalars (dtype=object).
# These enable working around #8315.
# N.B. Defined in order listed in Python documentation:
# https://docs.python.org/3.6/library/operator.html
lt = np.vectorize(operator.lt)
le = np.vectorize(operator.le)
eq = np.vectorize(operator.eq)
ne = np.vectorize(operator.ne)
ge = np.vectorize(operator.ge)
gt = np.vectorize(operator.gt)
