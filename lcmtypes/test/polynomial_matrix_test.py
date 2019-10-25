# This test is simply to confirm that nested LCM-generated python messages are
# code-gen'd correctly, and can be imported at will.  If something isn't
# working, it will raise an exception during instantiation or construction.

import drake.lcmt_polynomial
import drake.lcmt_polynomial_matrix

mat = drake.lcmt_polynomial_matrix()
mat.polynomials.append(drake.lcmt_polynomial())
mat.encode()
