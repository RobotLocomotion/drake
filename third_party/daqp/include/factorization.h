#ifndef DAQP_FACTORIZATION_H
# define DAQP_FACTORIZATION_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "types.h"
#include "constants.h"

c_float daqp_dot(const c_float* v1, const c_float* v2, const int n);

static inline c_float daqp_dot_inline(const c_float* v1, const c_float* v2, const int n) {
    c_float sum = 0.0;
    for (int i = 0; i < n; i++) sum += v1[i] * v2[i];
    return sum;
}
// rho is the reciprocal quadratic weight for a free soft slack, zero otherwise
void daqp_update_LDL_add(DAQPWorkspace *work, const int add_ind, const c_float rho);
void daqp_update_LDL_remove(DAQPWorkspace *work, const int rm_ind);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_FACTORIZATION_H
