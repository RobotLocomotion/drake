#ifndef DAQP_HIERARCHICAL_H
# define DAQP_HIERARCHICAL_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "types.h"
#include "constants.h"
#include "daqp.h"

int daqp_hiqp(DAQPWorkspace *work, c_float *lambda);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_HIERARCHICAL_H
