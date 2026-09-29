#ifndef DAQP_H
# define DAQP_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "factorization.h"
#include "constants.h"
#include "auxiliary.h"

int daqp_ldp(DAQPWorkspace *work);
void ldp2qp_solution(DAQPWorkspace *work);
void reset_daqp_workspace(DAQPWorkspace *work);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_H
