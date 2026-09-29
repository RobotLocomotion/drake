#ifndef DAQP_AVI_H
# define DAQP_AVI_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "types.h"

int daqp_solve_avi(DAQPWorkspace* work);

void daqp_solve_avi_kkt(DAQPWorkspace* work);
int daqp_check_optimal_avi(DAQPWorkspace* work);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_AVI_H
