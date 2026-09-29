#ifndef DAQP_AUX_H
# define DAQP_AUX_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "types.h"
#include "constants.h"

void daqp_remove_constraint(DAQPWorkspace* work, const int rm_ind);
void daqp_add_constraint(DAQPWorkspace *work, const int add_ind, c_float lam);
void daqp_compute_primal_and_fval(DAQPWorkspace *work);
// Slack of the soft constraint active at working set index i, and the largest
// slack among them (in the units of the original problem)
c_float daqp_soft_slack(DAQPWorkspace *work, const int i);
c_float daqp_max_soft_slack(DAQPWorkspace *work);
void daqp_compute_Mu(DAQPWorkspace *work);
int daqp_add_infeasible(DAQPWorkspace *work);
int daqp_remove_blocking(DAQPWorkspace *work);
void daqp_compute_CSP(DAQPWorkspace *work);
void daqp_refine_active(DAQPWorkspace *work);
void daqp_refine_primal(DAQPWorkspace *work);
void daqp_compute_singular_direction(DAQPWorkspace *work);

void daqp_pivot_last(DAQPWorkspace *work);

int daqp_drop_singular_last(DAQPWorkspace *work);
int daqp_activate_constraints(DAQPWorkspace *work);
void daqp_deactivate_constraints(DAQPWorkspace *work);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_AUX_H
