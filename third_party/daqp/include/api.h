#ifndef DAQP_API_H
# define DAQP_API_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "daqp.h"
#include "daqp_prox.h"
#include "bnb.h"
#include "hierarchical.h"
#include "avi.h"
#include "eq_elim.h"

typedef struct{
    c_float *x;
    c_float *lam;
    c_float fval;
    c_float soft_slack; // Largest violation of a soft constraint

    int exitflag;
    int iter;
    int nodes;
    c_float solve_time;
    c_float setup_time;

}DAQPResult;

void daqp_solve(DAQPResult* res, DAQPWorkspace *work);
void daqp_quadprog(DAQPResult* res, DAQPProblem* qp,DAQPSettings* settings);
void daqp_avi(DAQPResult *res, DAQPProblem* problem, DAQPSettings *settings);

int setup_daqp(DAQPProblem *qp, DAQPWorkspace* work, c_float* setup_time);
int setup_daqp_main(DAQPProblem *qp, DAQPWorkspace* work, c_float* setup_time, int init_mask);
int setup_daqp_ldp(DAQPWorkspace *work, DAQPProblem* qp, const int init_mask);
void setup_daqp_hiqp(DAQPWorkspace *work, int* break_points, int nh);
int setup_daqp_bnb(DAQPWorkspace* work, int* sense, int nb, int ns);
int setup_daqp_avi(DAQPAVI* avi, DAQPProblem* p, DAQPWorkspace* work, c_float* setup_time);

void allocate_daqp_settings(DAQPWorkspace *work);
void allocate_daqp_workspace(DAQPWorkspace *work, int n, int ns);
void allocate_daqp_ldp(DAQPWorkspace *work, int n, int m, int ms, int alloc_R, int alloc_v);
void allocate_daqp_avi(DAQPAVI *avi, int n);
// Per-constraint soft weights (both return 0 if the build has no support)
int  daqp_allocate_soft_weights(DAQPWorkspace *work);
int  daqp_set_soft_weights(DAQPWorkspace *work, c_float *rho_l, c_float *rho_u,
        c_float *w_l, c_float *w_u);
// Call after writing settings->rho_soft or settings->w_soft on a live workspace.
void daqp_refresh_soft_weights(DAQPWorkspace *work);

void free_daqp_ldp(DAQPWorkspace *work);
void free_daqp_workspace(DAQPWorkspace *work);
void free_daqp_bnb(DAQPWorkspace* work);
void free_daqp_avi(DAQPWorkspace* work);

void daqp_extract_result(DAQPResult* res, DAQPWorkspace* work);
// Turn the result of the installed reduced problem (see eq_elim.h) into the
// result of the original problem, and carry its working set over to the
// original constraints
void daqp_eq_expand(DAQPResult* res, DAQPWorkspace* work);
void daqp_extract_active_duals(DAQPResult* res, DAQPWorkspace* work);
void daqp_default_settings(DAQPSettings *settings);
void daqp_minrep(int* is_redundant, c_float* A, c_float* b, int n, int m, int ms);
int  daqp_first_violating(c_float* x, c_float* A, c_float* bu, c_float* bl, int n, int m, int ms, c_float tol);

void daqp_primal_init_active(DAQPProblem* qp, c_float* x);
void daqp_dual_init_active(DAQPProblem* qp, c_float* lam);
void daqp_set_primal_start(DAQPWorkspace* work, c_float* x);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_API_H
