#ifndef DAQP_UTILS_H
# define DAQP_UTILS_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "daqp.h"
#include "eq_elim.h"
// Utils for transforming QP to LDP
int daqp_update_ldp(const int mask, DAQPWorkspace *work, DAQPProblem *qp);
int daqp_update_Rinv(DAQPWorkspace *work, c_float *H, int is_factored);
c_float daqp_get_proximal_regularization(const DAQPWorkspace *work);
int daqp_update_M(DAQPWorkspace *work, c_float *A);
void daqp_update_v(c_float *f, DAQPWorkspace *work);
int daqp_update_d(DAQPWorkspace *work, c_float *bupper, c_float *blower);
int daqp_check_bounds(DAQPWorkspace* work, c_float* bupper, c_float* blower);
void daqp_normalize_Rinv(DAQPWorkspace *work);
int daqp_normalize_M(DAQPWorkspace *work);
int daqp_check_unconstrained(DAQPWorkspace* work, const int mask);

int daqp_update_avi(DAQPAVI *avi, DAQPProblem *problem, c_float zero_tol);
int daqp_retry_avi_with_reduced_rho(DAQPWorkspace *work);
int daqp_lu(c_float* A, int* P, int n);
void daqp_lu_solve(c_float* LU, int* P, c_float* b, c_float* x, int n);


void daqp_minrep_work(int* is_redundant,DAQPWorkspace* work);
// Utils for profiling
#ifdef PROFILING
#ifdef _WIN32
#include <windows.h>
typedef struct{
    LARGE_INTEGER start;
    LARGE_INTEGER stop;
}DAQPtimer;
#else // not _WIN32
#include <time.h>
typedef struct{
    struct timespec start;
    struct timespec stop;
}DAQPtimer;
#endif // _WIN32


void tic(DAQPtimer *timer);
void toc(DAQPtimer *timer);
double get_time(DAQPtimer *timer);
#endif // PROFILING

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_UTILS_H
