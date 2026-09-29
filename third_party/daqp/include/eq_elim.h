#ifndef DAQP_EQ_ELIM_H
# define DAQP_EQ_ELIM_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include "types.h"

/*
 * Elimination of equality constraints (see eq_elim.c).
 *
 * The equalities are eliminated from the QP before it is turned into an LDP.
 * The reduced problem is an ordinary DAQPProblem, whose LDP is formed and
 * solved by the usual routines: it is swapped into the workspace (installed)
 * while it is formed or solved, and the workspace describes the original
 * problem otherwise (in particular work->qp, work->n, work->m and work->sense).
 */

// Whether the equality constraints of the workspace are eliminated
#define DAQP_IS_REDUCED(work) ((work)->eq != NULL && (work)->eq->active)

// Whether qp is to be reduced for an update with the given mask
// (DAQPSettings.eq_reduction; AUTO only with DAQP_UPDATE_eliminate in mask)
int daqp_eq_wanted(const DAQPWorkspace* work, const DAQPProblem* qp, const int mask);

/*
 * Form or update the reduction of qp and the LDP of the reduced problem.
 * update_ldp forms the LDP of a problem that is in the workspace.
 * Returns DAQP_EQ_NOT_REDUCED if nothing can be eliminated, a negative exit
 * flag if the equality constraints cannot be satisfied, and the return value
 * of update_ldp otherwise.
 */
#define DAQP_EQ_NOT_REDUCED 1
int daqp_eq_update(DAQPWorkspace* work, DAQPProblem* qp, int mask,
        int (*update_ldp)(int, DAQPWorkspace*, DAQPProblem*));

// Give up the reduction (the LDP of the original problem then has to be formed)
void daqp_eq_deactivate(DAQPWorkspace* work);

// Swap the reduced problem into (install) or out of (restore) the workspace.
// Install returns whether the reduced problem is installed.
int daqp_eq_install(DAQPWorkspace* work);
void daqp_eq_restore(DAQPWorkspace* work);

// Set the starting iterate of the reduced problem from x of the original one
void daqp_eq_set_primal_start(DAQPWorkspace* work, const c_float* x);

void free_daqp_eq(DAQPWorkspace* work);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_EQ_ELIM_H
