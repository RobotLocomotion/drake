#ifndef DAQP_CONSTANTS_H
#define DAQP_CONSTANTS_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#include <stddef.h>

// Individual weights for the soft constraints, unless explicitly disabled
#ifndef DAQP_NO_SOFT_WEIGHTS
#define DAQP_SOFT_WEIGHTS
#endif

#define DAQP_EMPTY_IND -1
#define DAQP_UNCONSTRAINED_OPTIMAL -2
#define DAQP_INF ((c_float)1e30)

// DEFAULT SETTINGS
#define DAQP_DEFAULT_PRIM_TOL 1e-6
#define DAQP_DEFAULT_DUAL_TOL 1e-12
#define DAQP_DEFAULT_ZERO_TOL 1e-11
#define DAQP_DEFAULT_PROG_TOL 1e-14
#define DAQP_DEFAULT_PIVOT_TOL 1e-6
#define DAQP_DEFAULT_CYCLE_TOL 10
#define DAQP_DEFAULT_ETA -1.0
#define DAQP_AUTO_ETA_CAP 1e-6
#define DAQP_DEFAULT_ITER_LIMIT 10000
#define DAQP_DEFAULT_RHO_SOFT 1e-6
#define DAQP_DEFAULT_W_SOFT 0
#define DAQP_DEFAULT_REL_SUBOPT 0
#define DAQP_DEFAULT_ABS_SUBOPT 0
#define DAQP_DEFAULT_SING_TOL (3.7e-11)
#define DAQP_DEFAULT_REFACTOR_TOL 1e-9
#define DAQP_DEFAULT_EPS_PROX (-1e-6)

// Equality-reduction policy (DAQPSettings.eq_reduction). AUTO only reduces
// solves that start from scratch, which the setup/update marks with
// DAQP_UPDATE_eliminate (daqp_quadprog, the interfaces' one-shot solves, and
// the Eigen interface without warm start); ON also reduces a warm-started
// workspace that is updated and solved repeatedly
#define DAQP_EQ_REDUCTION_OFF (-1)
#define DAQP_EQ_REDUCTION_AUTO 0
#define DAQP_EQ_REDUCTION_ON 1

// Minimum number of iterations for daqp_refine_primal to be applied (prox)
#define DAQP_REFINE_MIN_ITER 5
// A Hessian with a larger condition number (estimate) is refined (daqp_refine_primal)
#define DAQP_REFINE_COND 1e6
// Refine if the rounding errors (about DAQP_REFINE_GAIN*eps*max(|u|,|v|)/min(D))
// might exceed primal_tol
#define DAQP_REFINE_GAIN 1e3

// eps (relative to max(H_ii)) used when a semi-proximal inner problem fails
#define DAQP_PROX_EPS_MAX 1e-3
// Regularize a reduced Hessian of an H with zero rows if cond > DAQP_HESSIAN_COND_MAX,
// and any Hessian if n*eps*cond > DAQP_HESSIAN_COND_EPS
#define DAQP_HESSIAN_COND_MAX 1e8
#define DAQP_HESSIAN_COND_EPS 0.1

// How the reduced problem of an equality elimination is posed
#define DAQP_EQ_PATH_LDP 0 // Identity Hessian, no linear term (H PD on the null space)
#define DAQP_EQ_PATH_QP 1  // Reduced Hessian W'HW (singular QP or AVI)
#define DAQP_EQ_PATH_LP 2  // No Hessian

// Equality constraints are eliminated if there are sufficiently many of them
// (neq > EQ_MIN_COUNT and EQ_MIN_RATIO*neq > n)
#define DAQP_EQ_MIN_COUNT 5
#define DAQP_EQ_MIN_RATIO 10
#define DAQP_EQ_MIN_DIM 20
// Diagonal Hessians require at least n/DAQP_EQ_DIAG_MIN_RATIO equalities
#define DAQP_EQ_DIAG_MIN_RATIO 4


// MACROS
#define DAQP_ARSUM(x) ((x)*(x+1)/2)
#define DAQP_R_OFFSET(X,Y) (((2*Y-X-1)*X)/2)

// EXIT FLAGS
#define DAQP_EXIT_SOFT_OPTIMAL 2
#define DAQP_EXIT_OPTIMAL 1
#define DAQP_EXIT_INFEASIBLE -1
#define DAQP_EXIT_CYCLE -2
#define DAQP_EXIT_UNBOUNDED -3
#define DAQP_EXIT_ITERLIMIT -4
#define DAQP_EXIT_NONCONVEX -5
#define DAQP_EXIT_OVERDETERMINED_INITIAL -6
#define DAQP_EXIT_TIMELIMIT -7
#define DAQP_EXIT_UNSUPPORTED -8

// UPDATE LDP MASKS
#define DAQP_UPDATE_Rinv 1
#define DAQP_UPDATE_M 2
#define DAQP_UPDATE_v 4
#define DAQP_UPDATE_d 8
#define DAQP_UPDATE_sense 16
#define DAQP_UPDATE_hierarchy 32
#define DAQP_UPDATE_unconstrained 64
// Lets DAQP_EQ_REDUCTION_AUTO eliminate the equality constraints of the LDP
// that this setup/update forms (for a solve that starts from scratch)
#define DAQP_UPDATE_eliminate 128

// WORKSPACE STATE MASKS
// The DAQP_UPDATE_* bits in DAQP_STATE_PENDING mark the parts of the LDP that
// an earlier update did not form, which the next update then forms
#define DAQP_STATE_PENDING (DAQP_UPDATE_Rinv+DAQP_UPDATE_M+DAQP_UPDATE_v+DAQP_UPDATE_d+DAQP_UPDATE_sense+DAQP_UPDATE_hierarchy)
#define DAQP_STATE_UNCONSTRAINED 256 // The unconstrained optimum is the solution
#define DAQP_STATE_RINV_NORMALIZED 512 // The first ms rows of Rinv are normalized
#define DAQP_STATE_INCUMBENT 1024 // work->x holds a candidate solution for BnB
#define DAQP_STATE_ILL_CONDITIONED 2048 // cond(H) (estimate) above DAQP_REFINE_COND

// CONSTRAINT MASKS
#define DAQP_ACTIVE 1
#define DAQP_IS_ACTIVE(x) (work->sense[x]&1)
#define DAQP_SET_ACTIVE(x) (work->sense[x]|=1)
#define DAQP_SET_INACTIVE(x) (work->sense[x]&=~1)

// marks if a constraints is active at its lower bound
#define DAQP_LOWER 2
#define DAQP_IS_LOWER(x) (work->sense[x]&2)
#define DAQP_SET_LOWER(x) (work->sense[x]|=2)
#define DAQP_SET_UPPER(x) (work->sense[x]&=~2)

// marks if a constraint cannot be activated/deactivated
#define DAQP_IMMUTABLE 4
#define DAQP_IS_IMMUTABLE(x) (work->sense[x]&4)
#define DAQP_SET_IMMUTABLE(x) (work->sense[x]|=4)
#define DAQP_SET_MUTABLE(x) (work->sense[x]&=~4)

// marks that a constraint might be violated (but the slack is penalized)
#define DAQP_SOFT 8
#define DAQP_IS_SOFT(x) (work->sense[x]&8)
#define DAQP_SET_SOFT(x) (work->sense[x]|=8)
#define DAQP_SET_HARD(x) (work->sense[x]&=~8)

// marks that a constraint has to be active at either its upper or lower bound
#define DAQP_BINARY 16
#define DAQP_IS_BINARY(x) (work->sense[x]&16)

// marks that the slack of a soft constraint is zero (see auxiliary.c)
#define DAQP_SLACK_FIXED 32
#define DAQP_IS_SLACK_FIXED(x) (work->sense[x]&32)
#define DAQP_IS_SLACK_FREE(x) ((work->sense[x]&32)==0)
#define DAQP_SET_SLACK_FIXED(x) (work->sense[x]|=32)
#define DAQP_SET_SLACK_FREE(x) (work->sense[x]&=~32)

// marks a constraint that is temporarily set aside (see gradient_step in daqp_prox.c)
#define DAQP_SET_ASIDE 64

// Internal marker: ACTIVE and IMMUTABLE were set by bound equality detection.
#define DAQP_AUTO_EQUALITY 128

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_CONSTANTS_H
