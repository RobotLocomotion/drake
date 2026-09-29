#ifndef DAQP_TYPES_H
# define DAQP_TYPES_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

#ifdef DAQP_SINGLE_PRECISION
typedef float c_float;
#else
typedef double c_float;
#endif

typedef struct{

    // Data for the QP problem
    //
    // min  0.5 x'*H*x + f'x
    // s.t   lb  <=  x  <= ub
    //       lbA <= A*x <= ubA
    //
    // n  - dimension of x
    // m  - total number of constraints
    // ms - number of simple bounds
    // blower = [lb; lbA];
    // bupper = [ub; ubA];
    // (The number of rows in A is hence m-ms)

    // sense define the state of the constraints
    // (active, immutable, upper/lower, soft, binary).

    int n;
    int m;
    int ms;

    c_float* H;
    c_float* f;

    c_float* A;
    c_float* bupper;
    c_float* blower;

    int* sense;

    // Hierarchical QP
    int* break_points;
    int nh;
    // Extra flags for problem
    int problem_type; // 1 == AVI otherwise QP
}DAQPProblem;

typedef struct{
    c_float primal_tol;
    c_float dual_tol;
    c_float zero_tol;
    c_float pivot_tol;
    c_float progress_tol;

    int cycle_tol;
    int iter_limit;
    c_float fval_bound;

    c_float eps_prox;
    c_float eta_prox;

    c_float rho_soft;

    c_float rel_subopt;
    c_float abs_subopt;

    c_float sing_tol;
    c_float refactor_tol;
    c_float time_limit;

    // Linear weight for soft constraints, 0 gives a pure quadratic penalty
    c_float w_soft;

    // Equality-reduction policy: DAQP_EQ_REDUCTION_{OFF,AUTO,ON}
    int eq_reduction;
}DAQPSettings;


typedef struct{
    int bin_id;
    int depth;
    int WS_start;
    int WS_end;
}DAQPNode;

typedef struct{
    int* bin_ids;
    int nb;
    int neq;

    DAQPNode* tree;
    int  n_nodes;

    int* tree_WS;
    int nWS;
    int n_clean;
    int* fixed_ids;

    int nodecount;
    int itercount;

    int* root_WS; // Working set of the latest root relaxation (warm starts the next solve)
    int n_root_WS;
}DAQPBnB;

typedef struct{
    int is_symmetric;
    int retry_rho_needed;

    c_float* Hsym;
    c_float* Hs_rho;
    c_float* H_rho;
    int* P_H2;

    c_float* LU_H;
    int* P_H;

    c_float* kkt_buffer;
    int* P_S;

    c_float* xtemp;
    c_float* Hx;
    c_float* x;
    c_float* y;

    c_float rho;
}DAQPAVI;

/*
 * The parts of the workspace that describe the LDP that is solved. An
 * equality elimination keeps a second set for its reduced problem, which is
 * swapped into the workspace while that problem is formed or solved, so that
 * the solvers only ever see an ordinary problem.
 */
typedef struct{
    DAQPProblem* qp;
    int n;
    int m;
    int ms;
    c_float *M;
    c_float *dupper;
    c_float *dlower;
    c_float *Rinv;
    c_float *RinvD;
    c_float *v;
    c_float *scaling;
    c_float *Mu;
    int *sense;
    c_float *rho_ls;
    c_float *rho_us;
    c_float *w_ls;
    c_float *w_us;
    int state;
    int n_prox;
    int *bin_ids; // Binary constraints (if BnB)
    int nb;
}DAQPLDPData;

/*
 * Elimination of equality constraints, applied to the QP before it is turned
 * into an LDP (see eq_elim.c).
 *
 * The equality constraints A_E x = b_E are eliminated through x = xp + W w,
 * where the columns of W span the null space of A_E and xp is a particular
 * solution. If the reduced Hessian is positive definite, W is chosen with
 * W'HW = I and xp as the minimizer over the equality constraints, so that the
 * reduced problem is min 0.5||w||^2 over the remaining constraints (no
 * factorization is needed to form its LDP). Otherwise W is orthonormal and the
 * reduced problem keeps the Hessian W'HW (a singular QP, an LP, or an AVI).
 */
typedef struct{
    int n;  // Number of primal variables of the original problem
    int m;  // Number of constraints of the original problem
    int ms; // Number of simple bounds of the original problem
    int neq; // Number of eliminated equality constraints
    int nz; // Number of variables of the reduced problem (n-neq)
    int mr; // Number of constraints of the reduced problem
    int ndrop; // Number of constraints that the equalities imply
    int ncand; // Number of equality candidates
    int path; // DAQP_EQ_PATH_*
    int metric; // Diagonal Hessian (QR in the metric of H)
    int active; // A reduction is formed
    int error; // Exit flag if the latest right-hand side is infeasible (else 0)
    int installed; // The reduced problem is in the workspace

    int* eq_ids; // Eliminated equalities (ascending)
    int* cand_ids; // Candidates that the reduction was formed for
    int* keep; // Original index of each constraint of the reduced problem
    int* drop_ids; // Constraints that are implied by the equalities
    c_float* V; // Householder vectors (leading neq columns), then Z = Q2
    c_float* tau; // Householder scalars
    c_float* s_eq; // Normalization of the eliminated equalities
    c_float* R; // Triangular factor of the (normalized) A_E' (packed by columns)
    c_float* dsq; // H^{-1/2} for a diagonal Hessian
    c_float* W; // Null-space basis (n x nz, row major)
    c_float* xp; // Particular solution
    c_float fp; // Objective function value at xp
    c_float* tmp; // Scratch of size 3n

    /*
     * For a warm-started workspace, xp, H xp, and the shifts of the bounds are
     * linear in b_E and f. The response to each equality with a nonzero
     * right-hand side is kept (cols[k] = [xp_k (n), H xp_k (n), shift_k (mr)],
     * formed when first needed), as is the response to f (xf, gf, df).
     */
    c_float** cols;
    int ncols; // Number of entries in cols
    c_float* xf;
    c_float* gf;
    c_float* df;
    c_float* sh;
    int f_valid;

    // The reduced problem and the storage of its data
    DAQPProblem qp;
    c_float* Hr;
    c_float* fr;
    c_float* Ar;
    c_float* bur;
    c_float* blr;
    int* sr;
    c_float* rho_r; // Soft weights of the reduced constraints (one block)
    // The LDP of the problem that is not in the workspace (the reduced one,
    // unless it is installed)
    DAQPLDPData other;
}DAQPEqElim;

typedef struct{
    DAQPProblem* qp;
    // LDP data
    int n; // Number of primal variables
    int m; // Number of constraints
    int ms; // Number of simple bounds
    c_float *M; // M' M is the Hessian of the dual objective function (dimensions: n x m)
    c_float *dupper; // Linear part of dual objective function (dimensions: m x 1)
    c_float *dlower; // Linear part of dual objective function (dimensions: m x 1)
    c_float *Rinv; // Inverse of upper cholesky factor of primal Hessian
    c_float *v; // v = R'\f (used to transform QP to LDP
    int *sense; // State of constraints
    c_float *scaling; // normalizations
    c_float *RinvD; // in case Rinv is diagonal


    // Iterates
    c_float *x; // The final primal solution
    c_float *xold; // The latest primal solution (used for proximal-point iteratios)

    c_float* lam; // Dual iterate
    c_float* lam_star; // Current constrained stationary point
    c_float* u; // Stores Mk' lam_star
    c_float fval;

    // LDL factors (Mk Mk' = L D L')
    c_float *L;
    c_float *D;
    // Intermittent variables (LDL')
    c_float* xldl; // Solution to L xdldl = -dk
    c_float* zldl; // zldl_i = xldl_i/D_i
    int reuse_ind; // How much work that can be saved when solving Mk Mk' lam* = -dk

    int *WS; // Working set, size: maximum number of constraints (n+ns+1)
    int n_active; // Number of active contraints

    int iterations;
    int sing_ind; // Flag for denoting whether Mk Mk' is singular or not

    // Proximal support. Diagonal Hessians can regularize individual
    // directions; dense singular Hessians use a full shift for stability.
    int* prox_mask;
    int  n_prox; // Number of directions that needed regularization


    // Largest violation of a soft constraint in the returned solution
    c_float soft_slack;

    // Settings
    DAQPSettings* settings;

    // BnB
    DAQPBnB* bnb;
    // Hierarchical QP
    int nh;
    int* break_points;
    // AVI
    DAQPAVI* avi;
    // Equality elimination (NULL if the LDP is not reduced)
    DAQPEqElim* eq;
    // Timer (used for time limit checking, set externally by daqp_solve)
    void *timer;
    // M*u from the latest feasibility scan (length m-ms); NULL disables batching
    c_float *Mu;

    /* Penalties of the soft constraints, in the scale of the original problem:
     * the objective gains w*s + s^2/(2*rho) per violated side, so w is the
     * linear (L1) weight and rho the *reciprocal* quadratic (L2) one (see the
     * documentation on soft constraints). A zero entry selects settings->w_soft
     * and settings->rho_soft, which is what every soft constraint uses when
     * DAQP_NO_SOFT_WEIGHTS is set.
     *
     * The arrays are NULL until daqp_allocate_soft_weights is called, so a
     * solve with uniform weights neither spends the memory nor reads them, and
     * they are last in the workspace to preserve the offsets of the fields
     * above and keep the layout common to builds with and without support.
     */
    c_float *rho_ls; // Reciprocal quadratic weight (default settings->rho_soft)
    c_float *rho_us;
    c_float *w_ls; // Linear weight (default settings->w_soft)
    c_float *w_us;

    int state;
}DAQPWorkspace;

#define DAQP_IS_HIERARCHICAL(work) \
    ((work)->break_points != NULL && (work)->nh > 1)

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif //ifndef DAQP_TYPES_H
