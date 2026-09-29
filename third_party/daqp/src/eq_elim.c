#include "api.h"
#include "utils.h"
#include "eq_elim.h"
#include "constants.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

// Keeps a hot kernel out of line, so that its generated code does not
// depend on the code it would otherwise be inlined into
#if defined(__GNUC__) || defined(__clang__)
#define DAQP_NOINLINE __attribute__((noinline))
#elif defined(_MSC_VER)
#define DAQP_NOINLINE __declspec(noinline)
#else
#define DAQP_NOINLINE
#endif

/*
 * Elimination of the equality constraints A_E x = b_E of
 *   min 0.5 x'Hx + f'x  s.t.  blower <= [x; A x] <= bupper
 * before the problem is turned into an LDP.
 *
 * The QR factorization A_E' = Q [R; 0] (of the normalized rows) gives the null
 * space Z = Q2 of A_E and the particular solution Q1 R^{-T} b_E. With x = xp +
 * W w the remaining constraints become constraints on w, the simple bounds
 * turning into general constraints, and the reduced problem is posed in one of
 * three ways (eq->path):
 *
 * DAQP_EQ_PATH_LDP: If Z'HZ = L L' is positive definite, W = Z L^{-T} gives
 *   W'HW = I and xp is taken as the minimizer over the equality constraints,
 *   so that W'(H xp + f) = 0 and the reduced problem is min 0.5||w||^2. Its LDP
 *   is formed without any factorization. For a diagonal (positive) Hessian,
 *   the QR is formed in the metric of H instead (A_E H^{-1/2} = [R' 0] Q'),
 *   which gives W = H^{-1/2} Z directly.
 * DAQP_EQ_PATH_QP: Otherwise (a singular reduced Hessian, or a nonsymmetric one
 *   for an AVI), W = Z and the reduced problem keeps the Hessian Z'HZ and the
 *   linear term Z'(H xp + f), which the proximal method (or the AVI solver)
 *   then handles in the reduced dimension.
 * DAQP_EQ_PATH_LP: For an LP, W = Z and the linear term is Z'f.
 *
 * Only the right-hand side (xp and the shifted bounds) depends on b and f, so
 * that an update of those does not redo the factorizations.
 */

/* ---------------------------------------------------------------------------
 * Kernels
 * -------------------------------------------------------------------------*/

// y <-- (I - tau v v') y on the indices >= k (v[k] = 1 implicitly)
static void eq_reflect(const c_float* v, const c_float tau, const int k, const int n, c_float* y){
    int i;
    c_float w = y[k];
    for(i = k+1; i < n; i++) w += v[i]*y[i];
    w *= tau;
    y[k] -= w;
    for(i = k+1; i < n; i++) y[i] -= w*v[i];
}

// Y <-- Q'Y for cnt vectors of length n (stride ld) with the reflectors
// 0..nr-1, four vectors per pass over a reflector
static DAQP_NOINLINE void eq_apply_QT_many(const c_float* V, const c_float* tau,
        const int nr, const int n, c_float* Y, const int cnt, const size_t ld){
    int i, j, k;
    for(j = 0; j+3 < cnt; j += 4){
        c_float *c0 = Y+j*ld, *c1 = c0+ld, *c2 = c1+ld, *c3 = c2+ld;
        for(k = 0; k < nr; k++){
            const c_float* v = V+(size_t)k*n;
            const c_float tk = tau[k];
            c_float w0 = c0[k], w1 = c1[k], w2 = c2[k], w3 = c3[k];
            for(i = k+1; i < n; i++){
                const c_float vi = v[i];
                w0 += vi*c0[i]; w1 += vi*c1[i]; w2 += vi*c2[i]; w3 += vi*c3[i];
            }
            w0 *= tk; w1 *= tk; w2 *= tk; w3 *= tk;
            c0[k] -= w0; c1[k] -= w1; c2[k] -= w2; c3[k] -= w3;
            for(i = k+1; i < n; i++){
                const c_float vi = v[i];
                c0[i] -= w0*vi; c1[i] -= w1*vi; c2[i] -= w2*vi; c3[i] -= w3*vi;
            }
        }
    }
    for(; j < cnt; j++)
        for(k = 0; k < nr; k++) eq_reflect(V+(size_t)k*n,tau[k],k,n,Y+j*ld);
}

// Z = Q[:,nr:] accumulated in the columns nr..n-1 of V (four at a time)
static DAQP_NOINLINE void eq_accumulate_Z(c_float* V, const c_float* tau, const int nr, const int n){
    int i, j, k;
    for(j = nr; j < n; j++){
        c_float* col = V+(size_t)j*n;
        for(i = 0; i < n; i++) col[i] = 0;
        col[j] = 1;
    }
    for(k = nr-1; k >= 0; k--){
        const c_float* v = V+(size_t)k*n;
        const c_float tk = tau[k];
        for(j = nr; j+3 < n; j += 4){
            c_float *c0 = V+(size_t)j*n, *c1 = c0+n, *c2 = c1+n, *c3 = c2+n;
            c_float w0 = c0[k], w1 = c1[k], w2 = c2[k], w3 = c3[k];
            for(i = k+1; i < n; i++){
                const c_float vi = v[i];
                w0 += vi*c0[i]; w1 += vi*c1[i]; w2 += vi*c2[i]; w3 += vi*c3[i];
            }
            w0 *= tk; w1 *= tk; w2 *= tk; w3 *= tk;
            c0[k] -= w0; c1[k] -= w1; c2[k] -= w2; c3[k] -= w3;
            for(i = k+1; i < n; i++){
                const c_float vi = v[i];
                c0[i] -= w0*vi; c1[i] -= w1*vi; c2[i] -= w2*vi; c3[i] -= w3*vi;
            }
        }
        for(; j < n; j++) eq_reflect(v,tk,k,n,V+(size_t)j*n);
    }
}

/*
 * C[i*ldc+j] = X_i . Y_j for the vectors X_i = X+i*ldx and Y_j = Y+j*ldy of
 * length n, in 4x4 register blocks. With upper, only j >= i is computed and
 * then mirrored.
 */
static DAQP_NOINLINE void eq_gemm_tn(const int n, const int p, const int q,
        const c_float* X, const size_t ldx, const c_float* Y, const size_t ldy,
        c_float* C, const size_t ldc, const int upper){
    int i, j, k, a, b;
    for(i = 0; i < p; i += 4){
        const int ib = (p-i < 4) ? p-i : 4;
        for(j = upper ? i : 0; j < q; j += 4){
            const int jb = (q-j < 4) ? q-j : 4;
            if(ib == 4 && jb == 4){
                const c_float *x0 = X+i*ldx, *x1 = x0+ldx, *x2 = x1+ldx, *x3 = x2+ldx;
                const c_float *y0 = Y+j*ldy, *y1 = y0+ldy, *y2 = y1+ldy, *y3 = y2+ldy;
                c_float s00 = 0, s01 = 0, s02 = 0, s03 = 0, s10 = 0, s11 = 0, s12 = 0, s13 = 0;
                c_float s20 = 0, s21 = 0, s22 = 0, s23 = 0, s30 = 0, s31 = 0, s32 = 0, s33 = 0;
                c_float* c;
                for(k = 0; k < n; k++){
                    const c_float a0 = x0[k], a1 = x1[k], a2 = x2[k], a3 = x3[k];
                    const c_float b0 = y0[k], b1 = y1[k], b2 = y2[k], b3 = y3[k];
                    s00 += a0*b0; s01 += a0*b1; s02 += a0*b2; s03 += a0*b3;
                    s10 += a1*b0; s11 += a1*b1; s12 += a1*b2; s13 += a1*b3;
                    s20 += a2*b0; s21 += a2*b1; s22 += a2*b2; s23 += a2*b3;
                    s30 += a3*b0; s31 += a3*b1; s32 += a3*b2; s33 += a3*b3;
                }
                c = C+i*ldc+j;
                c[0] = s00; c[1] = s01; c[2] = s02; c[3] = s03; c += ldc;
                c[0] = s10; c[1] = s11; c[2] = s12; c[3] = s13; c += ldc;
                c[0] = s20; c[1] = s21; c[2] = s22; c[3] = s23; c += ldc;
                c[0] = s30; c[1] = s31; c[2] = s32; c[3] = s33;
            }
            else{
                for(a = 0; a < ib; a++) for(b = 0; b < jb; b++){
                    const c_float *xa = X+(i+a)*ldx, *yb = Y+(j+b)*ldy;
                    c_float s = 0;
                    for(k = 0; k < n; k++) s += xa[k]*yb[k];
                    C[(i+a)*ldc+j+b] = s;
                }
            }
        }
    }
    if(upper)
        for(i = 0; i < p; i++) for(j = 0; j < i; j++) C[i*ldc+j] = C[j*ldc+i];
}

/*
 * B <-- P_k B P_k for k = 0..nr-1, for a symmetric B (lower triangle, row
 * major, ld = n). Only the trailing blocks B[k:,k:] are updated, which is all
 * that the trailing block B[nr:,nr:] = Z'BZ depends on.
 */
static DAQP_NOINLINE void eq_twoside(c_float* B, const int n, const c_float* V,
        const c_float* tau, const int nr, c_float* p){
    int i, j, k;
    for(k = 0; k < nr; k++){
        const c_float* v = V+(size_t)k*n;
        const c_float tk = tau[k];
        c_float pv = 0;
        // p = tk*B*v (v[k] = 1), a symmetric product with the lower triangle
        for(i = k; i < n; i++) p[i] = 0;
        for(i = k; i < n; i++){
            const c_float* Bi = B+(size_t)i*n;
            const c_float vi = (i == k) ? 1 : v[i];
            c_float s = Bi[k];
            for(j = k+1; j < i; j++){ s += Bi[j]*v[j]; p[j] += Bi[j]*vi; }
            if(i > k){ s += Bi[i]*vi; p[k] += Bi[k]*vi; }
            p[i] += s;
        }
        for(i = k; i < n; i++){ p[i] *= tk; pv += p[i]*((i == k) ? 1 : v[i]); }
        // w = p - (tk/2)(p'v) v, B <-- B - v w' - w v'
        pv *= 0.5*tk;
        for(i = k; i < n; i++) p[i] -= pv*((i == k) ? 1 : v[i]);
        for(i = k; i < n; i++){
            c_float* Bi = B+(size_t)i*n;
            const c_float vi = (i == k) ? 1 : v[i], wi = p[i];
            Bi[k] -= vi*p[k] + wi;
            for(j = k+1; j <= i; j++) Bi[j] -= vi*p[j] + wi*v[j];
        }
    }
}

/*
 * Cholesky factorization A = L L' (lower, row major, in place), with the rows
 * in blocks of four. Returns 0 if A is not positive definite in the sense of
 * daqp_update_Rinv (a pivot below zero_tol, or relative to the largest pivot).
 */
static DAQP_NOINLINE int eq_chol(c_float* A, const int n, const c_float zero_tol){
    int i, j, k, ib;
    c_float min_pivot = DAQP_INF, max_pivot = 0;
    for(ib = 0; ib < n; ib += 4){
        const int be = (ib+4 < n) ? ib+4 : n;
        for(j = 0; j < ib; j++){
            const c_float* Lj = A+(size_t)j*n;
            const c_float dj = 1/Lj[j];
            if(be-ib == 4){
                c_float *r0 = A+(size_t)ib*n, *r1 = r0+n, *r2 = r1+n, *r3 = r2+n;
                c_float s0 = r0[j], s1 = r1[j], s2 = r2[j], s3 = r3[j];
                for(k = 0; k < j; k++){
                    const c_float l = Lj[k];
                    s0 -= r0[k]*l; s1 -= r1[k]*l; s2 -= r2[k]*l; s3 -= r3[k]*l;
                }
                r0[j] = s0*dj; r1[j] = s1*dj; r2[j] = s2*dj; r3[j] = s3*dj;
            }
            else for(i = ib; i < be; i++){
                c_float* ri = A+(size_t)i*n;
                c_float s = ri[j];
                for(k = 0; k < j; k++) s -= ri[k]*Lj[k];
                ri[j] = s*dj;
            }
        }
        for(i = ib; i < be; i++){
            c_float* ri = A+(size_t)i*n;
            for(j = ib; j <= i; j++){
                const c_float* Lj = A+(size_t)j*n;
                c_float s = ri[j];
                for(k = 0; k < j; k++) s -= ri[k]*Lj[k];
                if(j < i) ri[j] = s/Lj[j];
                else{
                    if(s <= zero_tol) return 0;
                    if(s < min_pivot) min_pivot = s;
                    if(s > max_pivot) max_pivot = s;
                    ri[i] = sqrt(s);
                }
            }
        }
    }
    return min_pivot > zero_tol*max_pivot;
}

// Rows X_r <-- X_r L^{-T} (solves L x' = x'), cnt rows with stride ld
static DAQP_NOINLINE void eq_trsm_rows(const c_float* L, const int nz, c_float* X,
        const int cnt, const size_t ld){
    int i, j, k;
    for(i = 0; i+3 < cnt; i += 4){
        c_float *w0 = X+i*ld, *w1 = w0+ld, *w2 = w1+ld, *w3 = w2+ld;
        for(j = 0; j < nz; j++){
            const c_float* Lj = L+(size_t)j*nz;
            const c_float dj = 1/Lj[j];
            c_float s0 = w0[j], s1 = w1[j], s2 = w2[j], s3 = w3[j];
            for(k = 0; k < j; k++){
                const c_float l = Lj[k];
                s0 -= l*w0[k]; s1 -= l*w1[k]; s2 -= l*w2[k]; s3 -= l*w3[k];
            }
            w0[j] = s0*dj; w1[j] = s1*dj; w2[j] = s2*dj; w3[j] = s3*dj;
        }
    }
    for(; i < cnt; i++){
        c_float* w = X+i*ld;
        for(j = 0; j < nz; j++){
            const c_float* Lj = L+(size_t)j*nz;
            c_float s = w[j];
            for(k = 0; k < j; k++) s -= Lj[k]*w[k];
            w[j] = s/Lj[j];
        }
    }
}

// C = [a_ids] W for the rows ids of A (length n), with W row major n x nz
static DAQP_NOINLINE void eq_rows_times_W(const c_float* A, const int* ids, const int ms,
        const int cnt, const int n, const c_float* W, const int nz, c_float* C){
    int i, j, k;
    for(i = 0; i+3 < cnt; i += 4){
        const c_float *a0 = A+(size_t)(ids[i]-ms)*n, *a1 = A+(size_t)(ids[i+1]-ms)*n;
        const c_float *a2 = A+(size_t)(ids[i+2]-ms)*n, *a3 = A+(size_t)(ids[i+3]-ms)*n;
        c_float *c0 = C+(size_t)i*nz, *c1 = c0+nz, *c2 = c1+nz, *c3 = c2+nz;
        for(j = 0; j < 4*nz; j++) c0[j] = 0;
        for(k = 0; k < n; k++){
            const c_float* Wk = W+(size_t)k*nz;
            const c_float b0 = a0[k], b1 = a1[k], b2 = a2[k], b3 = a3[k];
            if(b0 == 0 && b1 == 0 && b2 == 0 && b3 == 0) continue;
            for(j = 0; j < nz; j++){
                const c_float w = Wk[j];
                c0[j] += b0*w; c1[j] += b1*w; c2[j] += b2*w; c3[j] += b3*w;
            }
        }
    }
    for(; i < cnt; i++){
        const c_float* a = A+(size_t)(ids[i]-ms)*n;
        c_float* c = C+(size_t)i*nz;
        for(j = 0; j < nz; j++) c[j] = 0;
        for(k = 0; k < n; k++){
            const c_float b = a[k];
            const c_float* Wk = W+(size_t)k*nz;
            if(b == 0) continue;
            for(j = 0; j < nz; j++) c[j] += b*Wk[j];
        }
    }
}

static c_float eq_dot(const c_float* a, const c_float* b, const int n){
    int i;
    c_float s = 0;
    for(i = 0; i < n; i++) s += a[i]*b[i];
    return s;
}

// y <-- H x (dense or diagonal H, which is known to be diagonal if metric)
static void eq_hess_times(const DAQPProblem* qp, const int metric, const c_float* x, c_float* y){
    int i;
    const int n = qp->n;
    if(qp->H == NULL){
        for(i = 0; i < n; i++) y[i] = 0;
        return;
    }
    if(metric){
        for(i = 0; i < n; i++) y[i] = qp->H[(size_t)i*n+i]*x[i];
        return;
    }
    for(i = 0; i < n; i++) y[i] = eq_dot(qp->H+(size_t)i*n,x,n);
}

/* ---------------------------------------------------------------------------
 * Deciding whether to eliminate
 * -------------------------------------------------------------------------*/

// Whether the general constraint i is an equality constraint that can be
// eliminated (the same criteria as daqp_check_bounds for equal bounds)
static int eq_is_candidate(const DAQPWorkspace* work, const DAQPProblem* qp, const int i){
    const int s = work->sense[i];
    if(s & (DAQP_SOFT+DAQP_BINARY)) return 0;
    if(qp->bupper[i] - qp->blower[i] < work->settings->zero_tol) return 1;
    if(s & DAQP_AUTO_EQUALITY) return 0; // Its bounds are no longer equal
    return (s & (DAQP_ACTIVE+DAQP_IMMUTABLE)) == (DAQP_ACTIVE+DAQP_IMMUTABLE);
}

static int eq_count_candidates(const DAQPWorkspace* work, const DAQPProblem* qp){
    int i, n_eq = 0;
    for(i = qp->ms; i < qp->m; i++)
        if(eq_is_candidate(work,qp,i)) n_eq++;
    return n_eq;
}

static int eq_is_diagonal(const DAQPProblem* qp, const c_float zero_tol){
    int i, j;
    const int n = qp->n;
    if(qp->H == NULL) return 0;
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            if(i != j && (qp->H[(size_t)i*n+j] > zero_tol || qp->H[(size_t)i*n+j] < -zero_tol))
                return 0;
    return 1;
}

/*
 * Whether eliminating n_eq equalities is expected to pay off. The reduction
 * turns every simple bound into a general constraint, which is a loss when a
 * diagonal Hessian makes a bound a single lookup and the equalities are the
 * only general constraints (multi-stage MPC, for instance).
 */
static int eq_is_worthwhile(const DAQPWorkspace* work, const DAQPProblem* qp, const int n_eq){
    const int n = qp->n;
    const int n_ineq = qp->m-qp->ms-n_eq;
    if(n < DAQP_EQ_MIN_DIM || n_eq <= DAQP_EQ_MIN_COUNT ||
            DAQP_EQ_MIN_RATIO*n_eq <= n) return 0;
    if(eq_is_diagonal(qp,work->settings->zero_tol) &&
            (n_ineq == 0 || DAQP_EQ_DIAG_MIN_RATIO*n_eq < n)) return 0;
    return 1;
}

int daqp_eq_wanted(const DAQPWorkspace* work, const DAQPProblem* qp, const int mask){
    int i, n_eq;
    const int policy = work->settings->eq_reduction;
    if(policy == DAQP_EQ_REDUCTION_OFF) return 0;
    if(policy != DAQP_EQ_REDUCTION_ON && !(mask&DAQP_UPDATE_eliminate)) return 0;
    if(qp->A == NULL || qp->m <= qp->ms || work->sense == NULL) return 0;
    // A hierarchy refers to the constraints by their index, and a factored
    // Hessian (problem_type 2) is not available as a Hessian
    if(qp->nh > 1 || DAQP_IS_HIERARCHICAL(work) || qp->problem_type == 2) return 0;
    n_eq = eq_count_candidates(work,qp);
    if(n_eq == 0) return 0;
    if(policy == DAQP_EQ_REDUCTION_ON) return 1;
    // The default weights of soft constraints refer to the normalization of
    // the full problem, which is left as it is by the automatic policy
    for(i = 0; i < qp->m; i++)
        if(work->sense[i] & DAQP_SOFT) return 0;
    return eq_is_worthwhile(work,qp,n_eq);
}

// Whether the equality constraints are the ones that the reduction was formed for
static int eq_same_candidates(const DAQPWorkspace* work, const DAQPProblem* qp){
    const DAQPEqElim* eq = work->eq;
    int i, k = 0;
    for(i = qp->ms; i < qp->m; i++){
        if(!eq_is_candidate(work,qp,i)) continue;
        if(k == eq->ncand || eq->cand_ids[k] != i) return 0;
        k++;
    }
    return k == eq->ncand;
}

/* ---------------------------------------------------------------------------
 * Storage
 * -------------------------------------------------------------------------*/

static void eq_free_reduced_ldp(DAQPEqElim* eq){
    DAQPLDPData* d = &eq->other;
    free(d->M); free(d->dupper); free(d->dlower); free(d->scaling); free(d->Mu);
    free(d->sense); free(d->Rinv); free(d->RinvD); free(d->v);
    free(d->bin_ids);
    memset(d,0,sizeof(DAQPLDPData));
    free(eq->rho_r);
    eq->rho_r = NULL;
}

// Storage of the LDP of the reduced problem (which has no simple bounds)
static void eq_allocate_reduced_ldp(DAQPEqElim* eq, const int nb){
    DAQPLDPData* d = &eq->other;
    const int nz = eq->nz, mr = eq->mr;
    eq_free_reduced_ldp(eq);
    d->qp = &eq->qp;
    d->n = nz; d->m = mr; d->ms = 0;
    d->M = malloc((size_t)nz*mr*sizeof(c_float));
    d->dupper = malloc(mr*sizeof(c_float));
    d->dlower = malloc(mr*sizeof(c_float));
    d->scaling = malloc(mr*sizeof(c_float));
    d->Mu = malloc(mr*sizeof(c_float));
    d->sense = malloc(mr*sizeof(int));
    d->Rinv = (eq->qp.H != NULL) ? malloc(((size_t)nz*(nz+1)/2)*sizeof(c_float)) : NULL;
    d->v = (eq->qp.f != NULL) ? malloc(nz*sizeof(c_float)) : NULL;
    d->bin_ids = (nb > 0) ? malloc(nb*sizeof(int)) : NULL;
}

static void eq_free_rhs_cache(DAQPEqElim* eq){
    int k;
    if(eq->cols != NULL) for(k = 0; k < eq->ncols; k++) free(eq->cols[k]);
    free(eq->cols); free(eq->xf); free(eq->gf); free(eq->df); free(eq->sh);
    eq->cols = NULL; eq->ncols = 0;
    eq->xf = eq->gf = eq->df = eq->sh = NULL;
    eq->f_valid = 0;
}

static void eq_free_reduction(DAQPEqElim* eq){
    eq_free_rhs_cache(eq);
    free(eq->eq_ids); free(eq->cand_ids); free(eq->keep); free(eq->drop_ids);
    free(eq->V); free(eq->tau); free(eq->s_eq); free(eq->R); free(eq->dsq);
    free(eq->W); free(eq->xp); free(eq->tmp);
    free(eq->Hr); free(eq->fr); free(eq->Ar); free(eq->bur); free(eq->blr); free(eq->sr);
    eq->eq_ids = eq->cand_ids = eq->keep = eq->drop_ids = eq->sr = NULL;
    eq->V = eq->tau = eq->s_eq = eq->R = eq->dsq = eq->W = eq->xp = eq->tmp = NULL;
    eq->Hr = eq->fr = eq->Ar = eq->bur = eq->blr = NULL;
}

// Storage that only depends on the dimensions of the original problem
static void eq_allocate_reduction(DAQPEqElim* eq, const int n, const int m, const int ms){
    if(eq->V != NULL && eq->n == n && eq->m == m && eq->ms == ms) return;
    eq_free_reduction(eq);
    eq->n = n; eq->m = m; eq->ms = ms;
    eq->eq_ids = malloc(m*sizeof(int));
    eq->cand_ids = malloc(m*sizeof(int));
    eq->keep = malloc(m*sizeof(int));
    eq->drop_ids = malloc(m*sizeof(int));
    eq->V = malloc((size_t)n*n*sizeof(c_float));
    eq->tau = malloc(n*sizeof(c_float));
    eq->s_eq = malloc(n*sizeof(c_float));
    eq->R = malloc(((size_t)n*(n+1)/2)*sizeof(c_float));
    eq->xp = malloc(n*sizeof(c_float));
    eq->tmp = malloc(3*(size_t)n*sizeof(c_float));
    eq->bur = malloc(m*sizeof(c_float));
    eq->blr = malloc(m*sizeof(c_float));
    eq->sr = malloc(m*sizeof(int));
}

/* ---------------------------------------------------------------------------
 * Forming the reduction
 * -------------------------------------------------------------------------*/

/*
 * Householder QR of A_E' (A_E H^{-1/2} if metric), left-looking in panels of
 * four candidates: the reflectors that are already formed are applied to the
 * whole panel at once, after which the panel is factorized column by column.
 * Candidates that are (numerically) linearly dependent on the ones before are
 * not eliminated; they are kept as constraints, which the equalities imply.
 */
static DAQP_NOINLINE void eq_build_qr(DAQPEqElim* eq, const DAQPProblem* qp, const c_float zero_tol){
    const int n = eq->n, ms = eq->ms;
    const c_float tol = sqrt(zero_tol);
    c_float* panel = malloc(4*(size_t)n*sizeof(c_float));
    c_float pn[4];
    int c, i, p, q, neq = 0;
    for(c = 0; c < eq->ncand && neq < n; c += 4){
        const int cnt = (eq->ncand-c < 4) ? eq->ncand-c : 4, k0 = neq;
        for(p = 0; p < cnt; p++){
            const c_float* a = qp->A+(size_t)(eq->cand_ids[c+p]-ms)*n;
            c_float* col = panel+(size_t)p*n;
            c_float nrm = 0;
            if(eq->metric) for(i = 0; i < n; i++) col[i] = a[i]*eq->dsq[i];
            else for(i = 0; i < n; i++) col[i] = a[i];
            for(i = 0; i < n; i++) nrm += col[i]*col[i];
            pn[p] = (nrm <= zero_tol) ? 0 : 1/sqrt(nrm);
            for(i = 0; i < n; i++) col[i] *= pn[p];
        }
        eq_apply_QT_many(eq->V,eq->tau,k0,n,panel,cnt,n);
        for(p = 0; p < cnt && neq < n; p++){
            c_float *col = panel+(size_t)p*n, alpha = 0, beta, d;
            if(pn[p] == 0) continue; // Empty constraint
            for(q = k0; q < neq; q++) eq_reflect(eq->V+(size_t)q*n,eq->tau[q],q,n,col);
            for(i = neq; i < n; i++) alpha += col[i]*col[i];
            alpha = sqrt(alpha);
            if(alpha <= tol) continue; // Linearly dependent
            beta = (col[neq] > 0) ? -alpha : alpha;
            d = col[neq]-beta;
            eq->tau[neq] = -d/beta;
            for(i = neq+1; i < n; i++) col[i] /= d;
            col[neq] = beta;
            for(i = 0; i <= neq; i++) eq->R[DAQP_ARSUM(neq)+i] = col[i];
            for(i = 0; i < n; i++) eq->V[(size_t)neq*n+i] = col[i];
            eq->s_eq[neq] = pn[p];
            eq->eq_ids[neq++] = eq->cand_ids[c+p];
        }
    }
    free(panel);
    eq->neq = neq;
}

// Symmetric part of an asymmetric Hessian (AVI) is not what the reduction needs
static int eq_is_symmetric(const DAQPProblem* qp, const c_float zero_tol){
    int i, j;
    const int n = qp->n;
    c_float scale = 0;
    for(i = 0; i < n; i++){
        const c_float d = qp->H[(size_t)i*n+i] < 0 ? -qp->H[(size_t)i*n+i] : qp->H[(size_t)i*n+i];
        if(d > scale) scale = d;
    }
    if(scale < 1) scale = 1;
    for(i = 0; i < n; i++)
        for(j = i+1; j < n; j++){
            const c_float diff = qp->H[(size_t)i*n+j]-qp->H[(size_t)j*n+i];
            if(diff > zero_tol*scale || diff < -zero_tol*scale) return 0;
        }
    return 1;
}

// Split Z (n x nz) into [Z1 Z2], with Z2 zero in the rows cid (the variables
// with curvature), so that H*Z2 = 0 exactly. Forms Hr = blockdiag(Z1'HZ1, 0)
// and returns the dimension of Z1
static DAQP_NOINLINE int eq_split_flat(const DAQPProblem* qp, c_float* Z, const int n,
        const int nz, const int* cid, const int nc, const c_float zero_tol, c_float* Hr){
    const c_float tol = sqrt(zero_tol);
    c_float *Y = calloc((size_t)nc*nz+1,sizeof(c_float)), *tau = calloc(nc+1,sizeof(c_float));
    c_float *Zr = malloc((size_t)n*nz*sizeof(c_float)), *G;
    int i, j, k, c, r = 0;

    // Householder QR of Z_C' (the rows cid of Z), with its rank r
    for(c = 0; c < nc && r < nz; c++){
        c_float *col = Y+(size_t)r*nz, alpha = 0, beta, d;
        for(j = 0; j < nz; j++) col[j] = Z[(size_t)j*n+cid[c]];
        for(k = 0; k < r; k++) eq_reflect(Y+(size_t)k*nz,tau[k],k,nz,col);
        for(j = r; j < nz; j++) alpha += col[j]*col[j];
        alpha = sqrt(alpha);
        if(alpha <= tol) continue; // Dependent on the rows before
        beta = (col[r] > 0) ? -alpha : alpha;
        d = col[r]-beta;
        tau[r] = -d/beta;
        for(j = r+1; j < nz; j++) col[j] /= d;
        col[r] = beta;
        r++;
    }

    // Z <-- Z Q: rows (Q'z')' (row major), the last nz-r columns are Z2
    for(i = 0; i < n; i++)
        for(j = 0; j < nz; j++) Zr[(size_t)i*nz+j] = Z[(size_t)j*n+i];
    eq_apply_QT_many(Y,tau,r,nz,Zr,n,nz);
    for(c = 0; c < nc; c++)
        for(j = r; j < nz; j++) Zr[(size_t)cid[c]*nz+j] = 0; // Roundoff
    for(i = 0; i < n; i++)
        for(j = 0; j < nz; j++) Z[(size_t)j*n+i] = Zr[(size_t)i*nz+j];

    // Hr = blockdiag(Z1_C' H_CC Z1_C, 0), with G = H_CC Z1_C (nc x r)
    for(i = 0; i < nz*nz; i++) Hr[i] = 0;
    G = malloc(((size_t)nc*r+1)*sizeof(c_float));
    for(c = 0; c < nc; c++){
        const c_float* Hc = qp->H+(size_t)cid[c]*n;
        for(j = 0; j < r; j++){
            c_float sm = 0;
            for(k = 0; k < nc; k++) sm += Hc[cid[k]]*Zr[(size_t)cid[k]*nz+j];
            G[(size_t)c*r+j] = sm;
        }
    }
    for(i = 0; i < r; i++)
        for(j = i; j < r; j++){
            c_float sm = 0;
            for(c = 0; c < nc; c++) sm += Zr[(size_t)cid[c]*nz+i]*G[(size_t)c*r+j];
            Hr[(size_t)i*nz+j] = Hr[(size_t)j*nz+i] = sm;
        }
    free(G); free(Y); free(tau); free(Zr);
    return r;
}

/*
 * Form the reduction of qp: the factorizations, the reduced constraints and the
 * storage of the reduced problem. Returns 0 if nothing (or everything) can be
 * eliminated.
 */
static int eq_build_reduction(DAQPWorkspace* work, DAQPProblem* qp){
    DAQPEqElim* eq = work->eq;
    const int n = qp->n, m = qp->m, ms = qp->ms;
    const c_float zero_tol = work->settings->zero_tol;
    int i, j, k, c, neq, nz, mI = 0, mtot, mr, nb = 0;
    int use_twoside = 0, use_refl = 0, symmetric = 1, split = 0, nc = 0;
    int *gen_ids, *cid = NULL;
    c_float* L = NULL;
    const c_float* Z;

    eq_allocate_reduction(eq,n,m,ms);
    eq_free_rhs_cache(eq); // Formed for the previous reduction
    eq->active = 0;
    if(work->bnb != NULL) work->bnb->n_root_WS = 0; // Refers to other constraints

    // Equality candidates
    eq->ncand = 0;
    for(i = ms; i < m; i++)
        if(eq_is_candidate(work,qp,i)) eq->cand_ids[eq->ncand++] = i;

    // A positive diagonal Hessian is used as a metric in the QR
    eq->metric = 0;
    if(qp->H != NULL && eq_is_diagonal(qp,zero_tol)){
        c_float scale = 0;
        for(i = 0; i < n; i++) if(qp->H[(size_t)i*n+i] > scale) scale = qp->H[(size_t)i*n+i];
        eq->metric = scale > 0;
        for(i = 0; i < n && eq->metric; i++)
            if(qp->H[(size_t)i*n+i] <= zero_tol*scale) eq->metric = 0;
        if(eq->metric){
            if(eq->dsq == NULL) eq->dsq = malloc(n*sizeof(c_float));
            for(i = 0; i < n; i++) eq->dsq[i] = 1/sqrt(qp->H[(size_t)i*n+i]);
        }
    }
    if(qp->H != NULL && !eq->metric && qp->problem_type == 1)
        symmetric = eq_is_symmetric(qp,zero_tol);

    eq_build_qr(eq,qp,zero_tol);
    neq = eq->neq;
    if(neq == 0 || neq == n) return 0; // Nothing (or everything) eliminated
    nz = eq->nz = n-neq;

    // The general constraints that are kept (dependent equalities included)
    gen_ids = malloc((m-ms)*sizeof(int));
    for(i = ms, k = 0; i < m; i++){
        if(k < neq && eq->eq_ids[k] == i){ k++; continue; }
        gen_ids[mI++] = i;
    }

    /*
     * Pick the kernels by their flop counts (multiply-adds). The two-sided
     * Householder product is memory bound, so its count is weighted.
     */
    if(qp->H != NULL && !eq->metric && symmetric){
        const c_float N = n, NZ = nz;
        const c_float f_gemm = N*N*NZ + N*NZ*NZ/2;
        const c_float f_two = 2.0/3.0*(N*N*N-NZ*NZ*NZ);
        use_twoside = 1.7*f_two < f_gemm;
    }
    {
        const c_float N = n, NZ = nz, NE = neq, MI = mI;
        const c_float f_w = MI*N*NZ;
        const c_float f_refl = MI*(2*N*NE + NZ*NZ/2);
        use_refl = f_refl < f_w;
    }

    eq_accumulate_Z(eq->V,eq->tau,neq,n);
    Z = eq->V+(size_t)neq*n; // Column j of Z at Z+j*n

    // Variables with curvature (nonzero rows of H), for eq_split_flat
    if(qp->H != NULL && !eq->metric && symmetric){
        cid = malloc(n*sizeof(int));
        for(i = 0; i < n; i++){
            const c_float* Hi = qp->H+(size_t)i*n;
            for(j = 0; j < n; j++) if(Hi[j] != 0 || qp->H[(size_t)j*n+i] != 0) break;
            if(j < n) cid[nc++] = i;
        }
        split = nc < nz; // Then some null directions have no curvature
    }

    // The reduced Hessian, and how the reduced problem is posed
    free(eq->Hr); eq->Hr = NULL;
    free(eq->fr); eq->fr = NULL;
    if(qp->H == NULL) eq->path = DAQP_EQ_PATH_LP;
    else if(eq->metric) eq->path = DAQP_EQ_PATH_LDP;
    else{
        eq->Hr = malloc((size_t)nz*nz*sizeof(c_float));
        if(split){
            eq_split_flat(qp,(c_float*)Z,n,nz,cid,nc,zero_tol,eq->Hr);
            use_refl = 0; // The rows (A Q)_2 refer to the unsplit Z
        }
        else if(use_twoside){
            c_float* B = malloc((size_t)n*n*sizeof(c_float));
            for(i = 0; i < n*n; i++) B[i] = qp->H[i];
            eq_twoside(B,n,eq->V,eq->tau,neq,eq->tmp);
            for(i = 0; i < nz; i++)
                for(j = 0; j <= i; j++)
                    eq->Hr[(size_t)i*nz+j] = eq->Hr[(size_t)j*nz+i] = B[(size_t)(neq+i)*n+neq+j];
            free(B);
        }
        else{
            c_float* HZ = malloc((size_t)n*nz*sizeof(c_float)); // Column j = H z_j
            eq_gemm_tn(n,nz,n,Z,n,qp->H,n,HZ,n,0); // (Z'H')_{ji} = (H Z)_{ij}
            eq_gemm_tn(n,nz,nz,Z,n,HZ,n,eq->Hr,nz,symmetric);
            free(HZ);
        }
        eq->path = DAQP_EQ_PATH_QP;
        if(symmetric){
            L = malloc((size_t)nz*nz*sizeof(c_float));
            for(i = 0; i < nz*nz; i++) L[i] = eq->Hr[i];
            if(eq_chol(L,nz,zero_tol)) eq->path = DAQP_EQ_PATH_LDP;
            else{ free(L); L = NULL; }
        }
    }

    // W (row major): H^{-1/2} Z, Z L^{-T}, or Z
    free(eq->W);
    eq->W = malloc((size_t)n*nz*sizeof(c_float));
form_W:
    for(i = 0; i < n; i++){
        c_float* Wi = eq->W+(size_t)i*nz;
        const c_float s = eq->metric ? eq->dsq[i] : 1;
        for(j = 0; j < nz; j++) Wi[j] = s*Z[(size_t)j*n+i];
    }
    if(L != NULL){
        eq_trsm_rows(L,nz,eq->W,n,nz);
        // Ill-conditioned Z'HZ => PATH_QP (where it is regularized). The squared
        // column norms of W are the diagonal of (Z'HZ)^-1
        const c_float eps_mach = sizeof(c_float) == sizeof(float) ? FLT_EPSILON : DBL_EPSILON;
        c_float wmax = 0, hmax = 0;
        for(j = 0; j < nz; j++){
            c_float s2 = 0;
            for(i = 0; i < n; i++) s2 += eq->W[(size_t)i*nz+j]*eq->W[(size_t)i*nz+j];
            if(s2 > wmax) wmax = s2;
            if(eq->Hr[(size_t)j*nz+j] > hmax) hmax = eq->Hr[(size_t)j*nz+j];
        }
        if((nc < n && wmax*hmax > DAQP_HESSIAN_COND_MAX) ||
                nz*eps_mach*wmax*hmax > DAQP_HESSIAN_COND_EPS){
            free(L); L = NULL;
            eq->path = DAQP_EQ_PATH_QP;
            goto form_W;
        }
    }
    if(eq->path == DAQP_EQ_PATH_LDP){
        if(eq->Hr == NULL) eq->Hr = calloc((size_t)nz*nz,sizeof(c_float));
        else for(i = 0; i < nz*nz; i++) eq->Hr[i] = 0;
        for(i = 0; i < nz; i++) eq->Hr[(size_t)i*nz+i] = 1;
    }
    else eq->fr = malloc(nz*sizeof(c_float));

    // Reduced constraints: rows of W for the simple bounds, then A_I W
    mtot = ms+mI;
    free(eq->Ar);
    eq->Ar = malloc((size_t)mtot*nz*sizeof(c_float));
    for(i = 0; i < ms; i++)
        for(j = 0; j < nz; j++) eq->Ar[(size_t)i*nz+j] = eq->W[(size_t)i*nz+j];
    if(mI > 0){
        c_float* AI = eq->Ar+(size_t)ms*nz;
        if(!use_refl) eq_rows_times_W(qp->A,gen_ids,ms,mI,n,eq->W,nz,AI);
        else{
            c_float* Ak = malloc((size_t)mI*n*sizeof(c_float));
            for(i = 0; i < mI; i++){
                const c_float* a = qp->A+(size_t)(gen_ids[i]-ms)*n;
                c_float* dst = Ak+(size_t)i*n;
                if(eq->metric) for(k = 0; k < n; k++) dst[k] = a[k]*eq->dsq[k];
                else for(k = 0; k < n; k++) dst[k] = a[k];
            }
            eq_apply_QT_many(eq->V,eq->tau,neq,n,Ak,mI,n); // Rows <-- (Q'a')'
            for(i = 0; i < mI; i++)
                for(j = 0; j < nz; j++) AI[(size_t)i*nz+j] = Ak[(size_t)i*n+neq+j];
            free(Ak);
            if(L != NULL) eq_trsm_rows(L,nz,AI,mI,nz);
        }
    }
    free(L);
    free(cid);

    /*
     * Keep the constraints that the reduced variables affect. The others are
     * implied by the equalities: they only have to be consistent with them.
     */
    mr = 0; eq->ndrop = 0;
    for(c = 0; c < mtot; c++){
        const int id = (c < ms) ? c : gen_ids[c-ms];
        const c_float* row = eq->Ar+(size_t)c*nz;
        if(eq_dot(row,row,nz) <= zero_tol){
            eq->drop_ids[eq->ndrop++] = id;
            continue;
        }
        if(mr != c) memmove(eq->Ar+(size_t)mr*nz,row,nz*sizeof(c_float));
        eq->keep[mr++] = id;
    }
    eq->mr = mr;
    free(gen_ids);
    for(c = 0; c < mr; c++){
        eq->sr[c] = work->sense[eq->keep[c]];
        if(eq->sr[c] & DAQP_BINARY) nb++;
    }

    // The reduced problem
    memset(&eq->qp,0,sizeof(DAQPProblem));
    eq->qp.n = nz; eq->qp.m = mr; eq->qp.ms = 0;
    eq->qp.H = eq->Hr;
    eq->qp.f = eq->fr;
    eq->qp.A = eq->Ar;
    eq->qp.bupper = eq->bur;
    eq->qp.blower = eq->blr;
    eq->qp.sense = eq->sr;
    eq->qp.nh = 1;
    eq->qp.problem_type = qp->problem_type;

    eq_allocate_reduced_ldp(eq,nb);
    return 1;
}

// Value of equality k (the upper bound, unless it is active at its lower one)
static c_float eq_value(const DAQPWorkspace* work, const DAQPProblem* qp, const int id){
    return (work->sense[id]&DAQP_LOWER) ? qp->blower[id] : qp->bupper[id];
}

/*
 * Response of xp to a unit right-hand side of eliminated equality k (in
 * PATH_LDP): y = Q1 R^{-T} e_k s_k (scaled by H^{-1/2} in the metric case) is
 * moved to the minimizer y - W W'H y, and H xp_k and the shifts -a_c xp_k of
 * the bounds of the kept constraints follow.
 */
static const c_float* eq_rhs_column(DAQPEqElim* eq, const DAQPProblem* qp, const int k){
    const int n = eq->n, ms = eq->ms, neq = eq->neq, nz = eq->nz, mr = eq->mr;
    c_float *col, *xk, *hk, *dk, *t = eq->tmp+2*n;
    int i, j, c;
    if(eq->cols == NULL){
        eq->cols = calloc(neq,sizeof(c_float*));
        eq->ncols = neq;
    }
    if(eq->cols[k] != NULL) return eq->cols[k];
    col = malloc((2*(size_t)n+mr)*sizeof(c_float));
    xk = col; hk = col+n; dk = col+2*n;
    // R'y = s_k e_k, forward substitution from k
    for(i = 0; i < n; i++) xk[i] = 0;
    for(i = k; i < neq; i++){
        const c_float* Ri = eq->R+DAQP_ARSUM(i);
        c_float s = (i == k) ? eq->s_eq[k] : 0;
        for(j = k; j < i; j++) s -= Ri[j]*xk[j];
        xk[i] = s/Ri[i];
    }
    for(i = neq-1; i >= 0; i--) eq_reflect(eq->V+(size_t)i*n,eq->tau[i],i,n,xk);
    if(eq->metric) for(i = 0; i < n; i++) xk[i] *= eq->dsq[i];
    // xk <-- xk - W W'H xk, hk = H xk
    eq_hess_times(qp,eq->metric,xk,hk);
    for(j = 0; j < nz; j++) t[j] = 0;
    for(i = 0; i < n; i++){
        const c_float* Wi = eq->W+(size_t)i*nz;
        const c_float hi = hk[i];
        if(hi == 0) continue;
        for(j = 0; j < nz; j++) t[j] += Wi[j]*hi;
    }
    for(i = 0; i < n; i++) xk[i] -= eq_dot(eq->W+(size_t)i*nz,t,nz);
    eq_hess_times(qp,eq->metric,xk,hk);
    for(c = 0; c < mr; c++){
        const int id = eq->keep[c];
        dk[c] = (id < ms) ? -xk[id] : -eq_dot(qp->A+(size_t)(id-ms)*n,xk,n);
    }
    eq->cols[k] = col;
    return col;
}

// Response of xp to f (PATH_LDP): xf = -W W'f, gf = H xf + f, df = -A xf
static void eq_rhs_f(DAQPEqElim* eq, const DAQPProblem* qp){
    const int n = eq->n, nz = eq->nz, mr = eq->mr;
    c_float* t = eq->tmp+2*n;
    int i, j, c;
    if(eq->xf == NULL){
        eq->xf = malloc(n*sizeof(c_float));
        eq->gf = malloc(n*sizeof(c_float));
        eq->df = malloc(mr*sizeof(c_float));
        eq->sh = malloc(mr*sizeof(c_float));
    }
    for(j = 0; j < nz; j++) t[j] = 0;
    if(qp->f != NULL)
        for(i = 0; i < n; i++){
            const c_float* Wi = eq->W+(size_t)i*nz;
            const c_float fi = qp->f[i];
            if(fi == 0) continue;
            for(j = 0; j < nz; j++) t[j] += Wi[j]*fi;
        }
    for(i = 0; i < n; i++) eq->xf[i] = -eq_dot(eq->W+(size_t)i*nz,t,nz);
    eq_hess_times(qp,eq->metric,eq->xf,eq->gf);
    if(qp->f != NULL) for(i = 0; i < n; i++) eq->gf[i] += qp->f[i];
    // -a_c xf = a_c W W'f = Ar_c W'f (and the rows of W for the simple bounds)
    for(c = 0; c < mr; c++) eq->df[c] = eq_dot(eq->Ar+(size_t)c*nz,t,nz);
    eq->f_valid = 1;
}

/*
 * The part that depends on b and f: the particular solution xp, the linear
 * term of the reduced problem (PATH_QP/LP), and the shifted bounds. With
 * warm, the responses to b_E and f are used if few equalities have a nonzero
 * right-hand side (as in MPC, where only the initial state enters b_E).
 */
static int eq_reduce_rhs(DAQPWorkspace* work, DAQPProblem* qp, const int warm){
    DAQPEqElim* eq = work->eq;
    const int n = eq->n, ms = eq->ms, neq = eq->neq, nz = eq->nz, mr = eq->mr;
    const c_float primal_tol = work->settings->primal_tol;
    c_float *y = eq->tmp, *g = eq->tmp+n, *fz = eq->tmp+2*n;
    int i, j, k, c, nnz = 0;

    if(warm && eq->path == DAQP_EQ_PATH_LDP)
        for(k = 0; k < neq; k++) if(eq_value(work,qp,eq->eq_ids[k]) != 0) nnz++;

    if(warm && eq->path == DAQP_EQ_PATH_LDP && 4*nnz <= neq){
        if(!eq->f_valid) eq_rhs_f(eq,qp);
        for(i = 0; i < n; i++){ eq->xp[i] = eq->xf[i]; g[i] = eq->gf[i]; }
        for(c = 0; c < mr; c++) eq->sh[c] = eq->df[c];
        for(k = 0; k < neq; k++){
            const c_float b = eq_value(work,qp,eq->eq_ids[k]);
            const c_float *col;
            if(b == 0) continue;
            col = eq_rhs_column(eq,qp,k);
            for(i = 0; i < n; i++){ eq->xp[i] += b*col[i]; g[i] += b*col[n+i]; }
            for(c = 0; c < mr; c++) eq->sh[c] += b*col[2*n+c];
        }
        eq->fp = 0;
        for(i = 0; i < n; i++) eq->fp += 0.5*eq->xp[i]*(g[i] + (qp->f != NULL ? qp->f[i] : 0));
        for(c = 0; c < mr; c++){
            const int id = eq->keep[c];
            eq->bur[c] = (qp->bupper[id] >= DAQP_INF) ? DAQP_INF : qp->bupper[id]+eq->sh[c];
            eq->blr[c] = (qp->blower[id] <= -DAQP_INF) ? -DAQP_INF : qp->blower[id]+eq->sh[c];
        }
    }
    else{
        // xp = Q1 R^{-T} b_E (scaled by H^{-1/2} in the metric case)
        for(k = 0; k < neq; k++){
            const c_float* Rk = eq->R+DAQP_ARSUM(k);
            c_float s = eq->s_eq[k]*eq_value(work,qp,eq->eq_ids[k]);
            for(i = 0; i < k; i++) s -= Rk[i]*y[i];
            y[k] = s/Rk[k];
        }
        for(i = 0; i < n; i++) eq->xp[i] = (i < neq) ? y[i] : 0;
        for(k = neq-1; k >= 0; k--) eq_reflect(eq->V+(size_t)k*n,eq->tau[k],k,n,eq->xp);
        if(eq->metric) for(i = 0; i < n; i++) eq->xp[i] *= eq->dsq[i];

        // g = H xp + f, f(xp) = 0.5 xp'(g + f)
        eq_hess_times(qp,eq->metric,eq->xp,g);
        if(qp->f != NULL) for(i = 0; i < n; i++) g[i] += qp->f[i];
        eq->fp = 0;
        for(i = 0; i < n; i++) eq->fp += 0.5*eq->xp[i]*(g[i] + (qp->f != NULL ? qp->f[i] : 0));

        // W'g, the linear term of the reduced problem
        for(j = 0; j < nz; j++) fz[j] = 0;
        for(i = 0; i < n; i++){
            const c_float* Wi = eq->W+(size_t)i*nz;
            const c_float gi = g[i];
            if(gi == 0) continue;
            for(j = 0; j < nz; j++) fz[j] += Wi[j]*gi;
        }
        if(eq->path == DAQP_EQ_PATH_LDP){
            // Move xp to the minimizer over the equalities: xp - W W'g, which
            // lowers the objective by 0.5||W'g||^2 since W'HW = I
            for(i = 0; i < n; i++) eq->xp[i] -= eq_dot(eq->W+(size_t)i*nz,fz,nz);
            eq->fp -= 0.5*eq_dot(fz,fz,nz);
        }
        else for(j = 0; j < nz; j++) eq->fr[j] = fz[j];

        // Shift the bounds of the kept constraints by their value at xp
        for(c = 0; c < mr; c++){
            const int id = eq->keep[c];
            const c_float shift = (id < ms) ? eq->xp[id] :
                eq_dot(qp->A+(size_t)(id-ms)*n,eq->xp,n);
            eq->bur[c] = (qp->bupper[id] >= DAQP_INF) ? DAQP_INF : qp->bupper[id]-shift;
            eq->blr[c] = (qp->blower[id] <= -DAQP_INF) ? -DAQP_INF : qp->blower[id]-shift;
        }
    }
    // The constraints that were left out only have to be consistent
    for(c = 0; c < eq->ndrop; c++){
        const int id = eq->drop_ids[c];
        const c_float val = (id < ms) ? eq->xp[id] : eq_dot(qp->A+(size_t)(id-ms)*n,eq->xp,n);
        if(qp->bupper[id]-val < -primal_tol || qp->blower[id]-val > primal_tol){
            // An inconsistent dependent equality is reported the same way as
            // when it is detected while forming the working set
            return eq_is_candidate(work,qp,id) ? DAQP_EXIT_OVERDETERMINED_INITIAL
                : DAQP_EXIT_INFEASIBLE;
        }
    }
    return 1;
}

/* ---------------------------------------------------------------------------
 * Swapping the reduced problem into the workspace
 * -------------------------------------------------------------------------*/

#define DAQP_SWAP(T,a,b) do{ T swp_ = (a); (a) = (b); (b) = swp_; }while(0)

static void eq_swap_ldp(DAQPWorkspace* work, DAQPLDPData* d){
    DAQP_SWAP(DAQPProblem*,work->qp,d->qp);
    DAQP_SWAP(int,work->n,d->n);
    DAQP_SWAP(int,work->m,d->m);
    DAQP_SWAP(int,work->ms,d->ms);
    DAQP_SWAP(c_float*,work->M,d->M);
    DAQP_SWAP(c_float*,work->dupper,d->dupper);
    DAQP_SWAP(c_float*,work->dlower,d->dlower);
    DAQP_SWAP(c_float*,work->Rinv,d->Rinv);
    DAQP_SWAP(c_float*,work->RinvD,d->RinvD);
    DAQP_SWAP(c_float*,work->v,d->v);
    DAQP_SWAP(c_float*,work->scaling,d->scaling);
    DAQP_SWAP(c_float*,work->Mu,d->Mu);
    DAQP_SWAP(int*,work->sense,d->sense);
    DAQP_SWAP(c_float*,work->rho_ls,d->rho_ls);
    DAQP_SWAP(c_float*,work->rho_us,d->rho_us);
    DAQP_SWAP(c_float*,work->w_ls,d->w_ls);
    DAQP_SWAP(c_float*,work->w_us,d->w_us);
    DAQP_SWAP(int,work->state,d->state);
    DAQP_SWAP(int,work->n_prox,d->n_prox);
    if(work->bnb != NULL){
        DAQP_SWAP(int*,work->bnb->bin_ids,d->bin_ids);
        DAQP_SWAP(int,work->bnb->nb,d->nb);
    }
}

int daqp_eq_install(DAQPWorkspace* work){
    DAQPEqElim* eq = work->eq;
    DAQPLDPData* d;
    int c;
    if(eq == NULL || !eq->active || eq->installed) return 0;
    d = &eq->other;
    /*
     * The soft weights are indexed by the original problem. The reduced
     * constraints are the original ones in their original units (only
     * shifted), so the weights carry over as they are.
     */
    d->rho_ls = d->rho_us = d->w_ls = d->w_us = NULL;
    if(work->rho_ls != NULL){
        const int mr = eq->mr;
        if(eq->rho_r == NULL) eq->rho_r = malloc(4*(size_t)mr*sizeof(c_float));
        d->rho_ls = eq->rho_r;
        d->rho_us = eq->rho_r+mr;
        d->w_ls = eq->rho_r+2*mr;
        d->w_us = eq->rho_r+3*mr;
        for(c = 0; c < mr; c++){
            const int id = eq->keep[c];
            d->rho_ls[c] = work->rho_ls[id];
            d->rho_us[c] = work->rho_us[id];
            d->w_ls[c] = work->w_ls[id];
            d->w_us[c] = work->w_us[id];
        }
    }
    eq_swap_ldp(work,d);
    eq->installed = 1;
    return 1;
}

void daqp_eq_restore(DAQPWorkspace* work){
    DAQPEqElim* eq = work->eq;
    if(eq == NULL || !eq->installed) return;
    eq_swap_ldp(work,&eq->other);
    eq->installed = 0;
}

void daqp_eq_deactivate(DAQPWorkspace* work){
    if(work->eq == NULL) return;
    free_daqp_eq(work);
    // The working set refers to the reduced problem
    reset_daqp_workspace(work);
    if(work->bnb != NULL) work->bnb->n_root_WS = 0;
}

/* ---------------------------------------------------------------------------
 * Updating
 * -------------------------------------------------------------------------*/

int daqp_eq_update(DAQPWorkspace* work, DAQPProblem* qp, int mask,
        int (*update_ldp)(int, DAQPWorkspace*, DAQPProblem*)){
    DAQPEqElim* eq;
    int flag, rebuild, mask_r, c, nb;
    const int keep_mask = mask & (DAQP_UPDATE_unconstrained+DAQP_UPDATE_eliminate);

    daqp_eq_restore(work);
    if(work->eq == NULL) work->eq = calloc(1,sizeof(DAQPEqElim));
    eq = work->eq;

    rebuild = !eq->active || (mask&(DAQP_UPDATE_Rinv+DAQP_UPDATE_M)) ||
        eq->n != qp->n || eq->m != qp->m || eq->ms != qp->ms ||
        !eq_same_candidates(work,qp);

    if(rebuild){
        flag = eq_build_reduction(work,qp);
        if(flag <= 0){
            daqp_eq_deactivate(work);
            return (flag < 0) ? flag : DAQP_EQ_NOT_REDUCED;
        }
        eq->active = 1;
        // (Rinv also for an LP, for which it marks the proximal directions)
        mask_r = DAQP_UPDATE_Rinv+DAQP_UPDATE_M+DAQP_UPDATE_d+DAQP_UPDATE_sense;
        if(eq->qp.f != NULL) mask_r |= DAQP_UPDATE_v;
    }
    else{
        mask_r = DAQP_UPDATE_d;
        if(eq->qp.f != NULL) mask_r |= DAQP_UPDATE_v; // Depends on xp
        if(mask&DAQP_UPDATE_sense){
            for(c = 0; c < eq->mr; c++) eq->sr[c] = work->sense[eq->keep[c]];
            mask_r |= DAQP_UPDATE_sense;
        }
    }
    mask_r |= keep_mask;

    if(mask&DAQP_UPDATE_v) eq->f_valid = 0; // f has changed
    flag = eq_reduce_rhs(work,qp,!rebuild);
    if(flag < 0){
        eq->other.state |= mask_r & DAQP_STATE_PENDING; // Formed by the next update
        eq->error = flag; // Reported by a solve before that
        return flag;
    }
    eq->error = 0;

    daqp_eq_install(work);
    flag = update_ldp(mask_r,work,&eq->qp);
    // A singular reduced Hessian needs v for the proximal linear term (as in
    // setup_daqp_ldp)
    if(flag >= 0 && work->n_prox > 0 && work->v == NULL && work->qp->H != NULL)
        work->v = calloc(work->n,sizeof(c_float));
    if(work->bnb != NULL){
        for(c = 0, nb = 0; c < work->m; c++)
            if(work->sense[c] & DAQP_BINARY) work->bnb->bin_ids[nb++] = c;
        work->bnb->nb = nb;
    }
    daqp_eq_restore(work);
    return flag;
}

/* ---------------------------------------------------------------------------
 * Retrieving the solution of the original problem
 * -------------------------------------------------------------------------*/

/*
 * Multipliers of the eliminated equalities, from the stationarity condition
 *   A_E' lam_E = -(H x + f + A_K' lam_K) =: -g,
 * with A_E' S = Q1 R (S the normalization; H^{1/2} Q1 R in the metric case):
 *   lam_E = -S R^{-1} Q1' g   (Q1' H^{-1/2} g in the metric case).
 */
static void eq_compute_lam_eq(DAQPWorkspace* work, const DAQPProblem* qp,
        const c_float* x, c_float* lam){
    DAQPEqElim* eq = work->eq;
    const int n = eq->n, ms = eq->ms, neq = eq->neq;
    c_float* g = eq->tmp;
    int i, k, c;
    eq_hess_times(qp,eq->metric,x,g);
    if(qp->f != NULL) for(i = 0; i < n; i++) g[i] += qp->f[i];
    for(c = 0; c < eq->mr; c++){
        const int id = eq->keep[c];
        const c_float l = lam[id];
        if(l == 0) continue;
        if(id < ms) g[id] += l;
        else{
            const c_float* a = qp->A+(size_t)(id-ms)*n;
            for(i = 0; i < n; i++) g[i] += l*a[i];
        }
    }
    if(eq->metric) for(i = 0; i < n; i++) g[i] *= eq->dsq[i];
    for(k = 0; k < neq; k++) eq_reflect(eq->V+(size_t)k*n,eq->tau[k],k,n,g);
    for(k = neq-1; k >= 0; k--){ // R mu = -Q1'g
        const c_float* Rk = eq->R+DAQP_ARSUM(k);
        const c_float mu = -g[k]/Rk[k];
        g[k] = mu;
        for(i = 0; i < k; i++) g[i] += Rk[i]*mu;
    }
    for(k = 0; k < neq; k++) lam[eq->eq_ids[k]] = eq->s_eq[k]*g[k];
}

void daqp_eq_expand(DAQPResult* res, DAQPWorkspace* work){
    DAQPEqElim* eq = work->eq;
    DAQPProblem* qp;
    const int state_mask = DAQP_ACTIVE | DAQP_LOWER | DAQP_SLACK_FIXED;
    c_float* x;
    c_float fval_r;
    int i, c;
    if(eq == NULL || !eq->installed) return;
    qp = eq->other.qp; // The original problem
    x = eq->tmp+2*(size_t)eq->n; // Not used by eq_compute_lam_eq

    // x = xp + W w
    for(i = 0; i < eq->n; i++) x[i] = eq->xp[i]+eq_dot(eq->W+(size_t)i*eq->nz,work->x,eq->nz);
    if(res->x != NULL) for(i = 0; i < eq->n; i++) res->x[i] = x[i];

    // Objective function value (the reduced problem of PATH_LDP has no linear
    // term, in which case daqp_extract_result leaves fval to be formed here)
    fval_r = res->fval;
    if(eq->path == DAQP_EQ_PATH_LDP && work->v == NULL) fval_r = 0.5*work->fval;
    res->fval = fval_r + eq->fp;
    if(work->avi != NULL && !work->avi->is_symmetric && qp->f != NULL)
        res->fval = eq_dot(qp->f,x,eq->n); // As daqp_extract_result for an AVI

    // Multipliers of the original constraints (keep is ascending, so they can
    // be scattered in place from the end)
    if(res->lam != NULL && !DAQP_IS_HIERARCHICAL(work)){
        for(i = eq->m-1; i >= eq->mr; i--) res->lam[i] = 0;
        for(c = eq->mr-1; c >= 0; c--){
            const c_float l = res->lam[c];
            res->lam[c] = 0;
            res->lam[eq->keep[c]] = l;
        }
        eq_compute_lam_eq(work,qp,x,res->lam);
    }

    // Carry the working set over to the original constraints
    for(c = 0; c < eq->mr; c++){
        const int id = eq->keep[c];
        eq->other.sense[id] = (eq->other.sense[id] & ~state_mask) | (work->sense[c] & state_mask);
    }
}

void daqp_eq_set_primal_start(DAQPWorkspace* work, const c_float* x){
    DAQPEqElim* eq = work->eq;
    const int n = eq->n, nz = eq->nz;
    const DAQPProblem* qp = eq->installed ? eq->other.qp : work->qp;
    c_float *t = eq->tmp, *ht = eq->tmp+n;
    int i, j;
    // w = W'H(x-xp) if W'HW = I, and w = W'(x-xp) if W is orthonormal
    for(i = 0; i < n; i++) t[i] = x[i]-eq->xp[i];
    if(eq->path == DAQP_EQ_PATH_LDP) eq_hess_times(qp,eq->metric,t,ht);
    else for(i = 0; i < n; i++) ht[i] = t[i];
    for(j = 0; j < nz; j++) work->x[j] = 0;
    for(i = 0; i < n; i++){
        const c_float* Wi = eq->W+(size_t)i*nz;
        const c_float hi = ht[i];
        if(hi == 0) continue;
        for(j = 0; j < nz; j++) work->x[j] += Wi[j]*hi;
    }
}

void free_daqp_eq(DAQPWorkspace* work){
    DAQPEqElim* eq = work->eq;
    if(eq == NULL) return;
    daqp_eq_restore(work);
    eq_free_reduction(eq);
    eq_free_reduced_ldp(eq);
    free(eq);
    work->eq = NULL;
}
