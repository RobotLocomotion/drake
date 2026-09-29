#include "daqp_prox.h"
#include "auxiliary.h"
#include "utils.h"
#include <math.h>

static int prox_step(DAQPWorkspace* work, c_float* s_prev);
static void prox_rescale(DAQPWorkspace* work, c_float eps, c_float eps_new);
static int prox_is_infeasible(const DAQPWorkspace* work);

/* --------------------------------------------------------------------------
 * daqp_prox  --  outer proximal-point / semi-proximal loop
 *
 * QP problems (Rinv or RinvD is set):
 *   Diagonal Hessians use a semi-proximal method, perturbing only singular
 *   coordinate directions. Dense Hessians use a full proximal shift when
 *   Cholesky detects numerical singularity; selectively shifting failed
 *   pivots is not a reliable nullspace regularization. If H is already
 *   positive definite (n_prox == 0), the inner QP equals the original and
 *   we exit after one solve.
 *
 * LP problems (Rinv == NULL && RinvD == NULL):
 *   Classical regularisation-based smoothing with adaptive eps.
 * --------------------------------------------------------------------------*/
int daqp_prox(DAQPWorkspace *work){
    int i, total_iter = 0;
    c_float s_prev = -1; // Exact step length of the latest prox_step (-1: none)
    int center_relaxed = 0;
    int nx, is_lp;
    const c_float relaxation = 1.5;
    int exitflag = DAQP_EXIT_ITERLIMIT; // If no iteration can be taken
    c_float *swp_ptr;
    c_float max_diff, tol_stat;
    c_float eta = work->settings->eta_prox;
    c_float eps;

    // nh act as a counter for outer iterations
    work->nh = 0;

    nx = work->n;
    is_lp = (work->Rinv == NULL && work->RinvD == NULL);
    eps = is_lp ? 1.0 : daqp_get_proximal_regularization(work);

    // For a QP whose Hessian is already positive definite (n_prox == 0),
    // no direction needs a proximal shift.  The inner QP equals the
    // original problem, so one solve gives the exact solution.
    const int all_pd = (!is_lp) && (work->n_prox == 0);

    // eps of semi-proximal directions can be changed cheaply (prox_rescale).
    // A failed inner problem (often due to a small eps) is resolved with eps_max
    const int adaptive = !is_lp && !all_pd && work->avi == NULL &&
        work->prox_mask != NULL && work->n_prox < nx;
    c_float eps_max = eps;
    int rescaled = 0;
    if(adaptive){
        c_float hmax = 0;
        for(i = 0; i < nx; i++){
            c_float hii = work->qp->H[(size_t)i*nx+i];
            if(hii < 0) hii = -hii;
            if(hii > hmax) hmax = hii;
        }
        // Reset an eps that an earlier solve has raised (as in daqp_update_Rinv)
        c_float eps0 = work->settings->eps_prox < 0 ? -work->settings->eps_prox : work->settings->eps_prox;
        if(eps0 < sqrt(work->settings->zero_tol)*hmax) eps0 = sqrt(work->settings->zero_tol)*hmax;
        if(eps > 1.01*eps0){
            prox_rescale(work,eps,eps0);
            eps = eps0;
            rescaled = 1;
        }
        eps_max = DAQP_PROX_EPS_MAX*hmax > eps ? DAQP_PROX_EPS_MAX*hmax : eps;
    }

    // A negative eta selects an automatic tolerance. Preserve the established
    // default, but tighten it when the user requests a non-default dual
    // tolerance. Skip this entirely for positive-definite QPs, where no
    // proximal convergence test is needed.
    if(!all_pd && eta < 0.0){
        eta = DAQP_AUTO_ETA_CAP;
        if(work->settings->dual_tol != DAQP_DEFAULT_DUAL_TOL &&
           0.1 * work->settings->dual_tol < eta)
            eta = 0.1 * work->settings->dual_tol;
    }

    while(total_iter < work->settings->iter_limit){
        /* ----------------------------------------------------------------
         * Perturb the problem: form v = R'\(f - eps_mask * x_old)
         * ----------------------------------------------------------------*/
        if(is_lp){
            // No Hessian factor.  Adapt eps heuristically: grow when the
            // inner LP stalls (iterations==1), shrink otherwise to improve
            // accuracy. Keep the deterministic initial value for the first
            // solve; work->iterations has no current-loop value yet.
            if(total_iter > 0)
                eps *= (work->iterations == 1) ? 10.0 : 0.9;
            if(eps > 1e3) eps = 1e3;
            for(i = 0; i < nx; i++)
                work->v[i] = work->qp->f[i]*eps - work->x[i];
        }
        else{
            if(work->prox_mask == NULL || work->n_prox == nx){
                // Dense singular Hessians use a full shift. Avoid a mask
                // load and branch for every component on every outer step.
                if(work->qp->f != NULL)
                    for(i = 0; i < nx; i++)
                        work->v[i] = work->qp->f[i] - eps * work->x[i];
                else
                    for(i = 0; i < nx; i++)
                        work->v[i] = -eps * work->x[i];
            }
            else{
                // Diagonal Hessians can regularize only singular directions.
                if(work->qp->f != NULL)
                    for(i = 0; i < nx; i++)
                        work->v[i] = work->qp->f[i]
                                     - (work->prox_mask[i] ? eps : 0.0) * work->x[i];
                else
                    for(i = 0; i < nx; i++)
                        work->v[i] = -(work->prox_mask[i] ? eps : 0.0)
                                     * work->x[i];
            }
            daqp_update_v(work->v, work);
        }

        daqp_update_d(work, work->qp->bupper, work->qp->blower);
        if(rescaled){ // The working set is factored anew after a rescaling
            reset_daqp_workspace(work);
            daqp_activate_constraints(work);
            rescaled = 0;
        }

        // xold <-- x  (pointer swap avoids copying)
        swp_ptr = work->xold; work->xold = work->x; work->x = swp_ptr;

        /* ----------------------------------------------------------------
         * Solve the (regularised) least-distance problem
         * ----------------------------------------------------------------*/
        work->u = work->x;
        work->nh++;
        exitflag = daqp_ldp(work);

        total_iter += work->iterations;
        if(adaptive && eps < eps_max && total_iter < work->settings->iter_limit &&
                (exitflag == DAQP_EXIT_CYCLE ||
                 (exitflag == DAQP_EXIT_INFEASIBLE && !prox_is_infeasible(work)))){
            prox_rescale(work,eps,eps_max);
            eps = eps_max;
            rescaled = 1;
            s_prev = -1;
            for(i = 0; i < nx; i++) work->x[i] = work->xold[i]; // The center
            continue;
        }
        if(exitflag < 0)
            break;              // Inner solver failed -- propagate error
        ldp2qp_solution(work); // Recover QP primal from LDP dual

        if(eps == 0) break;     // No regularisation -> single outer step

        /* ----------------------------------------------------------------
         * If H is fully positive definite, the inner QP is the original
         * problem.  The first solve gives the exact solution.
         * ----------------------------------------------------------------*/
        if(all_pd){
            exitflag = DAQP_EXIT_OPTIMAL;
            break;
        }

        /* ----------------------------------------------------------------
         * Convergence check: fixed point  ||x - x_old||_inf < tol_stat.
         *
         * A fixed point is a valid stationarity certificate regardless of
         * how many active-set changes the inner solve needed.  Checking it
         * after every successful solve avoids extra outer iterations and,
         * unlike objective stagnation, cannot label a non-stationary point
         * as optimal.
         * ----------------------------------------------------------------*/
        tol_stat = is_lp ? eta*eps : eta/eps;
        for(i = 0; i < nx; i++){
            max_diff = work->x[i] - work->xold[i];
            if(max_diff > tol_stat || max_diff < -tol_stat) break;
        }
        if(i == nx){
            if(center_relaxed &&
                    total_iter < work->settings->iter_limit){
                center_relaxed = 0;
                continue; // Confirm convergence from the feasible iterate.
            }
            exitflag = DAQP_EXIT_OPTIMAL;
            break;
        }

        // Unchanged working set => accelerate by moving the center along the
        // step (relax the step for an AVI). Convergence is confirmed afterwards
        center_relaxed = 0;
        if(work->iterations != 1) s_prev = -1; // The working set has changed
        if(work->iterations == 1 && work->n_active < nx &&
                total_iter < work->settings->iter_limit){
            if(work->avi != NULL){
                for(i = 0; i < nx; i++)
                    work->x[i] = work->xold[i]
                        + relaxation*(work->x[i] - work->xold[i]);
                center_relaxed = 1;
            }
            else{
                const int step_flag = prox_step(work,&s_prev);
                if(step_flag == DAQP_EXIT_UNBOUNDED){
                    exitflag = DAQP_EXIT_UNBOUNDED;
                    break;
                }
                center_relaxed = step_flag;
            }
        }
    }

    // Finalize
    if(total_iter >= work->settings->iter_limit) exitflag = DAQP_EXIT_ITERLIMIT;
    // Refine x (skipped for short solves)
    if(exitflag > 0 && total_iter > DAQP_REFINE_MIN_ITER) daqp_refine_primal(work);
    if(is_lp){
        for(i = 0; i < work->n_active; i++)
            work->lam_star[i] /= eps; // Rescale dual variables
    }
    else{
        /*
         * daqp_extract_result forms 0.5*(fval - ||v||^2). Correct the
         * regularized objective here while the reconstructed eps is local,
         * avoiding a second reconstruction during result extraction.
         */
        c_float prox_norm = 0.0;
        for(i = 0; i < nx; i++){
            if(work->prox_mask == NULL || work->prox_mask[i])
                prox_norm += work->x[i]*work->x[i];
        }
        work->fval += eps*prox_norm;
    }
    work->iterations = total_iter;
    return exitflag;
}

// Step length -g'd/d'Hd that minimizes the objective along d = x-x_old (d in
// xldl). Returns DAQP_INF if d has no curvature, -1 if d is not a descent direction
static c_float prox_curvature_step(DAQPWorkspace* work){
    int i, j;
    const int n = work->n;
    const DAQPProblem* qp = work->qp;
    // Scratch (reuse_ind is reset by the next daqp_update_d)
    c_float *d = work->xldl, *hd = work->zldl;
    c_float gd = 0, dhd = 0, dd = 0, hmax = 0;

    for(i = 0; i < n; i++) d[i] = work->x[i] - work->xold[i];
    if(qp->f != NULL) for(i = 0; i < n; i++) gd += qp->f[i]*d[i];
    if(qp->H != NULL){
        for(i = 0; i < n; i++){
            const c_float* Hi = qp->H+(size_t)i*n;
            const c_float hii = Hi[i] < 0 ? -Hi[i] : Hi[i];
            c_float sum = 0;
            for(j = 0; j < n; j++) sum += Hi[j]*d[j];
            hd[i] = sum;
            if(hii > hmax) hmax = hii;
        }
        for(i = 0; i < n; i++){
            gd += work->x[i]*hd[i];
            dhd += d[i]*hd[i];
            dd += d[i]*d[i];
        }
    }
    if(gd >= 0) return -1;
    return (dhd > work->settings->zero_tol*hmax*dd) ? -gd/dhd : DAQP_INF;
}

// First inactive constraint that blocks x + s*(x-x_old) for s < *s (*s is
// shortened to the blocking step, *lower marks a lower bound)
static int prox_blocking_constraint(DAQPWorkspace* work, c_float* s, int* lower){
    int i, j, ind = DAQP_EMPTY_IND;
    const int n = work->n, m = work->m, ms = work->ms;
    const DAQPProblem* qp = work->qp;
    c_float ad, ax, sb;
    for(i = 0; i < m; i++){
        if(work->sense[i] & (DAQP_ACTIVE + DAQP_IMMUTABLE + DAQP_SET_ASIDE)) continue;
        if(i < ms){ ax = work->x[i]; ad = ax - work->xold[i]; }
        else{
            const c_float* a = qp->A+(size_t)(i-ms)*n;
            for(j = 0, ad = 0, ax = 0; j < n; j++){
                ax += a[j]*work->x[j];
                ad += a[j]*(work->x[j] - work->xold[j]);
            }
        }
        if(ad > 0 && qp->bupper[i] < DAQP_INF) sb = (qp->bupper[i]-ax)/ad;
        else if(ad < 0 && qp->blower[i] > -DAQP_INF) sb = (qp->blower[i]-ax)/ad;
        else continue;
        if(sb < *s){
            *s = sb;
            *lower = ad < 0;
            ind = i;
        }
    }
    return ind;
}

/* --------------------------------------------------------------------------
 * prox_step  --  step along the latest proximal step d = x - x_old
 *
 * With an unchanged working set, the proximal iterates converge slowly along
 * d. x is therefore moved to the minimizer along d, or to the first blocking
 * constraint, which is added to the working set (dependent ones are set aside).
 *
 * Returns 1 if x was moved, 0 otherwise, and DAQP_EXIT_UNBOUNDED for an LP
 * with an unblocked descent direction.
 * --------------------------------------------------------------------------*/
static int prox_step(DAQPWorkspace* work, c_float* s_prev){
    int i, k, ind, lower = 0, moved = 0, skipped = 0, first = 1;
    c_float s;
    while((s = prox_curvature_step(work)) >= 0){
        const c_float* d = work->xldl;
        if(first){ // Lagged (Barzilai-Borwein) step length
            const c_float s_exact = s;
            if(s_exact < DAQP_INF && *s_prev >= 0)
                s = (*s_prev < 2*s_exact) ? *s_prev : 2*s_exact;
            *s_prev = (s_exact < DAQP_INF) ? s_exact : -1;
            first = 0;
        }
        ind = prox_blocking_constraint(work,&s,&lower);
        if(ind == DAQP_EMPTY_IND){
            if(s < DAQP_INF){ // The minimizer along d
                for(k = 0; k < work->n; k++) work->x[k] += s*d[k];
                moved = 1;
            }
            else if(work->qp->H == NULL && !moved && !skipped)
                return DAQP_EXIT_UNBOUNDED;
            break;
        }
        *s_prev = -1; // Blocked: the working set changes
        // Advance to the blocking constraint and activate it (no step back if
        // it is already violated)
        if(s >= 0) for(k = 0; k < work->n; k++) work->x[k] += s*d[k];
        moved = 1;
        if(lower) DAQP_SET_LOWER(ind);
        else DAQP_SET_UPPER(ind);
        daqp_add_constraint(work, ind, lower ? -1.0 : 1.0);
        if(work->sing_ind == DAQP_EMPTY_IND) break;
        // Linearly dependent on the active constraints: set it aside
        work->sense[daqp_drop_singular_last(work)] |= DAQP_SET_ASIDE;
        skipped = 1;
    }
    if(skipped)
        for(i = 0; i < work->m; i++) work->sense[i] &= ~DAQP_SET_ASIDE;
    return moved;
}

// Change eps of the semi-proximal directions to eps_new. The directions are
// decoupled, so only their columns of Rinv and M are scaled (the working set
// has to be refactored afterwards)
static void prox_rescale(DAQPWorkspace* work, c_float eps, c_float eps_new){
    int i, j, disp;
    const int n = work->n, ms = work->ms;
    const int* mask = work->prox_mask;
    const c_float* H = work->qp->H;
    // The scaling of column j of Rinv
#define DAQP_PROX_RATIO(j) sqrt((H[(size_t)(j)*n+(j)]+eps)/(H[(size_t)(j)*n+(j)]+eps_new))
    for(i = 0; i < n; i++){
        if(!mask[i]) continue;
        const c_float r = DAQP_PROX_RATIO(i);
        if(work->Rinv == NULL){
            work->RinvD[i] *= r;
            if(i < ms) work->scaling[i] /= r;
        }
        else if(i < ms && (work->state & DAQP_STATE_RINV_NORMALIZED))
            work->scaling[i] /= r;
        else
            work->Rinv[DAQP_R_OFFSET(i,n)+i] *= r;
    }
    for(i = ms, disp = 0; i < work->m; i++, disp += n){
        c_float* mi = work->M+disp;
        c_float norm2 = 1, sc;
        for(j = 0; j < n; j++){
            if(!mask[j] || mi[j] == 0) continue;
            const c_float r = DAQP_PROX_RATIO(j);
            norm2 += (r*r-1)*mi[j]*mi[j];
            mi[j] *= r;
        }
        if(norm2 == 1) continue;
        sc = 1/sqrt(norm2);
        for(j = 0; j < n; j++) mi[j] *= sc;
        work->scaling[i] *= sc;
    }
#undef DAQP_PROX_RATIO
}

// Whether the certificate of infeasibility from daqp_ldp (the dependency lam_star
// of a singular working set) is valid and implies a violation above primal_tol.
// In the constraints of the QP, q = S*lam_star gives sum q_i a_i = 0, and hence
// max violation >= (-sum q_i b_i - sum_wrong |q_i| (bu_i-bl_i))/sum |q_i|, where the
// sign of q_i is wrong for a violated b_i (only acceptable for two-sided constraints)
static int prox_is_infeasible(const DAQPWorkspace* work){
    int i, id, lower;
    c_float q, gap = 0, norm = 0, wrong = 0;
    if(work->sing_ind == DAQP_EMPTY_IND) return 0;
    for(i = 0; i < work->n_active; i++){
        id = work->WS[i];
        lower = DAQP_IS_LOWER(id);
        q = work->scaling != NULL ? work->lam_star[i]*work->scaling[id] : work->lam_star[i];
        gap -= q*(lower ? work->qp->blower[id] : work->qp->bupper[id]);
        if(!DAQP_IS_IMMUTABLE(id) && (lower ? q > 0 : q < 0)){
            if(work->qp->bupper[id] < DAQP_INF && work->qp->blower[id] > -DAQP_INF)
                gap -= (q < 0 ? -q : q)*(work->qp->bupper[id]-work->qp->blower[id]);
            else wrong += q < 0 ? -q : q;
        }
        norm += q < 0 ? -q : q;
    }
    return wrong <= 1e-8*norm && gap > work->settings->primal_tol*norm;
}
