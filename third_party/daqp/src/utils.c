#include "daqp.h"
#include "utils.h"
#include <math.h>
#include <float.h>
#include <stdio.h>

#ifndef DAQP_AVI_PIVOT_TRIGGER
#define DAQP_AVI_PIVOT_TRIGGER ((c_float)0.05)
#endif
#ifndef DAQP_AVI_RETRY_RHO_REDUCTION
#define DAQP_AVI_RETRY_RHO_REDUCTION ((c_float)16.0)
#endif

static c_float proximal_regularization_scaled(
        const DAQPWorkspace *work, c_float hessian_scale){
    c_float eps = work->settings->eps_prox;
    if(eps < 0.0) eps = -eps; // Negative eps_prox selects automatic mode.
    c_float floor = sqrt(work->settings->zero_tol)*hessian_scale;
    if(eps > 0.0 && eps < floor) eps = floor;
    return eps;
}

static void daqp_install_avi_rho(
        DAQPAVI* avi, const DAQPProblem* p, c_float rho){
    int i,j,disp;
    const int n = p->n;
    avi->rho = rho;
    for(i = 0, disp = 0; i < n; i++){
        for(j = 0; j < n; j++, disp++){
            avi->Hs_rho[disp] = avi->Hsym[disp];
            avi->H_rho[disp] = p->H[disp];
        }
        avi->Hs_rho[i*n+i] += rho;
        avi->H_rho[i*n+i] += rho;
    }
}

int daqp_retry_avi_with_reduced_rho(DAQPWorkspace* work){
    DAQPAVI* avi = work->avi;
    c_float retry_rho;
    int error_flag;

    if(avi == NULL || !avi->retry_rho_needed) return 0;
    avi->retry_rho_needed = 0; // At most one retry per setup
    retry_rho = avi->rho/DAQP_AVI_RETRY_RHO_REDUCTION;
    daqp_install_avi_rho(avi,work->qp,retry_rho);
    daqp_lu(avi->H_rho,avi->P_H2,work->n);
    error_flag = daqp_update_Rinv(work,avi->Hs_rho,0);
    if(error_flag < 0) return error_flag;
    daqp_update_v(work->qp->f,work);
    error_flag = daqp_update_M(work,work->qp->A);
    if(error_flag < 0) return error_flag;
    daqp_normalize_Rinv(work);
    daqp_update_d(work,work->qp->bupper,work->qp->blower);
    return 1;
}

/*
 * Form the LDP of qp, which is the problem that the solvers see (the reduced
 * problem of an equality elimination is passed here as it is).
 */
static int daqp_update_ldp_core(int mask, DAQPWorkspace *work, DAQPProblem* qp){
    int error_flag, i;
    int do_activate = 0;
    int unconstrained_flag = 0;

    // Also form what an earlier update left pending. Everything stays pending
    // until this update completes, so an update that fails is redone.
    mask |= work->state & DAQP_STATE_PENDING;
    work->state = (work->state & (DAQP_STATE_RINV_NORMALIZED|DAQP_STATE_ILL_CONDITIONED))
        | (mask & DAQP_STATE_PENDING);

    // Add qp to workspace
    work->qp = qp;

    // Update dimensions of problem
    work->n = qp->n;
    work->m = qp->m;
    work->ms = qp->ms;

    // Update constraint sense
    if(mask&DAQP_UPDATE_sense){
        if(work->qp->sense == NULL) // Assume all constraints are "normal" inequality constraints
            for(i=0;i<work->m;i++) work->sense[i] = 0;
        else{
            for(i=0;i<work->m;i++) work->sense[i] = qp->sense[i];
            do_activate = 1;
        }
    }

    // Check bounds early
    if(mask&DAQP_UPDATE_M||mask&DAQP_UPDATE_v||mask&DAQP_UPDATE_d||mask&DAQP_UPDATE_sense){
        error_flag = daqp_check_bounds(work,qp->bupper,qp->blower);
        if(error_flag<0) return error_flag;
        if(error_flag==1) do_activate = 1;
    }

    // Update Rinv
    if(mask&DAQP_UPDATE_Rinv){
        if(work->avi == NULL)
            error_flag = daqp_update_Rinv(work, qp->H, qp->problem_type==2 ? 1 : 0);
        else{
            daqp_update_avi(work->avi,qp,work->settings->zero_tol);
            if(work->avi->is_symmetric){
                error_flag = daqp_update_Rinv(work,qp->H,0);
            }
            else{
                // Early unconstrained check for AVI: skip Cholesky if x=-H^{-1}f is feasible
                unconstrained_flag = daqp_check_unconstrained(work,mask);
                if(unconstrained_flag == DAQP_UNCONSTRAINED_OPTIMAL) return 0;
                daqp_lu(work->avi->H_rho, work->avi->P_H2, work->n);
                error_flag = daqp_update_Rinv(work, work->avi->Hs_rho,0);
            }
        }
        if(error_flag<0)
            return error_flag;
    }

    // Update v (moved before M to enable early-exit check below)
    if(mask&DAQP_UPDATE_Rinv||mask&DAQP_UPDATE_v){
        daqp_update_v(qp->f,work);
    }

    if(work->avi == NULL || work->avi->is_symmetric)
        unconstrained_flag = daqp_check_unconstrained(work,mask);
    if(unconstrained_flag == DAQP_UNCONSTRAINED_OPTIMAL){
        // Rinv, v, and sense are formed, but not M and d, which depend on them
        work->state &= ~(DAQP_UPDATE_Rinv+DAQP_UPDATE_v+DAQP_UPDATE_sense);
        work->state |= DAQP_UPDATE_d;
        if(mask&DAQP_UPDATE_Rinv) work->state |= DAQP_UPDATE_M;
        return 0;
    }

    // Update M
    if(mask&DAQP_UPDATE_Rinv||mask&DAQP_UPDATE_M){
        error_flag = daqp_update_M(work,qp->A);
        if(error_flag<0) return error_flag;
        do_activate = 1; // daqp_update_M cleared the working set
    }

    daqp_normalize_Rinv(work);

    // Update d
    if(mask&DAQP_UPDATE_Rinv||mask&DAQP_UPDATE_M
            ||mask&DAQP_UPDATE_v||mask&DAQP_UPDATE_d){
        if(unconstrained_flag == 1){ // Already computed d, just need to normalize
            if(work->scaling != NULL){
                for(i = 0; i < work->m; i++){
                    work->dupper[i]*=work->scaling[i];
                    work->dlower[i]*=work->scaling[i];
                }
            }
            work->reuse_ind = 0; // d changed => cannot reuse intermediate results
        }
        else{// Normal update of d
            error_flag = daqp_update_d(work,qp->bupper,qp->blower);
        }
    }

    // Update hierarchy
    if(mask&DAQP_UPDATE_hierarchy){
        work->nh = qp->nh;
        work->break_points = (qp->nh > 1) ? qp->break_points : NULL;
    }

    // Hierarchies not allowed for prox + avi
    if(DAQP_IS_HIERARCHICAL(work) &&
            (work->n_prox > 0 ||
             (work->avi != NULL && !work->avi->is_symmetric)))
        return DAQP_EXIT_UNSUPPORTED;

    error_flag = 0;
    // An empty working set can be one that a reset left out
    if(do_activate || work->n_active == 0){
        reset_daqp_workspace(work);
        if(!DAQP_IS_HIERARCHICAL(work))
            error_flag = daqp_activate_constraints(work);
        else{// Activate the first level (since those constraints are hard)
            int m_tmp = work->m;
            work->m = work->break_points[0];
            error_flag = daqp_activate_constraints(work);
            work->m = m_tmp;
        }
    }
    if(error_flag < 0) return error_flag;
    work->state &= ~DAQP_STATE_PENDING; // Everything has been formed

    return 0;
}

/*
 * Update the workspace with (changes in) qp, as marked by mask. If the equality
 * constraints are to be eliminated (see eq_elim.h), the LDP is formed for the
 * reduced problem, and the workspace keeps describing qp otherwise.
 */
int daqp_update_ldp(int mask, DAQPWorkspace *work, DAQPProblem* qp){
    int i, flag;
    const int was_reduced = DAQP_IS_REDUCED(work);
    daqp_eq_restore(work);
    work->qp = qp;
    // The constraint states are indexed by qp, also while its equality
    // constraints are eliminated
    if(mask&DAQP_UPDATE_sense && qp->sense != work->sense){
        if(qp->sense == NULL) for(i = 0; i < qp->m; i++) work->sense[i] = 0;
        else for(i = 0; i < qp->m; i++) work->sense[i] = qp->sense[i];
    }
    if(daqp_eq_wanted(work,qp,mask)){
        flag = daqp_eq_update(work,qp,mask,daqp_update_ldp_core);
        if(flag != DAQP_EQ_NOT_REDUCED){
            work->n = qp->n;
            work->m = qp->m;
            work->ms = qp->ms;
            return flag;
        }
    }
    else daqp_eq_deactivate(work);
    // The LDP of qp was not formed while its equalities were eliminated
    if(was_reduced){
        mask |= DAQP_UPDATE_M+DAQP_UPDATE_d;
        if(qp->H != NULL) mask |= DAQP_UPDATE_Rinv;
        if(qp->f != NULL) mask |= DAQP_UPDATE_v;
    }
    return daqp_update_ldp_core(mask,work,qp);
}

int daqp_update_Rinv(DAQPWorkspace *work, c_float* H, int is_factored){
    int i, j, k, disp, disp2;
    const int n = work->n;
    c_float eps = work->settings->eps_prox;
    c_float zero_tol = work->settings->zero_tol;
    c_float factor_tol = zero_tol;
    c_float hessian_scale = 0.0;
    int regularize_all = 0;
    int regularization_tries = 0;

    const int force_prox = work->settings->eps_prox > 0.0 && !is_factored
        && (work->avi == NULL || work->avi->is_symmetric);

    if(force_prox) regularize_all = 1;


    // Reset the semi-proximal mask for this factorization
    if(work->prox_mask != NULL){
        for(i = 0; i < n; i++) work->prox_mask[i] = 0;
    }
    work->n_prox = 0;
    work->state &= ~(DAQP_STATE_RINV_NORMALIZED|DAQP_STATE_ILL_CONDITIONED);

    if(H == NULL){ // LP: all directions need proximal regularization
        if(work->qp != NULL && work->qp->f != NULL) work->n_prox = n;
        if(work->scaling != NULL)
            for(i = 0; i < work->ms; i++) work->scaling[i] = 1.0;
        return 1;
    }

    // Check if Diagonal
    int is_diagonal = 1;
    if(!is_factored){
        for(i = 0, disp = 1; i < n && is_diagonal; i++, disp += i+1){
            c_float abs_diag = H[i*n+i];
            if(abs_diag < 0) abs_diag = -abs_diag;
            if(abs_diag > hessian_scale) hessian_scale = abs_diag;
            for(j = 1; j < n-i; j++, disp++){
                if(H[disp] > zero_tol || H[disp] < -zero_tol){ is_diagonal = 0; break; }
            }
        }
    } else {
        for(i = 0, disp = 0; i < n; disp += n-i, i++){
            for(j = 1; j < n-i; j++){
                if(H[disp+j] > zero_tol || H[disp+j] < -zero_tol){
                    is_diagonal = 0; break;
                }
            }
            if(!is_diagonal) break;
        }
    }

    if(force_prox){
        if(!is_factored && !is_diagonal){
            hessian_scale = 0.0;
            for(i = 0; i < n; i++){
                c_float abs_diag = H[i*n+i];
                if(abs_diag < 0.0) abs_diag = -abs_diag;
                if(abs_diag > hessian_scale) hessian_scale = abs_diag;
            }
        }
        eps = proximal_regularization_scaled(work, hessian_scale);
        if(eps <= 0.0) return DAQP_EXIT_NONCONVEX;
        work->n_prox = n;
        if(work->prox_mask != NULL)
            for(i = 0; i < n; i++) work->prox_mask[i] = 1;
    }

    // Diagonal Case — for unfactored H read diagonals directly (no packing needed).
    if(is_diagonal){
        if(!is_factored){
            if(hessian_scale > 0)
                factor_tol = zero_tol * hessian_scale;
            eps = proximal_regularization_scaled(work, hessian_scale);
        }
        // Allow small-scale Hessians without tightening the legacy absolute
        // acceptance threshold for large-scale Hessians.
        const c_float acceptance_tol = factor_tol < zero_tol ? factor_tol : zero_tol;
        if(work->Rinv != NULL){ work->RinvD = work->Rinv; work->Rinv = NULL; }
        c_float dmin = DAQP_INF, dmax = 0;
        for(i = 0, disp = 0; i < n; i++){
            c_float Hi;
            if(is_factored){ Hi = H[disp]; disp += n-i; }
            else            { Hi = H[i*n+i]; }
            if(!is_factored){
                if(force_prox || Hi <= factor_tol){
                    if(!force_prox){
                        if(work->prox_mask != NULL) work->prox_mask[i] = 1;
                        work->n_prox++;
                    }
                    Hi += eps;
                }
                if(Hi <= acceptance_tol) return DAQP_EXIT_NONCONVEX;
                Hi = sqrt(Hi);
            } else {
                if(Hi <= zero_tol) return DAQP_EXIT_NONCONVEX;
            }
            work->RinvD[i] = 1/Hi;
            if(work->scaling != NULL && i < work->ms) work->scaling[i] = Hi;
            if(Hi < dmin) dmin = Hi;
            if(Hi > dmax) dmax = Hi;
        }
        if(dmax*dmax > DAQP_REFINE_COND*dmin*dmin) work->state |= DAQP_STATE_ILL_CONDITIONED;
        return 1;
    }

    // Not diagonal: ensure Rinv points to allocated data, then pack
    // (symmetrize) H into Rinv before Cholesky.
    if(work->RinvD != NULL){ work->Rinv = work->RinvD; work->RinvD = NULL; }
    if(!is_factored && !regularize_all && work->prox_mask != NULL && work->avi == NULL){
        // Zero rows of H are decoupled => regularize only them (semi-proximal)
        hessian_scale = 0.0;
        for(i = 0; i < n; i++){
            c_float abs_diag = H[i*n+i];
            if(abs_diag < 0.0) abs_diag = -abs_diag;
            if(abs_diag > hessian_scale) hessian_scale = abs_diag;
        }
        for(i = 0; i < n; i++){
            if(H[i*n+i] != 0) continue;
            for(j = 0; j < n && H[i*n+j] == 0 && H[j*n+i] == 0; j++);
            if(j < n) continue;
            work->prox_mask[i] = 1;
            work->n_prox++;
        }
        if(work->n_prox > 0){
            eps = proximal_regularization_scaled(work, hessian_scale);
            if(eps <= 0.0) return DAQP_EXIT_NONCONVEX;
        }
    }
    if(!is_factored){
pack_hessian:
        for(i = 0, disp = 0; i < n; i++){
            work->Rinv[disp++] = H[i*n+i] + ((regularize_all ||
                        (work->n_prox > 0 && work->prox_mask[i])) ? eps : 0.0);
            for(j = i+1; j < n; j++)
                work->Rinv[disp++] = (c_float)0.5*(H[i*n+j] + H[j*n+i]);
        }
    }

    // Cholesky.
    if(is_factored){
        for(i=0, disp=0; i<n; i++){
            if(H[disp] <= zero_tol) return DAQP_EXIT_NONCONVEX;
            work->Rinv[disp] = 1/H[disp]; // Store 1/rii
            for(j=1, disp++; j<n-i; j++, disp++)
                work->Rinv[disp] = H[disp];
        }
    } else {
        c_float min_pivot = DAQP_INF;
        c_float max_pivot = 0.0;
        for(i = 0, disp = 0; i < n; disp += n-i, i++){
            c_float diag_i = work->Rinv[disp];  // read before overwrite
            for(k = 0, disp2 = i; k < i; k++, disp2 += n-k)
                diag_i -= work->Rinv[disp2] * work->Rinv[disp2];
            if(diag_i <= zero_tol)
                goto regularize_hessian;
            // (Skip regularized pivots)
            if(diag_i < min_pivot && (regularize_all || work->n_prox == 0 ||
                        !work->prox_mask[i]))
                min_pivot = diag_i;
            if(diag_i > max_pivot) max_pivot = diag_i;
            diag_i = 1/sqrt(diag_i);
            for(j = 1; j < n-i; j++){
                for(k = 0, disp2 = i; k < i; k++, disp2 += n-k)
                    work->Rinv[disp+j] -= work->Rinv[disp2] * work->Rinv[disp2+j];
                work->Rinv[disp+j] *= diag_i;
            }
            work->Rinv[disp] = diag_i;
        }
         // A successful unregularized Cholesky factorization represents a
         // positive-definite Hessian down to zero_tol relative pivots.
         // Once a singular Hessian has been shifted, be more conservative 
        if(min_pivot <= ((regularize_all || work->n_prox > 0) && !force_prox ?
                    sqrt(zero_tol) : zero_tol)*max_pivot){
regularize_hessian:
            if(regularize_all){
                if(eps <= 0 || regularization_tries++ >= 16) return DAQP_EXIT_NONCONVEX;
                eps *= 2.0;
            }
            else{
                hessian_scale = 0.0;
                for(k = 0; k < n; k++){
                    c_float abs_diag = H[k*n+k];
                    if(abs_diag < 0) abs_diag = -abs_diag;
                    if(abs_diag > hessian_scale) hessian_scale = abs_diag;
                }
                eps = proximal_regularization_scaled(work, hessian_scale);
                if(eps <= 0) return DAQP_EXIT_NONCONVEX;
                regularize_all = 1;
                work->n_prox = n;
                if(work->prox_mask != NULL)
                    for(k = 0; k < n; k++) work->prox_mask[k] = 1;
            }
            goto pack_hessian;
        }
    }
    // R -> Rinv
    for(k=0, disp=0; k<n; k++){
        disp2 = disp;
        work->Rinv[disp] = work->Rinv[disp2++];
        for(j=k+1; j<n; j++) work->Rinv[disp2++] *= -work->Rinv[disp];
        for(i=k+1, disp++; i<n; i++, disp++){
            work->Rinv[disp] *= work->Rinv[disp2++];
            for(j=1; j<n-i; j++)
                work->Rinv[disp+j] -= work->Rinv[disp2++] * work->Rinv[disp];
        }
    }
    // cond(H) >= max (H^-1)_ii * max H_ii (H_ii >= R_ii^2 if H is factored)
    c_float hinv_max = 0, hmax = 0;
    for(i = 0, disp = 0; i < n; i++){
        const c_float hii = is_factored ? 1/(work->Rinv[disp]*work->Rinv[disp]) : H[i*n+i];
        c_float s2 = 0;
        for(j = i; j < n; j++, disp++) s2 += work->Rinv[disp]*work->Rinv[disp];
        if(s2 > hinv_max) hinv_max = s2;
        if(hii > hmax) hmax = hii;
    }
    // Regularize an ill-conditioned Hessian, or mark it for refinement
    const c_float eps_mach = sizeof(c_float) == sizeof(float) ? FLT_EPSILON : DBL_EPSILON;
    if(!is_factored && !regularize_all && work->n_prox == 0 && work->avi == NULL &&
            ((work->eq != NULL && work->eq->installed && hinv_max*hmax > DAQP_HESSIAN_COND_MAX) ||
             n*eps_mach*hinv_max*hmax > DAQP_HESSIAN_COND_EPS))
        goto regularize_hessian;
    if(hinv_max*hmax > DAQP_REFINE_COND) work->state |= DAQP_STATE_ILL_CONDITIONED;
    return 1;
}

c_float daqp_get_proximal_regularization(const DAQPWorkspace *work){
    int i;
    c_float eps, recovered, rinv, scale = 0.0;

    if(work->n_prox == 0 || work->qp == NULL || work->qp->H == NULL)
        return 0.0;

    eps = work->settings->eps_prox;
    if(eps < 0.0) eps = -eps;
    if(work->RinvD != NULL && work->prox_mask != NULL && work->n_prox < work->n){
        for(i = 0; i < work->n && !work->prox_mask[i]; i++);
        return 1/(work->RinvD[i]*work->RinvD[i]) - work->qp->H[i*work->n+i];
    }
    if(work->RinvD != NULL){
        // Diagonal regularization has no retry loop, so reproduce its
        // scale-based floor directly. Avoid subtracting nearly equal large
        // diagonals from the factor, which would lose a small eps.
        if(work->qp->problem_type != 2){
            for(i = 0; i < work->n; i++){
                c_float abs_diag = work->qp->H[i*work->n+i];
                if(abs_diag < 0.0) abs_diag = -abs_diag;
                if(abs_diag > scale) scale = abs_diag;
            }
            eps = proximal_regularization_scaled(work, scale);
        }
        return eps;
    }

    // Semi-proximal: recover eps from a regularized row of Rinv (e_i/sqrt(eps))
    if(work->n_prox < work->n && work->prox_mask != NULL){
        for(i = 0; i < work->n && !work->prox_mask[i]; i++);
        rinv = (i < work->ms && (work->state & DAQP_STATE_RINV_NORMALIZED)) ?
            1/work->scaling[i] : work->Rinv[DAQP_R_OFFSET(i,work->n)+i];
        return 1/(rinv*rinv);
    }

    // Handle eps-shift correctly for simple bounds
    rinv = work->Rinv[0];
    if(work->ms > 0)
        rinv /= work->scaling[0];
    recovered = 1.0/(rinv*rinv) - work->qp->H[0];

    for(i = 0; i < work->n; i++){
        c_float abs_diag = work->qp->H[i*work->n+i];
        if(abs_diag < 0.0) abs_diag = -abs_diag;
        if(abs_diag > scale) scale = abs_diag;
    }
    eps = proximal_regularization_scaled(work, scale);
    if(eps <= 0.0) return 0.0;
    while(1.5*eps < recovered) eps *= 2.0;
    return eps;
}

// A blocked implementation of M <-- A*Rinv
static void daqp_rinv_product_block(const c_float* Rinv, const c_float** a,
        c_float** m, const int n){
    int i0, j;
    const c_float *a0 = a[0], *a1 = a[1], *a2 = a[2], *a3 = a[3];
    const c_float* r;
    for(i0 = n-2; i0 >= 0; i0 -= 2){
        // Row i0+1 of Rinv only reaches the second column of the tile
        r = Rinv+DAQP_R_OFFSET((i0+1),n)+i0;
        c_float s01 = a0[i0+1]*r[1], s11 = a1[i0+1]*r[1];
        c_float s21 = a2[i0+1]*r[1], s31 = a3[i0+1]*r[1];
        c_float s00 = 0, s10 = 0, s20 = 0, s30 = 0;
        for(j = i0; j >= 0; j--){
            r = Rinv+DAQP_R_OFFSET(j,n)+i0;
            const c_float r0 = r[0], r1 = r[1];
            const c_float x0 = a0[j], x1 = a1[j], x2 = a2[j], x3 = a3[j];
            s00 += x0*r0; s01 += x0*r1;
            s10 += x1*r0; s11 += x1*r1;
            s20 += x2*r0; s21 += x2*r1;
            s30 += x3*r0; s31 += x3*r1;
        }
        m[0][i0] = s00; m[0][i0+1] = s01;
        m[1][i0] = s10; m[1][i0+1] = s11;
        m[2][i0] = s20; m[2][i0+1] = s21;
        m[3][i0] = s30; m[3][i0+1] = s31;
    }
    if(n%2){ // The first column is left over, and only has a single term
        const c_float s0 = a0[0]*Rinv[0], s1 = a1[0]*Rinv[0];
        const c_float s2 = a2[0]*Rinv[0], s3 = a3[0]*Rinv[0];
        m[0][0] = s0; m[1][0] = s1; m[2][0] = s2; m[3][0] = s3;
    }
}

int daqp_update_M(DAQPWorkspace *work, c_float *A){
    int i,j,k,disp;
    const int n = work->n;
    const int mA = work->m-work->ms;
    // The rows of Rinv of the simple bounds are scaled if Rinv is normalized
    const int ns = (work->state & DAQP_STATE_RINV_NORMALIZED) ? work->ms : 0;
    if(work->Rinv != NULL){
        for(k = 0; k < mA; k += 4){
            const c_float* a[4];
            c_float* m[4];
            for(i = 0; i < 4; i++){ // The last row fills an incomplete block
                const int row = (k+i < mA) ? k+i : mA-1;
                a[i] = A+(size_t)row*n;
                m[i] = work->M+(size_t)row*n;
            }
            if(ns > 0){ // Undo the scaling in Rinv, in place in M
                for(i = 0; i < 4 && k+i < mA; i++){
                    for(j = 0; j < ns; j++) m[i][j] = a[i][j]/work->scaling[j];
                    for(; j < n; j++) m[i][j] = a[i][j];
                    a[i] = m[i];
                }
                for(; i < 4; i++) a[i] = a[i-1];
            }
            daqp_rinv_product_block(work->Rinv,a,m,n);
        }
    }
    else{
        if(work->RinvD == NULL){ // Copy A to M
            for(k = 0,disp=0;k<mA;k++){
                for(i=0;i<n;i++,disp++)
                    work->M[disp] = A[disp];
            }
        }
        else{
            for(k = 0,disp=0;k<mA;k++){
                for(i=0;i<n;i++,disp++)
                    work->M[disp] = A[disp]*work->RinvD[i];
            }
        }
    }

    reset_daqp_workspace(work); // Internal factorizations need to be redone!
    return daqp_normalize_M(work);
}

void daqp_update_v(c_float *f, DAQPWorkspace *work){
    int i,j,disp;
    const int n = work->n;
    if(work->v == NULL || f == NULL) return;
    if(work->Rinv == NULL){// Rinv = I => v = R'\v = f
        if(work->RinvD != NULL)
            for(i=0;i<n;++i) work->v[i] = f[i]*work->RinvD[i];
        else
            for(i=0;i<n;++i) work->v[i] = f[i];
        return;
    }
    int stop_id = (work->state & DAQP_STATE_RINV_NORMALIZED) ? work->ms : 0;
    for(j=n-1,disp=DAQP_ARSUM(n);j>=stop_id;j--){
        for(i=n-1;i>j;i--)
            work->v[i] +=work->Rinv[--disp]*f[j];
        work->v[j]=work->Rinv[--disp]*f[j];
    }
    for(;j>=0;j--){// Take into accoutn scaling in Rinv
        c_float col_scaling = f[j]/work->scaling[j];
        for(i=n-1;i>j;i--)
            work->v[i] +=work->Rinv[--disp]*col_scaling;
        work->v[j]=work->Rinv[--disp]*col_scaling;
    }
}

int daqp_update_d(DAQPWorkspace *work, c_float *bupper, c_float *blower){
    /* Compute d  = b+M*v */
    int i,j,disp;
    int do_activate = 0;
    c_float sum;

    const int n = work->n;
    work->reuse_ind = 0; // RHS of KKT system changed => cannot reuse intermediate results
    // Take into scaling of constraints
    if(work->scaling != NULL){
        for(i = 0;i<work->m;i++){
            work->dupper[i] = bupper[i]*work->scaling[i];
            work->dlower[i] = blower[i]*work->scaling[i];
        }
    }
    else{
        for(i = 0;i<work->m;i++){
            work->dupper[i] = bupper[i];
            work->dlower[i] = blower[i];
        }
    }

    if(work->v == NULL) return do_activate;
    // Simple bounds
    if(work->Rinv !=NULL){
        for(i = 0,disp=0;i<work->ms;i++){
            for(j=i, sum=0;j<n;j++)
                sum+=work->Rinv[disp++]*work->v[j];
            work->dupper[i]+=sum;
            work->dlower[i]+=sum;
        }
    }else{
        for(i = 0,disp=0;i<work->ms;i++){
            work->dupper[i]+=work->v[i];
            work->dlower[i]+=work->v[i];
        }
    }
    //General bounds
    for(i = work->ms, disp=0;i<work->m;i++){
        for(j=0, sum=0;j<n;j++)
            sum+=work->M[disp++]*work->v[j];
        work->dupper[i]+=sum;
        work->dlower[i]+=sum;
    }
    return do_activate;
}

int daqp_check_bounds(DAQPWorkspace* work, c_float* bupper, c_float* blower){
    int do_activate = 0;
#ifndef DAQP_ASSUME_VALID
    int i;
    c_float diff;
    for(i =0;i<work->m;i++){
        if(DAQP_IS_IMMUTABLE(i) && !(work->sense[i] & DAQP_AUTO_EQUALITY)) continue;
        diff = bupper[i] - blower[i];
        // Check for trivial infeasibility
        if ( diff < -work->settings->primal_tol ){
            return DAQP_EXIT_INFEASIBLE;
        }
        // Check for unmarked equality constraint (blower == bupper)
        else if (diff < work->settings->zero_tol && !DAQP_IS_SOFT(i)){
            if(!(work->sense[i] & DAQP_AUTO_EQUALITY) || !DAQP_IS_ACTIVE(i))
                do_activate = 1;
            work->sense[i] |= DAQP_ACTIVE | DAQP_IMMUTABLE | DAQP_AUTO_EQUALITY;
        }
        else if(work->sense[i] & DAQP_AUTO_EQUALITY){
            work->sense[i] &= ~(DAQP_ACTIVE | DAQP_IMMUTABLE | DAQP_AUTO_EQUALITY);
            do_activate = 1;
        }
    }
#endif
    return do_activate;
}

void daqp_normalize_Rinv(DAQPWorkspace* work){
    int i,j,disp;
    c_float scaling_i;
    if(work->state & DAQP_STATE_RINV_NORMALIZED) return;
    work->state |= DAQP_STATE_RINV_NORMALIZED;
    // Normalize simple constraints
    if(work->Rinv !=NULL){
        for(i=0, disp=0; i < work->ms;i++){
            scaling_i = 0;
            for(j=i; j < work->n; j++,disp++){
                scaling_i+=work->Rinv[disp]*work->Rinv[disp];
            }
            scaling_i = 1/sqrt(scaling_i);
            work->scaling[i] = scaling_i; // Need to save to correctly retrieve solution
            for(j=i,disp-=(work->n-i); j < work->n; j++,disp++)
                work->Rinv[disp]*= scaling_i;
        }
    }
}
int daqp_normalize_M(DAQPWorkspace* work){
    int i,j,disp;
    c_float scaling_i;
    c_float zero_tol = work->settings->zero_tol;
    // Normalize general constraints
    for(i=work->ms, disp=0;i<work->m;i++){
        scaling_i = 0;
        for(j=0;j<work->n;disp++,j++)
            scaling_i+=work->M[disp]*work->M[disp];
        if(scaling_i < zero_tol){
            // Keep downstream transformations well-defined for constraints
            // that are deliberately omitted from the normalized LDP.
            work->scaling[i] = 1.0;
#ifndef DAQP_ASSUME_VALID
            if(work->qp->bupper[i] < -zero_tol || work->qp->blower[i] > zero_tol)
                if((work->sense[i] & (DAQP_IMMUTABLE | DAQP_ACTIVE)) != DAQP_IMMUTABLE && !DAQP_IS_SOFT(i))
                    return DAQP_EXIT_INFEASIBLE;
#endif
            work->sense[i] = DAQP_IMMUTABLE; // ignore zero-row constraint
            continue; // TODO: mark infeasibility if dupper & dlower are nonzero
        }
        scaling_i = 1/sqrt(scaling_i);
        work->scaling[i]=scaling_i;
        for(j=0, disp-=work->n;j<work->n;j++,disp++)
            work->M[disp]*=scaling_i;
    }
    return 0;
}

// Returns 0 if it did not even compute the unconstrained solution
// Returns 1 if it computed the unconstrained solution, but it was not optimal
// Returns DAQP_UNCONSTRAINED_OPTIMAL if the unconstrained solution is optimal
int daqp_check_unconstrained(DAQPWorkspace* work, const int mask){
    int i;
    if ((mask&DAQP_UPDATE_unconstrained)==0) return 0;
    if ((mask&(DAQP_UPDATE_Rinv+DAQP_UPDATE_M+DAQP_UPDATE_v+DAQP_UPDATE_d)) == 0) return 0; // Nothing to update
    if (work->bnb != NULL || DAQP_IS_HIERARCHICAL(work) || work->n_prox >0) return 0; // Not a standard QP/AVI
    for(i = 0; i < work->m; i++) if(work->sense[i]&(DAQP_ACTIVE + DAQP_IMMUTABLE)) return 0; // No equalities

    // Check if unconstrained optimum is primal feasible.
    int j, disp;
    c_float sum;
    c_float* swp_ptr;
    int feasible = 1;
    const int n = work->n;
    const c_float primal_tol = work->settings->primal_tol;

    // Compute x_unc stored temporarily in work->x.
    swp_ptr = work->x; work->u = work->xold; work->x = work->xold; work->xold = swp_ptr;

    if(work->avi != NULL && !work->avi->is_symmetric){
        // AVI: unconstrained solution is x = -H^{-1} f
        if(work->qp->f != NULL)
            daqp_lu_solve(work->avi->LU_H, work->avi->P_H, work->qp->f, work->x, n);
        else
            for(i = 0; i < n; i++) work->x[i] = 0.0;
        for(i = 0; i < n; i++) work->x[i] = -work->x[i];
    }
    else if(work->v != NULL){
        if(work->Rinv != NULL){
            // Upper-triangular back-substitution: u[i] = sum_{j>=i} Rinv[i,j]*(-v[j])
            for(i = 0, disp = 0; i < n; i++){
                for(j = i, sum = 0; j < n; j++)
                    sum += work->Rinv[disp++] * work->v[j];
                work->x[i] = -sum;
            }
            if(work->state & DAQP_STATE_RINV_NORMALIZED)
                for(i = 0; i < work->ms; i++) work->x[i] /= work->scaling[i];
        } else if(work->RinvD != NULL){
            for(i = 0; i < n; i++) work->x[i] = -work->RinvD[i] * work->v[i];
        } else {
            for(i = 0; i < n; i++) work->x[i] = -work->v[i];
        }
    } else {
        // No linear term: unconstrained optimum is x = 0
        for(i = 0; i < n; i++) work->x[i] = 0.0;
    }
    //
    // Check simple bounds: blower[i] <= x_unc[i] <= bupper[i]
    for(i = 0; i < work->ms; i++){
        work->dupper[i] = work->qp->bupper[i] - work->x[i];
        work->dlower[i] = work->qp->blower[i] - work->x[i];
        if(work->dupper[i] < -primal_tol || work->dlower[i] > primal_tol)
            feasible = 0;
    }

    // Check general constraints: blower[i] <= A[i,:]*x_unc <= bupper[i]
    for(i = work->ms, disp = 0; i < work->m; i++){
        for(j = 0, sum = 0.0; j < n; j++)
            sum += work->qp->A[disp++] * work->x[j];
        work->dupper[i] = work->qp->bupper[i] - sum;
        work->dlower[i] = work->qp->blower[i] - sum;
        if(work->dupper[i] < -primal_tol || work->dlower[i] > primal_tol)
            feasible = 0;
    }
    if(feasible){
        reset_daqp_workspace(work);
        work->state |= DAQP_STATE_UNCONSTRAINED;
        return DAQP_UNCONSTRAINED_OPTIMAL;
    }
    // Switch back such that any warm starts a preserved
    swp_ptr = work->x; work->u = work->xold; work->x = work->xold; work->xold = swp_ptr;
    return 1;
}

int daqp_update_avi(DAQPAVI* avi, DAQPProblem* p, c_float zero_tol){
    const int n = p->n;
    // Setup matrices Hsym, Hs_rho, and H_rho, LU_H
    int i,j,disp;
    c_float val;
    c_float min_diag = DAQP_INF;
    c_float max_row_sum = 0.0;
    c_float fro_norm_sq = 0.0;
    c_float max_asymmetry = 0.0;
    avi->rho = 0.0;
    avi->retry_rho_needed = 0;
    for (i = 0, disp=0; i < n; i++) {
        c_float row_sum = 0.0;
        for (j = 0; j < n; j++, disp++) {
            if(j > i){
                c_float asymmetry = fabs(p->H[disp] - p->H[j * n + i]);
                if(asymmetry > max_asymmetry) max_asymmetry = asymmetry;
            }
            val = (p->H[disp] + p->H[j * n + i]) * 0.5;
            avi->Hsym[disp] = val;
            avi->Hs_rho[disp] = val;
            avi->H_rho[disp] = p->H[disp];
            avi->LU_H[disp] = p->H[disp];
            row_sum += (val < 0.0) ? -val : val;
            fro_norm_sq += p->H[disp] * p->H[disp];
            if(i == j && val < min_diag) min_diag = val;
        }
        if(row_sum > max_row_sum) max_row_sum = row_sum;
    }
    c_float hessian_scale = sqrt(fro_norm_sq);
    if(hessian_scale < 1.0) hessian_scale = 1.0;
    avi->is_symmetric = max_asymmetry <= zero_tol * hessian_scale;
    if(avi->is_symmetric) return 1;

    // Detect possible problematic rho from LU pivots
    int lu_status = daqp_lu(avi->LU_H, avi->P_H, n);
    if(lu_status == 0 && min_diag > 0.0){
        c_float min_lu_pivot = DAQP_INF;
        for(i = 0; i < n; i++){
            c_float pivot = fabs(avi->LU_H[i*n+i]);
            if(pivot < min_lu_pivot) min_lu_pivot = pivot;
        }
        if(min_lu_pivot < DAQP_AVI_PIVOT_TRIGGER * min_diag)
            avi->retry_rho_needed = 1;
    }

    // Start with default step length heuristic
    if(min_diag > 0.0 && max_row_sum > 0.0)
        avi->rho = sqrt(min_diag * max_row_sum);
    else
        avi->rho = sqrt(fro_norm_sq)/2;
    for(i = 0, disp = 0; i < n; i++, disp += n+1){
        avi->Hs_rho[disp] += avi->rho;
        avi->H_rho[disp] += avi->rho;
    }

    // H_rho factorization deferred until needed (skipped if unconstrained optimal)
    return 1;
}

int daqp_lu(c_float* A, int* P, int n) {
    c_float max_val,pA;
    for (int i = 0; i < n; i++) P[i] = i; // Initialize permutation vector
    for (int i = 0; i < n; i++) {
        // Pivot
        max_val = 0.0;
        int pivot = i;
        for (int j = i; j < n; j++) {
            pA = A[j*n+i];
            pA = (pA < 0) ? -pA : pA; // |pA|
                if (pA > max_val) {
                max_val = pA;
                pivot = j;
            }
        }

        // Check for singularity
        if (max_val < 1e-12) return -1;

        // Swap rows in A
        for (int k = 0; k < n; k++) {
            c_float temp = A[i * n + k];
            A[i * n + k] = A[pivot * n + k];
            A[pivot * n + k] = temp;
        }
        // Swap elements in permutation vector
        int tempP = P[i];
        P[i] = P[pivot];
        P[pivot] = tempP;

        // Elimination
        for (int j = i + 1; j < n; j++) {
            A[j * n + i] /= A[i * n + i]; // Store multiplier in L part
            for (int k = i + 1; k < n; k++) {
                A[j * n + k] -= A[j * n + i] * A[i * n + k];
            }
        }
    }
    return 0;
}

void daqp_lu_solve(c_float* LU, int* P, c_float* b, c_float* x, int n) {
    // Solve Ly = Pb
    for (int i = 0; i < n; i++) {
        x[i] = b[P[i]]; // Apply permutation to b
        for (int j = 0; j < i; j++) {
            x[i] -= LU[i * n + j] * x[j];
        }
    }
    // Solve Ux = y
    for (int i = n - 1; i >= 0; i--) {
        for (int j = i + 1; j < n; j++) {
            x[i] -= LU[i * n + j] * x[j];
        }
        x[i] /= LU[i * n + i];
    }
}

/* Remove Minrep */
void daqp_minrep_work(int* is_redundant, DAQPWorkspace* work){
    int i,j,exitflag;

    for(i=0; i < work->m; i++)
        is_redundant[i] = -1;

    for(i=0; i < work->m; i++){
        if(is_redundant[i] != -1 || DAQP_IS_IMMUTABLE(i)) continue;
        reset_daqp_workspace(work);
        work->sense[i] = 5;
        daqp_add_constraint(work,i,1.0);
        //work->dupper[i] += tol_weak; TODO support weaky infeasible constraints
        exitflag = daqp_ldp(work);
        if(exitflag== DAQP_EXIT_INFEASIBLE){
            is_redundant[i] = 1;
            work->sense[i] &=~DAQP_ACTIVE; // deactive (remains immutable -> ignored)
        }
        else{
            is_redundant[i] = 0;
            work->sense[i] &=~DAQP_IMMUTABLE;
            if(exitflag==DAQP_EXIT_OPTIMAL)
                for(j=0; j < work->n_active; j++) // all active constraint must also be nonredundant
                    is_redundant[work->WS[j]] = 0;
        }
        // work->dupper[i] -= tol_weak; // TODO support weakly infeasible constraints
        daqp_deactivate_constraints(work);
    }
}

/* Profiling */
#ifdef PROFILING
#ifdef _WIN32
void tic(DAQPtimer *timer){
    QueryPerformanceCounter(&(timer->start));
}
void toc(DAQPtimer *timer){
    QueryPerformanceCounter(&(timer->stop));
}
double get_time(DAQPtimer *timer){
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return (double)(timer->stop.QuadPart - timer->start.QuadPart)/f.QuadPart;
}
#else // not _WIN32 (assume that time.h works)

void tic(DAQPtimer *timer){
    clock_gettime(CLOCK_MONOTONIC, &(timer->start));
}
void toc(DAQPtimer *timer){
    clock_gettime(CLOCK_MONOTONIC, &(timer->stop));
}

double get_time(DAQPtimer *timer){
    struct timespec diff;
    if ((timer->stop.tv_nsec - timer->start.tv_nsec) < 0) {
        diff.tv_sec  = timer->stop.tv_sec - timer->start.tv_sec - 1;
        diff.tv_nsec = 1e9 + timer->stop.tv_nsec - timer->start.tv_nsec;
    } else {
        diff.tv_sec  = timer->stop.tv_sec - timer->start.tv_sec;
        diff.tv_nsec = timer->stop.tv_nsec - timer->start.tv_nsec;
    }
    return (double)diff.tv_sec + (double )diff.tv_nsec / 1e9;
}
#endif // _WIN32
#endif // PROFILING
