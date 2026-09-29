#include "daqp.h"
#ifdef PROFILING
#include "utils.h"
#endif

int daqp_ldp(DAQPWorkspace *work){
    int exitflag=DAQP_EXIT_ITERLIMIT,iter,i;
    work->soft_slack = 0; // Only set when a solution is found
    int tried_repair=0, cycle_counter=0;
    int refine_adds=0; // Refinements without progress that added a constraint
    c_float best_fval = -1;
    c_float fval_bound = 2*work->settings->fval_bound; // Internal objective is twice the nomninal

    // Correctly cleanup a singular working-set on entry
    if(work->sing_ind != DAQP_EMPTY_IND && work->sing_ind == work->n_active-1 &&
            !DAQP_IS_IMMUTABLE(work->WS[work->sing_ind])){
        work->n_active--;
        DAQP_SET_INACTIVE(work->WS[work->n_active]);
        work->sing_ind = DAQP_EMPTY_IND;
        if(work->reuse_ind > work->n_active) work->reuse_ind = work->n_active;
    }

    for(iter=1; iter < work->settings->iter_limit; ++iter){

        if(work->sing_ind==DAQP_EMPTY_IND){
            daqp_compute_CSP(work);
            // Check dual feasibility of CSP
            if(!daqp_remove_blocking(work)){ //lam_star >= 0 (i.e., dual feasible)
                daqp_compute_primal_and_fval(work);
                // fval termination criterion
                if(work->fval > fval_bound){
                    exitflag = DAQP_EXIT_INFEASIBLE;
                    break;
                }
                // Try to add infeasible constraint
                if(!daqp_add_infeasible(work)){ //mu >= (i.e., primal feasible)
                                           // All KKT-conditions satisfied -> optimum found

                    c_float min_D = work->D[0];
                    for(i = 1; i < work->n_active; i++)
                        if(work->D[i] < min_D) min_D = work->D[i];

                    // If LDL is truly ill-conditioned, refactor for a better pivot ordering
                    // (Not in BnB, which relies on the order of the working set)
                    if(work->n_active > 2 && tried_repair != 1 && work->bnb == NULL &&
                            min_D < work->settings->refactor_tol){
                        tried_repair = 1;
                        // Correct LOWER/UPPER (important for equality constraints)
                        for(i = 0; i < work->n_active; i++){
                            if (work->lam[i] >= 0)
                                DAQP_SET_UPPER(work->WS[i]);
                            else
                                DAQP_SET_LOWER(work->WS[i]);
                        }
                        reset_daqp_workspace(work);
                        daqp_activate_constraints(work);
                        continue; // Try again with new LDL factorization
                    }

                    // If the LDL is near-singular, backward errors in lam_star are
                    // amplified by 1/scaling in original space.  Apply one step of
                    // iterative refinement to correct both u and lam_star before
                    // declaring optimal.
                    // If the refined and the unrefined solution keep disagreeing
                    // about a constraint (it is added after the refinement and
                    // then removed again without progress), further refinements
                    // repeat that deterministically, so they are stopped after
                    // two such attempts and the current solution is accepted.
                    if(work->n_active > 0 && min_D < work->settings->pivot_tol &&
                            refine_adds < 2){
                        daqp_refine_active(work);
                        // A constraint added after the refinement goes through
                        // the cycle guard, like any other addition
                        if(daqp_add_infeasible(work)){
                            refine_adds++;
                            goto cycle_guard;
                        }
                    }


                    // Softening was needed if a soft constraint ended up violated
                    work->soft_slack = daqp_max_soft_slack(work);
                    if(work->soft_slack > work->settings->primal_tol)
                        exitflag = DAQP_EXIT_SOFT_OPTIMAL;
                    else
                        exitflag = DAQP_EXIT_OPTIMAL;
                    break;
                }

cycle_guard:
                // Cycle guard
                if(work->fval-best_fval < work->settings->progress_tol){
                    if(cycle_counter++ > work->settings->cycle_tol){
                        if(tried_repair == 1 || work->bnb != NULL){
                            exitflag = DAQP_EXIT_CYCLE;
                            break;
                        }
                        else{// Cycling -> Try to reorder and refactorize LDL
                            tried_repair =1;
                            reset_daqp_workspace(work);
                            daqp_activate_constraints(work);
                            cycle_counter=0;
                            best_fval = -1;
                        }
                    }
                }
                else{ // Progress was made
                    best_fval = work->fval;
                    cycle_counter = 0;
                    refine_adds = 0;
                }
            }
        }
        else{// Singular case
            daqp_compute_singular_direction(work);
            if(!daqp_remove_blocking(work)){
                exitflag = DAQP_EXIT_INFEASIBLE;
                break;
            }
        }
#ifdef PROFILING
        if(work->timer != NULL && iter % 32 == 0){
            toc((DAQPtimer*)work->timer);
            if(get_time((DAQPtimer*)work->timer) > work->settings->time_limit){
                exitflag = DAQP_EXIT_TIMELIMIT;
                break;
            }
        }
#endif
    }
    // Finalize result before returning
    work->iterations = iter;
    return exitflag;
}

// Compute x = -R\(u+v)
void ldp2qp_solution(DAQPWorkspace *work){
    int i,j,disp;
    // x* = Rinv*(u-v)
    if(work->v != NULL)
        for(i=0;i<work->n;i++) work->x[i]=work->u[i]-work->v[i];
    else
        for(i=0;i<work->n;i++) work->x[i]=work->u[i];

    if(work->Rinv != NULL){ // (Skip if LP since R = I)
        for(i=0,disp=0;i<work->n;i++){
            work->x[i]*=work->Rinv[disp++];
            for(j=i+1;j<work->n;j++)
                work->x[i]+=work->Rinv[disp++]*work->x[j];
        }
        if(work->scaling != NULL){
            for(i=0;i<work->ms;i++)
                work->x[i]/=work->scaling[i];
        }
    }
    else if(work->RinvD != NULL)
    {
        for(i=0;i<work->n;i++)
            work->x[i]*=work->RinvD[i];
    }
    if(work->scaling != NULL){ // Correctly scale output
        for(i=0;i<work->n_active;i++)
            work->lam_star[i]*=work->scaling[work->WS[i]];
    }
}

// Reset workspace to default values
void reset_daqp_workspace(DAQPWorkspace *work){
    work->sing_ind=DAQP_EMPTY_IND;
    work->n_active =0;
    work->reuse_ind=0;
}
