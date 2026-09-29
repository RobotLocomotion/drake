#include "bnb.h"
#ifdef PROFILING
#include "utils.h"
#endif

static c_float daqp_binary_diff(const int id, DAQPWorkspace* work){
    int j, disp;
    c_float diff = 0.5*(work->dupper[id]+work->dlower[id]);

    if(id < work->ms){//Simple bound
        if(work->Rinv==NULL) diff -= work->u[id]; //Hessian is identity
        else{
            for(j=id,disp=id+DAQP_R_OFFSET(id,work->n);j<work->n;j++)
                diff -= work->Rinv[disp++]*work->u[j];
        }
    }
    else{//General bound; daqp_add_infeasible already computed M*u
        diff -= work->Mu[id-work->ms];
    }
    return diff;
}

// Store the free part of the working set in ids. Returns the number stored.
static int daqp_bnb_store_ws(int* ids, DAQPWorkspace* work){
    int i, n_ids = 0;
    for(i=work->bnb->neq; i<work->n_active;i++){
        if((work->sense[work->WS[i]]&(DAQP_IMMUTABLE+DAQP_BINARY))!=DAQP_IMMUTABLE+DAQP_BINARY)
            ids[n_ids++] = work->WS[i]+(DAQP_IS_LOWER(work->WS[i]) << (DAQP_LOWER_BIT-1));
    }
    return n_ids;
}

// Add the constraints in ids to the working set (aborted if the basis gets singular)
static void daqp_bnb_load_ws(const int* ids, const int n_ids, DAQPWorkspace* work){
    int i;
    for(i=0; i < n_ids; i++){
        daqp_add_upper_lower(ids[i],work);
        if(work->sing_ind != DAQP_EMPTY_IND) {
            work->n_active--;
            DAQP_SET_INACTIVE(work->WS[work->n_active]);
            work->sing_ind = DAQP_EMPTY_IND;
            break;
        }
    }
}

// The immutable constraints (e.g., equalities) are kept fixed as a prefix of
// the working set throughout the tree. Returns the length of that prefix.
static int daqp_bnb_setup_root(DAQPWorkspace* work){
    int i, j, error_flag;
    for(i = 0; i < work->n_active; i++)
        if(!DAQP_IS_IMMUTABLE(work->WS[i])) break;
    for(j = i; j < work->n_active; j++)
        if(DAQP_IS_IMMUTABLE(work->WS[j])) break;
    if(j == work->n_active) return i; // Mutable constraints are a warm start

    // Immutable after mutable => only activate immutable constraints
    for(i = 0; i < work->n_active; i++)
        if(!DAQP_IS_IMMUTABLE(work->WS[i])) DAQP_SET_INACTIVE(work->WS[i]);
    reset_daqp_workspace(work);
    error_flag = daqp_activate_constraints(work);
    return error_flag < 0 ? error_flag : work->n_active;
}

// Use the candidate in work->x as incumbent. Returns its objective
// (internal scale) with u = R*x+v stored in work->xold or -1 if infeasible.
static c_float daqp_bnb_incumbent(DAQPWorkspace* work){
    int i, j, disp;
    const int n = work->n;
    c_float val, fval = 0, tol = work->settings->primal_tol;
    c_float *x = work->x, *u = work->xold;
    DAQPProblem* qp = work->qp;
    if(qp == NULL) return -1;

    // Check feasibility (soft constraints are treated as hard)
    for(i=0, disp=0; i < work->m; i++){
        if(i < work->ms) val = x[i];
        else for(j=0, val=0; j < n; j++) val += qp->A[disp++]*x[j];
        if(DAQP_IS_IMMUTABLE(i) && !DAQP_IS_ACTIVE(i) && !DAQP_IS_BINARY(i)) continue; // Ignored
        if(val > qp->bupper[i]+tol || val < qp->blower[i]-tol) return -1;
        if(DAQP_IS_BINARY(i) && val > qp->blower[i]+tol && val < qp->bupper[i]-tol) return -1;
    }

    // Invert ldp2qp_solution: u = R*x + v
    for(i=0; i < n; i++) u[i] = x[i];
    if(work->Rinv != NULL){
        if(work->scaling != NULL)
            for(i=0; i < work->ms; i++) u[i] *= work->scaling[i];
        for(i=n-1; i >= 0; i--){ // Back substitution with upper triangular Rinv
            disp = i+DAQP_R_OFFSET(i,n);
            for(j=i+1; j < n; j++) u[i] -= work->Rinv[disp+j-i]*u[j];
            u[i] /= work->Rinv[disp];
        }
    }
    else if(work->RinvD != NULL)
        for(i=0; i < n; i++) u[i] /= work->RinvD[i];
    if(work->v != NULL)
        for(i=0; i < n; i++) u[i] += work->v[i];

    for(i=0; i < n; i++) fval += u[i]*u[i];
    return fval;
}

int daqp_bnb(DAQPWorkspace* work){
    int branch_id, exitflag;
    DAQPNode* node;
    c_float *swp_ptr = NULL;

    exitflag = daqp_bnb_setup_root(work);
    if(exitflag < 0) return exitflag;
    work->bnb->neq = exitflag;

    // Warm start the root with the root working set of the previous solve
    // (unless a warm start has been provided)
    if(work->n_active == work->bnb->neq)
        daqp_bnb_load_ws(work->bnb->root_WS,work->bnb->n_root_WS,work);

    // Modify upper bound based on absolute/relative suboptimality tolerance
    c_float fval_bound0 = work->settings->fval_bound;
    c_float eps_r = 1/(1+work->settings->rel_subopt);
    work->settings->fval_bound = (fval_bound0 - work->settings->abs_subopt)*eps_r;

    // Start from a user-provided integer-feasible solution
    if(work->state & DAQP_STATE_INCUMBENT){
        work->state &= ~DAQP_STATE_INCUMBENT;
        c_float fval_inc = 0.5*daqp_bnb_incumbent(work);
        if(fval_inc >= 0 && fval_inc < fval_bound0){
            work->settings->fval_bound = (fval_inc - work->settings->abs_subopt)*eps_r;
            swp_ptr = work->xold; // Marks that a feasible solution is stored in xold
        }
    }

    work->bnb->itercount=0;
    work->bnb->nodecount=0;
    // Setup root node
    work->bnb->tree[0].depth=-1;
    work->bnb->tree[0].WS_start=0;
    work->bnb->tree[0].WS_end=0;
    work->bnb->tree[0].bin_id=0;
    work->bnb->n_nodes=1;
    work->bnb->n_clean=work->bnb->neq;
    work->bnb->nWS=0;

    exitflag = DAQP_EXIT_INFEASIBLE;
    // Start tree exploration
    while( work->bnb->n_nodes > 0 ){

        node = work->bnb->tree+(--work->bnb->n_nodes);
        exitflag = daqp_process_node(node,work); // Solve relaxation
        if(node->depth < 0 && exitflag > 0 && work->bnb->root_WS != NULL)
            work->bnb->n_root_WS = daqp_bnb_store_ws(work->bnb->root_WS,work);
#ifdef PROFILING
        // Individual relaxations are often too short to reach the timer check
        // in daqp_ldp, so also enforce the limit across the BnB tree.
        if(work->timer != NULL && (work->bnb->nodecount&31)==0){
            toc((DAQPtimer*)work->timer);
            if(get_time((DAQPtimer*)work->timer) > work->settings->time_limit){
                exitflag = DAQP_EXIT_TIMELIMIT;
                break;
            }
        }
#endif
        // Cut conditions
        if(exitflag==DAQP_EXIT_INFEASIBLE) continue; // Dominance cut
        if(exitflag<0) break; // Inner solver failed => exit loop

        // Find index to branch over
        branch_id = daqp_get_branch_id(work);
        if(branch_id==DAQP_EMPTY_IND){// Nothing to branch over => integer feasible
            work->settings->fval_bound = (0.5*work->fval - work->settings->abs_subopt)*eps_r;
            swp_ptr=work->xold; work->xold= work->u; work->u=swp_ptr; // Store feasible sol
        }
        else{
            daqp_spawn_children(node,branch_id, work);
        }
    }

    // Exploration completed
    work->iterations = work->bnb->itercount;
    // Restore the root state (unfix binaries etc.) so that the workspace can
    // be reused for subsequent solves
    daqp_node_cleanup_workspace(work->bnb->neq,work);
    work->bnb->n_clean = work->bnb->neq;
    if(swp_ptr==NULL){
        work->settings->fval_bound = fval_bound0;
        return exitflag < 0 ? exitflag : DAQP_EXIT_INFEASIBLE;
    }
    else{
        // Invert fval_bound = (0.5*fval_best - abs_subopt)*eps_r to recover fval_best
        work->fval = 2*work->settings->fval_bound/eps_r + 2*work->settings->abs_subopt;
        work->settings->fval_bound = fval_bound0;
        // Let work->u point to the best feasible solution
        swp_ptr=work->u; work->u= work->xold; work->xold=swp_ptr;
        return exitflag < DAQP_EXIT_INFEASIBLE ? exitflag : DAQP_EXIT_OPTIMAL;
    }
}

int daqp_process_node(DAQPNode* node, DAQPWorkspace* work){
    int exitflag;
    work->bnb->nodecount+=1;
    if(node->depth >=0){
        // Fix a binary constraints
        work->bnb->fixed_ids[node->depth] = node->bin_id;
        // Setup relaxation
        if(work->bnb->n_nodes==0 || (node-1)->depth!=node->depth){
            // Sibling has been processed => need to fix workspace state
            work->bnb->n_clean += (node->depth-(node+1)->depth);
            daqp_node_cleanup_workspace(work->bnb->n_clean,work);
            daqp_warmstart_node(node,work);
        }
        else{
            daqp_add_upper_lower(node->bin_id,work);
            work->sense[DAQP_REMOVE_LOWER_FLAG(node->bin_id)] |= DAQP_IMMUTABLE; //Equality
            if(work->sing_ind != DAQP_EMPTY_IND){ // Need to cold start to not miss integer feasible
                daqp_setup_cold_bnb(node,work);
            }
        }
        // Add binary constraint
    }
    // Solve relaxation
    exitflag = daqp_ldp(work);
    work->bnb->itercount += work->iterations;

    if(exitflag == DAQP_EXIT_CYCLE){// Try to repair (cold start)
        // A cycle can be caused by stale cached forward-substitution data.
        // Force the retained fixed prefix to be recomputed during repair.
        work->reuse_ind=0;
        daqp_setup_cold_bnb(node,work);
        exitflag = daqp_ldp(work);
        work->bnb->itercount += work->iterations;
    }

    return exitflag;
}

int daqp_get_branch_id(DAQPWorkspace* work){
    int i;
    int id = DAQP_EMPTY_IND;
    c_float diff, dist, tol;

    for(i=0; i < work->bnb->nb; i++){
        id = work->bnb->bin_ids[i];
        // Skip fixed binary constraints
        if(DAQP_IS_ACTIVE(id)) continue;

        // Compute signed distance from midpoint between bounds
        diff = daqp_binary_diff(id,work);

        // A zero-dual binary constraint can lie at an endpoint without being
        // active. It is already integer feasible and does not need branching.
        dist = 0.5*(work->dupper[id]-work->dlower[id])
            -(diff < 0 ? -diff : diff);
        tol = work->settings->primal_tol;
        if(work->scaling != NULL) tol *= work->scaling[id];
        if(dist <= tol) continue;

        // Explore the endpoint nearest to the relaxation first.
        return diff < 0 ? id : DAQP_ADD_LOWER_FLAG(id);
    }

    return DAQP_EMPTY_IND;
}

void daqp_spawn_children(DAQPNode* node, const int branch_id, DAQPWorkspace* work){

    daqp_save_warmstart(node,work);

    // Update child1 (reuse current node)
    node->bin_id = DAQP_TOGGLE_LOWER_FLAG(branch_id);
    node->depth +=1;

    // Update child2
    (node+1)->bin_id = branch_id;
    (node+1)->depth = node->depth;
    (node+1)->WS_start = node->WS_start;
    (node+1)->WS_end= node->WS_end;

    work->bnb->n_nodes+=2;
}

void daqp_node_cleanup_workspace(int n_clean, DAQPWorkspace* work){
    int i;
    // Cleanup sense
    for(i=n_clean; i<work->n_active; i++)
        work->sense[work->WS[i]]&= DAQP_IS_BINARY(work->WS[i]) ?
            ~(DAQP_ACTIVE+DAQP_IMMUTABLE): ~DAQP_ACTIVE;
    // Reset workspace
    work->sing_ind=DAQP_EMPTY_IND;
    work->n_active=n_clean;
    // Only the retained prefix can still have a valid cached substitution.
    if(work->reuse_ind > n_clean)
        work->reuse_ind=n_clean;
}


void daqp_warmstart_node(DAQPNode* node, DAQPWorkspace* work){
    int i;
    // Add fixed constraints
    for(i=work->bnb->n_clean - work->bnb->neq; i< node->depth+1;i++){
        daqp_add_upper_lower(work->bnb->fixed_ids[i],work);
        DAQP_SET_IMMUTABLE(DAQP_REMOVE_LOWER_FLAG(work->bnb->fixed_ids[i]));
    }
    work->bnb->n_clean = work->bnb->neq+node->depth;
    // Add free constraints
    daqp_bnb_load_ws(work->bnb->tree_WS+node->WS_start,node->WS_end-node->WS_start,work);
    work->bnb->nWS = node->WS_start; // always move up tree after warmstart
}

void daqp_save_warmstart(DAQPNode* node, DAQPWorkspace* work){
    node->WS_start = work->bnb->nWS;
    work->bnb->nWS += daqp_bnb_store_ws(work->bnb->tree_WS+work->bnb->nWS,work);
    node->WS_end = work->bnb->nWS;
}

int daqp_add_upper_lower(const int add_id, DAQPWorkspace* work){
    int true_add_id = DAQP_REMOVE_LOWER_FLAG(add_id);
    // Setup new binary constraint
    if(DAQP_EXTRACT_LOWER_FLAG(add_id)){
        DAQP_SET_LOWER(true_add_id);
        daqp_add_constraint(work,true_add_id,-1.0);
    }
    else{
        DAQP_SET_UPPER(true_add_id);
        daqp_add_constraint(work,true_add_id,1.0);
    }
    return 1;
}

void daqp_setup_cold_bnb(DAQPNode* node,DAQPWorkspace *work){
    int i;
    daqp_node_cleanup_workspace(work->bnb->n_clean,work);
    for(i=work->bnb->n_clean - work->bnb->neq; i< node->depth+1;i++){
        daqp_add_upper_lower(work->bnb->fixed_ids[i],work);
        DAQP_SET_IMMUTABLE(DAQP_REMOVE_LOWER_FLAG(work->bnb->fixed_ids[i]));
    }
    work->bnb->n_clean = work->bnb->neq+node->depth;
}
