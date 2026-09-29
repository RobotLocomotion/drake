#include "factorization.h"

 // Necessary to retain accuracy when FAST_MATH is used 
static c_float dot_row(const c_float* a, const c_float* b, const int n){
    c_float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    int i;
    for(i = 0; i+3 < n; i += 4){
        s0 += a[i]*b[i];
        s1 += a[i+1]*b[i+1];
        s2 += a[i+2]*b[i+2];
        s3 += a[i+3]*b[i+3];
    }
    for(; i < n; i++) s0 += a[i]*b[i];
    return (s0+s1)+(s2+s3);
}

c_float daqp_dot(const c_float* v1, const c_float* v2, const int n) {
    return daqp_dot_inline(v1, v2, n);
}

void daqp_update_LDL_add(DAQPWorkspace *work, const int add_ind, const c_float rho){
    work->sing_ind = DAQP_EMPTY_IND;
    int i,j,disp,id;
    int new_L_start= DAQP_ARSUM(work->n_active);
    int start_col;
    int ns_active=0;
    c_float sum;
    c_float *Mi, *Mk;

    // di <-- Mi' Mi
    // If normalized this will always be 1...
    if(add_ind < work->ms){
        Mi = (work->Rinv)? work->Rinv+DAQP_R_OFFSET(add_ind,work->n): NULL;
        start_col = add_ind;
    }
    else{
        Mi = work->M+work->n*(add_ind-work->ms);
        start_col = 0;
    }
    if(Mi==NULL) sum = 1;
    else
        sum = dot_row(Mi+start_col,Mi+start_col,work->n-start_col);

    // A nonzero rho marks a free soft slack and contributes to the diagonal
    if(rho != 0){
        sum += rho;
        ns_active++;
    }

    work->D[work->n_active] = sum;

    if(work->n_active==0) return;

    // store l <-- Mk* m
    for(i=0;i<work->n_active;i++){
        id = work->WS[i];
        if(DAQP_IS_SOFT(id) && DAQP_IS_SLACK_FREE(id)) ns_active++;
        // Use Rinv or M for Mk depending on if k is simple bound or not 
        if(id < work->ms){ 
            Mk = (work->Rinv) ? work->Rinv+DAQP_R_OFFSET(id,work->n): NULL;
            j= (start_col > id) ? start_col : id;
        }
        else{
            Mk = work->M+work->n*(id-work->ms);
            j= start_col;
        }
        // Multiply Mk*Mi (NULL signify unity)
        if(Mk == NULL)
            sum = (Mi ==NULL) ? 0 : Mi[j];
        else if(Mi == NULL)
            sum = Mk[j];
        else
            sum = dot_row(Mk+j,Mi+j,work->n-j);

        work->L[new_L_start+i] = sum;
    }
    //Forward substitution: l <-- L\(Mk*m)
    for(i=0,disp=0; i<work->n_active; i++){
        sum = work->L[new_L_start+i];
        for(j=0; j<i; j++)
            sum -= work->L[disp++]*work->L[new_L_start+j];
        work->L[new_L_start+i] = sum;
        disp++; //Skip diagonal elements (which is 1)
    }

    // Scale: l_i <-- l_i/d_i
    // Update d_new -= l'Dl
    sum = work->D[work->n_active];
    c_float tmp;
    for (i =0,disp=new_L_start; i<work->n_active;i++,disp++){
        tmp = work->L[disp];
        work->L[disp] /= work->D[i];
        sum -= tmp*work->L[disp];
    }
    work->D[work->n_active]=sum;

    // Check for singularity
    if(work->D[work->n_active] < work->settings->sing_tol ||
            (work->n_active >= work->n + ns_active)){
        work->sing_ind=work->n_active;
    }
}
void daqp_update_LDL_remove(DAQPWorkspace *work, const int rm_ind){
    if(work->n_active==rm_ind+1)
        return;
    int i, j, r, old_disp, new_disp, w_count, n_update=work->n_active-rm_ind-1;
    c_float* w = &work->zldl[rm_ind]; // zldl will be obsolete => use to allocations
    // Extract parts to keep/update in L & D
    new_disp=DAQP_ARSUM(rm_ind);
    old_disp=new_disp+(rm_ind+1);
    w_count= 0;
    // Remove column rm_ind (and add parts of L in its new place)
    // I.e., copy row i into i-1
    for(i = rm_ind+1;i<work->n_active;old_disp++,new_disp++,i++) //(disp++ skips blank element)..
        for(j=0;j<i;j++){
            if(j!=rm_ind)
                work->L[new_disp++]=work->L[old_disp++];
            else
                w[w_count++] = work->L[old_disp++];
        }
    // Algorithm C1 in Gill 1974 for low-rank update of LDL
    // L2 block
    c_float p,beta,dbar,alpha=work->D[rm_ind];
    // i - Element/row to update|j - Column which is looped over|r - Row to loop over
    old_disp=DAQP_ARSUM(rm_ind)+rm_ind;
    for(j = 0, i=rm_ind+1;j<n_update;j++,i++){
        p=w[j];
        dbar = work->D[i]+alpha*p*p;
        work->D[i-1] = dbar;


        beta = p*alpha/dbar;
        alpha =work->D[i]*alpha/dbar;

        old_disp+=i;
        for(r=j+1, new_disp=old_disp+j;r<n_update;r++){
            w[r] -= p*work->L[new_disp];
            work->L[new_disp]+=beta*w[r];
            new_disp+=rm_ind+r+1; //Update to the id which starts the next row in L
        }
    }
}
