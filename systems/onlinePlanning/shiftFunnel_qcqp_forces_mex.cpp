/* Tries to shift a funnel using qcqp (Forces Pro) to bring it out of collision
 *
 * Inputs:
 * x: current state
 * forest: cell array containing vertices of obstacles
 * funnelLibrary: struct containing funnel library
 * funnelLibrary(*).xyz: xyz positions at time points (double 3 X N)
 * funnelLibrary(*).cS: cholesky factorization of projection of S matrix (cell array)
 * funnelIdx: which funnel in library to shift
 *
 * Outputs: 
 * collFree: true iff we successfully brought the funnel out of collision after shifting
 * x_opt: Start position of shifted funnel
 *
 * Author: Anirudha Majumdar
 * Date: March, 2015.
 */

// Mex stuff
// BLAS

#if !defined(_WIN32)
#define dgemm dgemm_
#endif
#include <mex.h>
#include <blas.h>
#include <math.h>
#include <matrix.h>
// #include <iostream>
// #include <stdio.h>
#include <string.h>

// Timer functions
// #include <chrono>
// #include <ctime>
// #include <time.h>


// Internal access to bullet
#include "LinearMath/btTransform.h"

#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "LinearMath/btTransformUtil.h"

using namespace std;

// Set solvers for bullet
static btVoronoiSimplexSolver sGjkSimplexSolver;
static btGjkEpaPenetrationDepthSolver epaSolver;

/*Sphere of radius r representing the point.
 * We could probably make the radius 0 and be ok, but I'm not sure if bullet expects things to be non-degenrate.
 */
static const double g_radius = 1.0;
static btSphereShape* g_point = new btSphereShape(g_radius);

// Make funnel library, funnelIdx, x_current, forest global variables
static const mxArray *funnelLibrary; // Funnel library (pointer)
static int funnelIdx;
static  const mxArray *x_current;
static double *dx_current;
static const mxArray *forest; // Cell array containing forest (pointer)


double ptToPolyBullet(double *vertsPr, size_t nRows, size_t nCols, mxArray *normal_vec){
    
    // Initialize polytope object with single point
    btConvexHullShape polytope(btVector3(vertsPr[0],vertsPr[1],vertsPr[2]), 1);
    
    // Add rest of the points (note the indexing starts from 1 on the loop)
    for(int i=1;i<nCols;i++){
        polytope.addPoint(btVector3(vertsPr[i*nRows],vertsPr[i*nRows+1],vertsPr[i*nRows+2]));
        
    }
    
    // Assign elements of verts (input) to polytope
    btTransform tr;
    btGjkPairDetector::ClosestPointInput input;
    tr.setIdentity();
    input.m_transformA = tr;
    input.m_transformB = tr;
    
    btGjkPairDetector convexConvex(g_point,&polytope,&sGjkSimplexSolver,&epaSolver);
    
    // Output
    btPointCollector gjkOutput;
    
    convexConvex.getClosestPoints(input, gjkOutput, 0);
    
    // mexPrintf("1: %f\n", gjkOutput.m_normalOnBInWorld[0]);
    // mexPrintf("2: %f\n", gjkOutput.m_normalOnBInWorld[1]);
    // mexPrintf("3: %f\n", gjkOutput.m_normalOnBInWorld[2]);
    
    double *normal_vec_d = mxGetPr(normal_vec);
    
    normal_vec_d[0] = gjkOutput.m_normalOnBInWorld[0];
    normal_vec_d[1] = gjkOutput.m_normalOnBInWorld[1];
    normal_vec_d[2] = gjkOutput.m_normalOnBInWorld[2];
    
    return gjkOutput.m_distance + CONVEX_DISTANCE_MARGIN + g_radius;
    
}


/* Shift and transform vertices
 */
double *shiftAndTransform(double *verts, double *vertsT, const mxArray *x, mxArray *x0, int k, mxArray *cSk, size_t nRows, size_t nCols)
{
    /* This can and maybe should be sped up. For example, we know that cSk is upper triangular.*/
    double *dcSk = mxGetPr(cSk);
    double *dx0 = mxGetPr(x0);
    double *dx = mxGetPr(x);
    
    for(int i=0;i<nRows;i++){
        for(int j=0;j<nCols;j++){
            vertsT[j*nRows+i] = dcSk[i]*(verts[j*nRows+0]-dx0[k*nRows+0]-dx[0]) + dcSk[nRows+i]*(verts[j*nRows+1]-dx0[k*nRows+1]-dx[1]) + dcSk[2*nRows+i]*(verts[j*nRows+2]-dx0[k*nRows+2]-dx[2]);
            // mexPrintf("i: %d, j: %d, val: %f\n", i, j, vertsT[j*nRows+i]);
            // mexPrintf("i: %d, j: %d, val: %f\n", i, j, dx0[k*nRows]);
        }
    }
    
    
    return vertsT;
    
}

/* Sets up linear constraints of QCQP.
 */
void setupLinearConstraints(double *A_ineq, double *b_ineq, int numRows_A, int maxObs)
{
    
// Get number of obstacles
    mwSize numObs = mxGetNumberOfElements(forest); // Number of obstacles
    
    
// Initialize some variables
    double *verts; // cell element (i.e. vertices)
    mxArray *x0 = mxGetField(funnelLibrary, funnelIdx, "xyz"); // all points on trajectory
    mxArray *obstacle;
    mxArray *cS = mxGetField(funnelLibrary, funnelIdx, "cS");
    mxArray *cSk;
    size_t nCols;
    size_t nRows;
    double distance;
    mxArray *normal_vec;
    normal_vec = mxCreateDoubleMatrix(1,3,mxREAL);
    
    mxArray *normal_vec_transformed_mx = mxCreateDoubleMatrix(1,3,mxREAL);
    double *normal_vec_transformed = mxGetPr(normal_vec_transformed_mx);
    int rowNum = 0;
    
    
// Get number of time samples
    mwSize N = mxGetNumberOfElements(mxGetField(funnelLibrary, funnelIdx, "cS"));
    
// Check size compatibilities
    const mwSize *N_x0 = mxGetDimensions(x0);
    if (N_x0[1] != N) {
        mexErrMsgTxt("Sizes of x0 and cS do not match!");
    }
    
    // For each time sample
    for(int k=0;k<N;k++)
    {
        // Get pointer to cholesky factorization of S at this time
        cSk = mxGetCell(cS,k);
        
        // For each obstacle
        for(mwIndex jForest=0;jForest<numObs;jForest++)
        {
            
            // Get vertices of this obstacle
            obstacle = mxGetCell(forest, jForest);
            verts = mxGetPr(mxGetCell(forest, jForest)); // Get vertices
            nCols = mxGetN(obstacle);
            nRows = mxGetM(obstacle);
            
            double *vertsT = mxGetPr(mxCreateDoubleMatrix(nRows, nCols, mxREAL));
            
            // Shift vertices so that point on trajectory is at origin and transform by cholesky of S
            vertsT = shiftAndTransform(verts, vertsT, x_current, x0, k, cSk, nRows, nCols);
            
            // Call bullet to find f (distance) and df (normal_vec)
            distance = ptToPolyBullet(vertsT, nRows, nCols, normal_vec);
            
            // Update A_ineq and b_ineq
            
            // Multiply normal_vec by cSk to get it back in the correct coordinate frame (i.e., normal_vec'*cSk)
            double one = 1.0, zero = 0.0; // Seriously?
            long int ione = 1;
            long int dim = 3;
            
            char *chn = "N";
            dgemm(chn, chn, &ione, &dim, &dim, &one, mxGetPr(normal_vec), &ione, mxGetPr(cSk), &dim, &zero, normal_vec_transformed, &ione);
            
            // Update A_ineq and b_ineq
            // A_ineq = [A_ineq;[-normal_vec_transformed,-1]];
            // b_ineq = [b_ineq;distance];
            A_ineq[0*numRows_A+rowNum] = -normal_vec_transformed[0];
            A_ineq[1*numRows_A+rowNum] = -normal_vec_transformed[1];
            A_ineq[2*numRows_A+rowNum] = -normal_vec_transformed[2];
            A_ineq[3*numRows_A+rowNum] = -1.0;
            
            b_ineq[rowNum] = distance - g_radius; // Note the subtraction of g_radius

                    
            // Update row number        
            rowNum = rowNum + 1;
            
        }
    }
    
    // Add constraint to make sure tau >= 0
    // A_ineq = [A_ineq;0,0,0,-1];
    // b_ineq = [b_ineq;0];
    A_ineq[0*numRows_A+rowNum] = 0.0;
    A_ineq[1*numRows_A+rowNum] = 0.0;
    A_ineq[2*numRows_A+rowNum] = 0.0;
    A_ineq[3*numRows_A+rowNum] = -1.0;
    rowNum = rowNum + 1;
    
    // Now pad A_ineq and b_ineq
    // pad_length = N*(maxObs - numObs);
    // A_ineq = [A_ineq;zeros(pad_length,4)];
    // b_ineq = [b_ineq;zeros(pad_length,1)];
    int pad_length = N*(maxObs - numObs);
    for(int k=0;k<pad_length;k++)
    {
        A_ineq[0*numRows_A+(rowNum+k)] = 0.0;
        A_ineq[1*numRows_A+(rowNum+k)] = 0.0;
        A_ineq[2*numRows_A+(rowNum+k)] = 0.0;
        A_ineq[3*numRows_A+(rowNum+k)] = 0.0;
        
        b_ineq[rowNum+k] = 0.0;
        
        
    }

}

/* Sets up quadratic part of constraint in QCQP
 */
void setupQuadraticConstraint(double *ql, double *qr)
{
        
    // Initialize some variables
    mxArray *x0 = mxGetField(funnelLibrary, funnelIdx, "x0"); // all points on trajectory
    
    double *dx0 = mxGetPr(x0);
    // double *dx_current = mxGetPr(x_current);
    
    long int dim = 12; // mxGetM(x_current); // Dimension of state
    long int dimx0 = mxGetM(x0);
    
// Check that we got the right dimensions
    if(dim > 1){
        if (dim != dimx0){
            mexErrMsgTxt("x and x0 have different dimensions!");
        }
    }
    else{
        mexErrMsgTxt("State seems to have the wrong dimension");
    }
    
// Get S matrix at time 0
    mxArray *pS0 = mxGetField(funnelLibrary, funnelIdx, "S0");
    double *S0 = mxGetPr(pS0);
    
    // Get S22 = S0(4:end,4:end)
    mxArray *S22_mx = mxCreateDoubleMatrix(dim-3,dim-3,mxREAL);
    double *S22 = mxGetPr(S22_mx);
    for(int i=3;i<dim;i++)
    {
        for(int j=3;j<dim;j++)
        {
            S22[(j-3)*(dim-3)+(i-3)] = S0[j*dim+i];  
            
        }
        
    }
    
    // Now get S12 = S0(1:3,4:end);
    mxArray *S12_mx = mxCreateDoubleMatrix(3,dim-3,mxREAL);
    double *S12 = mxGetPr(S12_mx);
    for(int i=0;i<3;i++)
    {
        for(int j=3;j<dim;j++)
        {
            S12[(j-3)*(3)+(i)] = S0[j*dim+i];  
            // mexPrintf("S12: %f \n", S12[(j-3)*(3)+(i)]);
        }
    }
    
    // Get v = x0(4:end) - x_current(4:end); % Difference in non-cyclic dimensions
    mxArray *v = mxCreateDoubleMatrix(dim-3,1,mxREAL);
    double *v_d = mxGetPr(v);
    for(int k=0;k<(dim-3);k++)
    {
     v_d[k] = dx0[k+3] - dx_current[k+3]; 
     // mexPrintf("v_d: %f \n", v_d[k]);
    }
    
// Now compute qr = 1 - v'*S22*v using lapack
// First do S22*v
    double one = 1.0, zero = 0.0; // Seriously?
    long int ione = 1;
    long int dimv = dim-3;
    mxArray *S22v = mxCreateDoubleMatrix(dimv,1,mxREAL);
    double *dS22v = mxGetPr(S22v);
    char *chn = "N";
    dgemm(chn, chn, &dimv, &ione, &dimv, &one, S22, &dimv, v_d, &dimv, &zero, dS22v, &dimv);    
    
// Now do qr = 1 - v'*S22v. Note: we're doing this entire operation using dgemm
    double minus_one = -1.0;
    *qr = 1.0;
    char *chnT = "T"; // since we want xrel transpose
    // dgemm(chnT, chn, &ione, &ione, &dimv, &one, v_d, &dimv, dS22v, &dimv, &zero, &C, &ione);
    dgemm(chnT, chn, &ione, &ione, &dimv, &minus_one, v_d, &dimv, dS22v, &dimv, &one, qr, &ione);
    
    // Now compute ql = 2*S12*v;
    double two = 2.0;
    long int ithree = 3;
    dgemm(chn, chn, &ithree, &ione, &dimv, &two, S12, &ithree, v_d, &dimv, &zero, ql, &ithree);
    
    
    
}


// Shift funnel using qcqp
bool shiftFunnelQCQP(int funnelIdx, const mxArray *funnelLibrary, const mxArray *forest, mwSize numObs, double *x_opt)
{
    
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     *  Calls Forces Pro solver mex function.
     * 
     *  Uses a QCQP to try to shift a funnel out of collision while keeping the
     *  current state in the inlet of the (shifted) funnel.
     * 
     *  Sets up and solves the following optimization problem:
     *  min   max(0,tau)
     * 
     *  s.t.  (x_current-x_shifted_full)^T S0 (x_current-x_shifted_full) <= 1
     *        tau + ns_jk*x_shiftl(1:3) + ds_jk >= 0, for all j,k
     * 
     *  Here, the decision variables are x_shift. x_shifted_full is:
     *  [x_shift+x_current(1:3);x0(4:end)] (x0 is funnel.x0(:,1)).
     * 
     *  ns_jk and ds_jk are the collision normal and penetration distance
     *  respectively. The indices j and k iterate over obstacles and time-steps
     *  in the funnel respectively.
     * 
     *  Returns the shift required (not the absolute position).
     * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
        
    // Maximum number of obstacles we can handle
    int maxObs = 20;
    
    if (numObs > maxObs)
    {
        // Error
        mexErrMsgTxt("Number of obstacles is greater than maximum obstacles we can handle (20). If you want more obstacles, regenerate forces code and change maxObs above");
    }
    
    // Get number of time samples
    mwSize N = mxGetNumberOfElements(mxGetField(funnelLibrary, funnelIdx, "cS"));
    
    // First setup constraints
    int numRows_A = maxObs*N+1;
    mxArray *A_ineq = mxCreateDoubleMatrix(numRows_A,4,mxREAL);
    double *A_ineq_d = mxGetPr(A_ineq);
    mxArray *b_ineq = mxCreateDoubleMatrix(numRows_A,1,mxREAL);
    double *b_ineq_d = mxGetPr(b_ineq);
    
    // Linear constraints
    setupLinearConstraints(A_ineq_d, b_ineq_d, numRows_A, maxObs);  
    
    // Now setup quadratic constraints
    mxArray *ql = mxCreateDoubleMatrix(3,1,mxREAL);
    double *ql_d = mxGetPr(ql);
    double qr;
    setupQuadraticConstraint(ql_d,&qr);
    
    /*mexPrintf("qr: %f \n", qr);
    mexPrintf("ql[0]: %f \n", ql_d[0]);
    mexPrintf("ql[1]: %f \n", ql_d[1]);
    mexPrintf("ql[2]: %f \n", ql_d[2]);*/
    
    // Now create problem struct to pass to forces solver mex code.
    const char *field_names[] = {"A", "b", "ql", "qr"};
    mwSize ndim = 1;
    const mwSize ndims = 1;
    mxArray *problem = mxCreateStructArray(ndim,&ndims,4,field_names);
    mxSetField(problem, 0, "A", A_ineq);
    mxSetField(problem, 0, "b", b_ineq);
    mxSetField(problem, 0, "ql", ql);
    mxSetField(problem, 0, "qr", mxCreateDoubleScalar(qr));
    
    // Initialize result struct
    mxArray *lhs_forces[1];
    mxArray *result;
    lhs_forces[0] = result;
    mxArray *forces_rhs[1];
    forces_rhs[0] = problem;
    
    // Call funnel shifting forces pro mex code
    // The Mex file to call depends on the funnelIdx. So, for example, if 
    // we want to shift the third funnel, the mex file is "funnel3_shift".
    if (funnelIdx+1 > 9)
    {
        char function_name[15];
        snprintf(function_name, sizeof function_name, "%s%d%s", "funnel", funnelIdx+1, "_shift");
        const char *function_name_char = &function_name[0];
        // mexCallMATLAB(1, lhs_forces, 1, forces_rhs, "funnel1_shift");
        mexCallMATLAB(1, lhs_forces, 1, forces_rhs, function_name_char);
    }
    else
    {
        char function_name[14];
        snprintf(function_name, sizeof function_name, "%s%d%s", "funnel", funnelIdx+1, "_shift");
        const char *function_name_char = &function_name[0];
        // mexCallMATLAB(1, lhs_forces, 1, forces_rhs, "funnel1_shift");
        mexCallMATLAB(1, lhs_forces, 1, forces_rhs, function_name_char);
    }
    
        
    // Get x_opt and tau from result structure
    mxArray *x_opt_forces = mxCreateDoubleMatrix(3,1,mxREAL);
    mxArray *tau_forces = mxCreateDoubleScalar(mxREAL);
    
    x_opt_forces = mxGetField(lhs_forces[0], 0, "x_shift");
    tau_forces = mxGetField(lhs_forces[0], 0, "tau");
        
    double *x_opt_forces_d = mxGetPr(x_opt_forces);
    double *tau_d = mxGetPr(tau_forces);
    // mexPrintf("x_opt[0]: %f \n", x_opt_forces_d[0]);
    // mexPrintf("x_opt[1]: %f \n", x_opt_forces_d[1]);
    // mexPrintf("x_opt[2]: %f \n", x_opt_forces_d[2]);
    
    // Assign to x_opt
    x_opt[0] = *x_opt_forces_d;
    x_opt[1] = *(x_opt_forces_d+1);
    x_opt[2] = *(x_opt_forces_d+2);
    
    // Check if we are collision free (tau <= 1e-4. well, really tau == 0,
    // but there will be numerical crap from the solver.)
    bool collFree = false; // mexPrintf("tau: %f \n", *tau_d);
    if (*tau_d < 1e-4)
    {
        collFree = true;
    }
    
    // Destroy struct and other stuff we don't need
    mxDestroyArray(*lhs_forces);
    mxDestroyArray(*forces_rhs);
    // mxDestroyArray(result);
    
    
    
    // Return collFree
    return collFree;

}


/* Main mex funtion*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    
// Get current state (from which nominal/unshifted funnel would be executed)
    // const mxArray *x_current;
    x_current = prhs[0];
    dx_current = mxGetPr(x_current);
       
    
// Deal with forest cell (second input)
    forest = prhs[1]; // Get forest cell (second input)
    mwSize numObs = mxGetNumberOfElements(forest); // Number of obstacles
    
    funnelLibrary = prhs[2]; // Get funnel library (third input)
    
// Get funnelIdx (subtract 1 since index is coming from matlab)
    // int funnelIdx;
    funnelIdx = (int )mxGetScalar(prhs[3]);
    funnelIdx = funnelIdx-1;
    
    // Optimal shift from qcqp
    mxArray *x_opt = mxCreateDoubleMatrix(3,1,mxREAL);
    double *x_opt_d = mxGetPr(x_opt);
    
    
// Initialize next funnel (output of this function)
    bool collFree = shiftFunnelQCQP(funnelIdx, funnelLibrary, forest, numObs, x_opt_d);
    
    // Add in x_current to get SHIFTED position of funnel
    x_opt_d[0] = x_opt_d[0] + dx_current[0];
    x_opt_d[1] = x_opt_d[1] + dx_current[1];
    x_opt_d[2] = x_opt_d[2] + dx_current[2];
    
// Return collFree
    if (nlhs > 0){
         plhs[0] = mxCreateLogicalScalar(collFree);
    }
    
// Return x_opt if asked for
    if (nlhs > 1){
        plhs[1] = x_opt;
    }
    
    return;
}
















