/* Finds a collision free funnel from a funnel library, given a decription of obstacle positions.
 *
 * Sketch of algorithm:
 * Go through funnels one by one (in order)
 * If we're not in inlet of funnel, continue
 * If a collision free funnel is found, we're done
 * Else if funnel is barely in collision, try to shift it using snopt
 * If nothing collision free is found, return one with least penetration
 * (except if this one is also deep in collision)
 * If nothing satisfactory is found, use failsafe (return nextFunnel = 0)
 *
 * Inputs:
 * x: current state
 * forest: cell array containing vertices of obstacles
 * funnelLibrary: struct containing funnel library
 * funnelLibrary(*).xyz: xyz positions at time points (double 3 X N)
 * funnelLibrary(*).cS: cholesky factorization of projection of S matrix (cell array)
 *
 * Outputs:
 * Next funnel to be executed
 * Where to execute it from (only the cyclic coordinates)
 *
 * Author: Anirudha Majumdar
 * Date: March 2015
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

// Snopt stuff
namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
//#include "snoptProblem.hh"
}

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
// #include <iostream>

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "LinearMath/btTransformUtil.h"

using namespace std;


// Make funnel library, funnelIdx, x_current, forest global variables (with
// scope only in this file) because we need to be able to access these from
// the snopt user functions. :(
static const mxArray *funnelLibrary; // Funnel library (pointer)
static int funnelIdx;
static  const mxArray *x_current;
static double *dx_current;
static const mxArray *forest; // Cell array containing forest (pointer)
static double min_dist_snopt;


// Set solvers for bullet
static btVoronoiSimplexSolver sGjkSimplexSolver;
static btGjkEpaPenetrationDepthSolver epaSolver;

/*Sphere of radius r representing the point.
 * We could probably make the radius 0 and be ok, but I'm not sure if bullet expects things to be non-degenrate.
 */
static const double g_radius = 1.0;
static btSphereShape* g_point = new btSphereShape(g_radius);


/******************* Utility functions ***********************************/

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
    /* This can and maybe should be sped up. For example, we know that cSk is upper triangular. ***TIME****/
    double *dcSk = mxGetPr(cSk);
    double *dx0 = mxGetPr(x0);
    double *dx = mxGetPr(x);
    
    for(int i=0;i<nRows;i++){
        for(int j=0;j<nCols;j++){
            vertsT[j*nRows+i] = dcSk[i]*(verts[j*nRows+0]-dx0[k*nRows+0]-dx[0]) + dcSk[nRows+i]*(verts[j*nRows+1]-dx0[k*nRows+1]-dx[1]) + dcSk[2*nRows+i]*(verts[j*nRows+2]-dx0[k*nRows+2]-dx[2]);
            // mexPrintf("i: %d, j: %d, val: %f\n", i, j, vertsT[j*nRows+i]);
        }
    }
    
    
    return vertsT;
    
}

/******************************************************************************/




/****************************** Snopt functions for shifting funnels *********************************************************************************************/
/* Constraint to make sure current state is in inlet of shifted funnel
 */
double containmentConstraint(snopt::doublereal x_shift[], double *containment_grad)

{
    
    // Initialize some variables
    mxArray *x0 = mxGetField(funnelLibrary, funnelIdx, "x0"); // all points on trajectory
    
    double *dx0 = mxGetPr(x0);
    // double *dx_current = mxGetPr(x_current);
    
    long int dim = mxGetM(x_current); // Dimension of state
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
    
    
// Get x - x0(:,1) (but zero out x,y,z)
    mxArray *xrel = mxCreateDoubleMatrix(dim,1,mxREAL);
    double *dxrel = mxGetPr(xrel);
    /*dxrel[0] = 0.0;
     * dxrel[1] = 0.0;
     * dxrel[2] = 0.0;*/
    
// Set the non-cyclic dimensions to be the difference of x0 and x_current
    for(int k=3;k<dim;k++)
    {
        dxrel[k] = dx0[k] - dx_current[k];
    }
    
// Set the cyclic dimensions to be those of x_shift
    for(int k=0;k<3;k++)
    {
        dxrel[k] = x_shift[k];
    }
    
// Now compute xrel'*S0*xrel using lapack
// First do S0*xrel
    double one = 1.0, zero = 0.0; // Seriously?
    long int ione = 1;
    mxArray *S0xrel = mxCreateDoubleMatrix(dim,1,mxREAL);
    double *dS0xrel = mxGetPr(S0xrel);
    char *chn = "N";
    dgemm(chn, chn, &dim, &ione, &dim, &one, S0, &dim, dxrel, &dim, &zero, dS0xrel, &dim);
    
    // Use this to compute gradient: First 3 elements of 2*S0*xrel
    containment_grad[0] = 2*dS0xrel[0];
    containment_grad[1] = 2*dS0xrel[1];
    containment_grad[2] = 2*dS0xrel[2];
    
    
    
// Now do xrel'*S0xrel
    mxArray *val = mxCreateDoubleScalar(mxREAL);
    double *dval = mxGetPr(val);
    char *chnT = "T"; // since we want xrel transpose
    dgemm(chnT, chn, &ione, &ione, &dim, &one, dxrel, &dim, dS0xrel, &dim, &zero, dval, &ione);
    
    /*mexPrintf("\nval: %f\n", dx_current[0]);
     * mexPrintf("val: %f\n", dx_current[1]);
     * mexPrintf("val: %f\n", dx_current[2]);*/
    
// Return dval
    return *dval;
    
}

/* Penetration cost */
/* Computes f and df for penetration. f is > 1 iff there is no penetration. Also returns a boolean which is true if
 * the funnel is collision free and false otherwise
 */
bool penetrationCost(snopt::doublereal x[], double *min_dist, double *normal_vec_transformed)
{
    
    // Convert snopt doublereal to mex array so we can call shift and transform function.
    mxArray *x_shifted = mxCreateDoubleMatrix(3,1,mxREAL);
    double *dx_shifted = mxGetPr(x_shifted);
    
    dx_shifted[0] = x[0] + dx_current[0]; // Get shifted position of funnel
    dx_shifted[1] = x[1] + dx_current[1];
    dx_shifted[2] = x[2] + dx_current[2];
    
    
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
    
    
// Get number of time samples
    mwSize N = mxGetNumberOfElements(mxGetField(funnelLibrary, funnelIdx, "cS"));
    
// Check size compatibilities
    const mwSize *N_x0 = mxGetDimensions(x0);
    if (N_x0[1] != N) {
        mexErrMsgTxt("Sizes of x0 and cS do not match!");
    }
    
    
// Initialize collFree to true
    bool collFree = true;
    
    
// For each time sample, we need to check if we are collision free
// mxArray *collisions = mxCreateLogicalMatrix(numObs,N);
    
    for(int k=0;k<N;k++)
    {
        // Get pointer to cholesky factorization of S at this time
        cSk = mxGetCell(cS,k);
        
        for(mwIndex jForest=0;jForest<numObs;jForest++)
        {
            
            // Get vertices of this obstacle
            obstacle = mxGetCell(forest, jForest);
            verts = mxGetPr(mxGetCell(forest, jForest)); // Get vertices
            nCols = mxGetN(obstacle);
            nRows = mxGetM(obstacle);
            
            double *vertsT = mxGetPr(mxCreateDoubleMatrix(nRows, nCols, mxREAL));
            
            // Shift vertices so that point on trajectory is at origin and transform by cholesky of S
            vertsT = shiftAndTransform(verts, vertsT, x_shifted, x0, k, cSk, nRows, nCols);
            
            // Call bullet to find f (distance) and df (normal_vec)
            distance = ptToPolyBullet(vertsT, nRows, nCols, normal_vec);
            
            // Update min_dist
            if(distance < *min_dist){
                *min_dist = distance;
                
                // Multiply normal_vec by cSk to get it back in the correct coordinate frame (i.e., normal_vec'*cSk)
                double one = 1.0, zero = 0.0; // Seriously?
                long int ione = 1;
                long int dim = 3;
                
                char *chn = "N";
                dgemm(chn, chn, &ione, &dim, &dim, &one, mxGetPr(normal_vec), &ione, mxGetPr(cSk), &dim, &zero, normal_vec_transformed, &ione);
                
            }
            
            
        }
    }
    
    if(*min_dist < 1){
        collFree = false;
    }
    
    return collFree;
    
}





int snopt_userfun( snopt::integer    *Status, snopt::integer *n,    snopt::doublereal x[],
        snopt::integer    *needF,  snopt::integer *neF,  snopt::doublereal F[],
        snopt::integer    *needG,  snopt::integer *neG,  snopt::doublereal G[],
        char       *cu,     snopt::integer *lencu,
        snopt::integer    iu[],    snopt::integer *leniu,
        snopt::doublereal ru[],    snopt::integer *lenru )
{
    //==================================================================
    // Computes the nonlinear objective and constraint terms for the problem
    // of shifting a funnel to bring it out of collision.
    // neF = 2, n = 3.
    //
    //   Minimize (penetration - penetration_ideal)^2
    //
    //   subject to   x_current belongs to inlet of funnel
    //
    // Here, we're trying to push the funnel out of collision, but not too
    // far out. The decision variable is the amount to shift the funnel by
    // (and NOT the shiftED position of the funnel).
    //
    // The triples (g(k),iGfun(k),jGvar(k)), k = 1:neG, define
    // the sparsity pattern and values of the nonlinear elements
    // of the Jacobian.
    //==================================================================
    
    
    
    /*mexPrintf("\nx[0]: %f \n", x[0]);
     * mexPrintf("x[1]: %f \n", x[1]);
     * mexPrintf("x[2]: %f \n", x[2]);*/
    
    
    // Penetration cost and gradient
    double min_dist = 1000000.0;
    mxArray *normal_vec = mxCreateDoubleMatrix(1,3,mxREAL);
    double *normal_vec_d = mxGetPr(normal_vec);
    penetrationCost(x, &min_dist, normal_vec_d);
    
    min_dist_snopt = min_dist;
    
    // Ideal min_dist (should be greater than 1)
    double min_dist_ideal = 1.1;
    
    F[0] = min_dist*(min_dist-2.0*min_dist_ideal); // We're ignoring a constant term here
    
    if (*needG > 0)
    {
        // Penetration gradient
        G[0] = 2.0*min_dist*normal_vec_d[0] - 2.0*min_dist_ideal*normal_vec_d[0];
        G[1] = 2.0*min_dist*normal_vec_d[1] - 2.0*min_dist_ideal*normal_vec_d[1];
        G[2] = 2.0*min_dist*normal_vec_d[2] - 2.0*min_dist_ideal*normal_vec_d[2];
    }
    
    
    // Containment cost and gradient
    mxArray *containment_grad = mxCreateDoubleMatrix(3,1,mxREAL);
    double *containment_grad_d = mxGetPr(containment_grad);
    F[1] = containmentConstraint(x, containment_grad_d);
    
    if (*needG > 0)
    {
        // Containment gradient
        G[3] = containment_grad_d[0];
        G[4] = containment_grad_d[1];
        G[5] = containment_grad_d[2];
    }
    
    
    /*mexPrintf("F[0]: %f \n", F[0]);
     * mexPrintf("F[1]: %f \n", F[1]);
     *
     * mexPrintf("G[0]: %f \n", G[0]);
     * mexPrintf("G[1]: %f \n", G[1]);
     * mexPrintf("G[2]: %f \n", G[2]);
     * mexPrintf("G[3]: %f \n", G[3]);
     * mexPrintf("G[4]: %f \n", G[4]);
     * mexPrintf("G[5]: %f \n", G[5]);*/
    
    
    
    return 0;
}

// Shift funnel using snopt
bool shiftFunnel_snopt(int funnelIdx, const mxArray *funnelLibrary, const mxArray *forest, mwSize numObs, double *min_dist, double *x_opt)
{
    
// Number of decision variables (3 in our case: we're searching for shifted x,y,z)
    snopt::integer nx = 3;
    
// Number of rows of user constraint function (2 in our case).
// The first row is the collision "constraint" (really it's a cost)
// The second row is the containment constraint
    snopt::integer nF = 2;
    
// Cold start
    snopt::integer Cold = 0;
    
// Some silly stuff
    snopt::integer nxname = 1, nFname = 1, npname = 0;
    
// What to add to objective: 0 in our case
    snopt::doublereal ObjAdd = 0.0;
    
// Row in user function that corresponds to objective
    snopt::integer ObjRow = 1;
    
// Name of problem (used to print stuff)
    char Prob[200]="";
    
// iGfun, jGvar
    snopt::integer lenG = 6;
    
    snopt::integer*    iGfun = new snopt::integer[lenG];
    iGfun[0] = 1;
    iGfun[1] = 1;
    iGfun[2] = 1;
    iGfun[3] = 2;
    iGfun[4] = 2;
    iGfun[5] = 2;
    
    snopt::integer*    jGvar = new snopt::integer[lenG];
    jGvar[0] = 1;
    jGvar[1] = 2;
    jGvar[2] = 3;
    jGvar[3] = 1;
    jGvar[4] = 2;
    jGvar[5] = 3;
    
// Linear part of gradient (we don't have a linear part, so it's all zeros)
    snopt::integer lenA = 0;
    snopt::doublereal* A     = new snopt::doublereal[0];
    snopt::integer* iAfun = new snopt::integer[0];
    snopt::integer* jAvar = new snopt::integer[0];
    
// xlow, xupp, Flow, Fupp
    snopt::doublereal* xlow = new snopt::doublereal[nx];
    xlow[0] = -1.0/0.0;
    xlow[1] = -1.0/0.0;
    xlow[2] = -1.0/0.0;
    
    snopt::doublereal* xupp = new snopt::doublereal[nx];
    xupp[0] = 1.0/0.0;
    xupp[1] = 1.0/0.0;
    xupp[2] = 1.0/0.0;
    
    snopt::doublereal* Flow = new snopt::doublereal[nF];
    Flow[0] = -1.0/0.0;
    Flow[1] = -1.0/0.0;
    
    snopt::doublereal* Fupp = new snopt::doublereal[nF];
    Fupp[0] = 1.0/0.0;
    Fupp[1] = 1.0;
    
// xnames, Fnames
    snopt::integer INFO_snopt;
    char xnames[8*1];  // should match nxname
    char Fnames[8*1];  // should match nFname
    
// Initial guess for x_shift is 0.
    snopt::doublereal* x_guess    = new snopt::doublereal[nx];
    x_guess[0] = 0.0; // (*dx_current);
    x_guess[1] = 0.0; // (*(dx_current+1));
    x_guess[2] = 0.0; // (*(dx_current+2));
    
// xstate
    snopt::integer *xstate = new snopt::integer[nx];
    for(snopt::integer i = 0;i<nx;i++)
    {
        xstate[i] = 0;
    }
    
// xmul
    snopt::doublereal *xmul = new snopt::doublereal[nx];
    
// F, Fmul, Fstate
    snopt::doublereal *F      = new snopt::doublereal[nF];
    snopt::doublereal *Fmul   = new snopt::doublereal[nF];
    snopt::integer    *Fstate = new snopt::integer[nF];
    for(snopt::integer i = 0;i<nF;i++)
    {
        Fstate[i] = 0;
    }
    
// mincw, miniw, minrw
    snopt::integer minrw,miniw,mincw;
    
// nS, nInf, sInf
    snopt::integer nS,nInf;
    snopt::doublereal sInf;
    
// lenrw, leniw, lencw
    const int DEFAULT_LENRW = 500000;
    const int DEFAULT_LENIW = 500000;
    const int DEFAULT_LENCW = 500;
    
    snopt::integer lenrw = DEFAULT_LENRW, leniw = DEFAULT_LENIW, lencw = DEFAULT_LENCW;
    
// cw, iw, rw
    snopt::doublereal rw_static[DEFAULT_LENRW];
    snopt::integer iw_static[DEFAULT_LENIW];
    char cw_static[8*DEFAULT_LENCW];
    snopt::doublereal *rw = rw_static;
    snopt::integer *iw = iw_static;
    char* cw = cw_static;
    
// Memory allocation
    snopt::integer iSumm = -1;
    snopt::integer iPrint = -1;
    
    snopt::sninit_(&iPrint,&iSumm,cw,&lencw,iw,&leniw,rw,&lenrw,8*lencw);
    snopt::snmema_(&INFO_snopt,&nF,&nx,&nxname,&nFname,&lenA,&lenG,&mincw,&miniw,&minrw,cw,&lencw,iw,&leniw,rw,&lenrw,8*lencw);
    if (minrw>lenrw) {
        //mexPrintf("reallocation rw with size %d\n",minrw);
        lenrw = minrw;
        rw = new snopt::doublereal[lenrw];
    }
    if (miniw>leniw) {
        //mexPrintf("reallocation iw with size %d\n",miniw);
        leniw = miniw;
        iw = new snopt::integer[leniw];
    }
    if (mincw>lencw) {
        //mexPrintf("reallocation cw with size %d\n",mincw);
        lencw = mincw;
        cw = new char[8*lencw];
    }
    
    // snopt::snopenappend_(&iPrint,"ani",&INFO_snopt,3);
    
    
    snopt::sninit_(&iPrint,&iSumm,cw,&lencw,iw,&leniw,rw,&lenrw,8*lencw);
    
    snopt::snopta_
            ( &Cold, &nF, &nx, &nxname, &nFname,
            &ObjAdd, &ObjRow, Prob, snopt_userfun,
            iAfun, jAvar, &lenA, &lenA, A,
            iGfun, jGvar, &lenG, &lenG,
            xlow, xupp, xnames, Flow, Fupp, Fnames,
            x_guess, xstate, xmul, F, Fstate, Fmul,
            &INFO_snopt, &mincw, &miniw, &minrw,
            &nS, &nInf, &sInf,
            cw, &lencw, iw, &leniw, rw, &lenrw,
            cw, &lencw, iw, &leniw, rw, &lenrw,
            npname, 8*nxname, 8*nFname,
            8*lencw,8*lencw);
    
    snopt::snclose_(&iPrint);
    
    mexPrintf("Info: %d \n", INFO_snopt);
    
    // mexPrintf("F[0] %f \n", F[0]);
    
    // Start position of shifted funnel
    x_opt[0] = x_guess[0] + dx_current[0];
    x_opt[1] = x_guess[1] + dx_current[1];
    x_opt[2] = x_guess[2] + dx_current[2];
    
    // Delete stuff
    delete[] x_guess;
    delete[] xlow;
    delete[] xupp;
    delete[] Flow;
    delete[] Fupp;
    delete[] A;
    delete[] iAfun;
    delete[] jAvar;
    delete[] iGfun;
    delete[] jGvar;
    delete[] xmul;
    delete[] xstate;
    // delete[] F;
    delete[] Fmul;
    delete[] Fstate;
    
    if (rw != rw_static) { delete[] rw; }
    if (iw != iw_static) { delete[] iw; }
    if (cw != cw_static) { delete[] cw; }
    
    // Set min distance
    *min_dist = min_dist_snopt;
    
    if (*min_dist > 1.0) // Collision free
    {
        delete[] F;
        return true;
    }
    else
    {
        delete[] F;
        return false;
    }
    
    
    
}

/*************** Functions for funnel collision checking *******************************************/

/* Checks if a given funnel number funnelIdx is collision free if executed beginning at state x. Returns a boolean (true if collision free, false if not).
 */
bool isCollisionFree(int funnelIdx, const mxArray *x, const mxArray *funnelLibrary, const mxArray *forest, mwSize numObs, double *min_dist, bool setupQCQP_cons, double *A_ineq, double *b_ineq, int numRows_A, int maxObs)
{
    
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
    
// Initialize collFree to true
    bool collFree = true;
    
// For each time sample, we need to check if we are collision free
// mxArray *collisions = mxCreateLogicalMatrix(numObs,N);
    
    for(int k=0;k<N;k++)
    {
        
        // Get pointer to cholesky factorization of S at this time
        cSk = mxGetCell(cS,k);
        
        for(mwIndex jForest=0;jForest<numObs;jForest++)
        {
            
            // Get vertices of this obstacle
            obstacle = mxGetCell(forest, jForest);
            verts = mxGetPr(mxGetCell(forest, jForest)); // Get vertices
            nCols = mxGetN(obstacle);
            nRows = mxGetM(obstacle);
            
            double *vertsT = mxGetPr(mxCreateDoubleMatrix(nRows, nCols, mxREAL));
            
            // Shift vertices so that point on trajectory is at origin and transform by cholesky of S
            vertsT = shiftAndTransform(verts, vertsT, x, x0, k, cSk, nRows, nCols);
            
            // Call bullet to do point to polytope distance computation
            distance = ptToPolyBullet(vertsT, nRows, nCols, normal_vec);
            
            // Update min_dist
            if(distance < *min_dist){
                *min_dist = distance;
            }
            
            // mexPrintf("distance: %f\n", distance);
            
            // Check if distance is less than 1 (i.e. not collision free). If so, return
            //if(distance < 1){
            //collFree = false;
            // return collFree;
            //}
            
            
            /*************************************************************/
            // Setup linear constraints of QP while we're at it
            // (since we've already computed almost everything we need).
            
            if (setupQCQP_cons)
            {
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
            /*************************************************************/
            
            
        }
    }
    
    if (setupQCQP_cons)
    {
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
    
    if(*min_dist < 1){
        collFree = false;
    }
    
    return collFree;
    
}

/* Checks if a given state is inside the inlet of a funnel (after shifting things along cyclic coordinates)
 */
bool isInsideInlet(int funnelIdx, const mxArray *x, const mxArray *funnelLibrary)
{    
    
// Initialize some variables
    mxArray *x0 = mxGetField(funnelLibrary, funnelIdx, "x0"); // all points on trajectory
    
    double *dx0 = mxGetPr(x0);
    double *dx = mxGetPr(x);
    
    long int dim = mxGetM(x); // Dimension of state
    long int dimx0 = mxGetM(x0);
    
// Check that we got the right dimensions
    if(dim > 1){
        if (dim != dimx0){
            mexErrMsgTxt("x and x0 have different dimensions!");
        }
    }
    else{
        mexErrMsgTxt("State seems to be transposed");
    }
    
    
// Get S matrix at time 0
    mxArray *pS0 = mxGetField(funnelLibrary, funnelIdx, "S0");
    double *S0 = mxGetPr(pS0);
    
    
// Get x - x0(:,1) (but zero out x,y,z)
    mxArray *xrel = mxCreateDoubleMatrix(dim,1,mxREAL);
    double *dxrel = mxGetPr(xrel);
    dxrel[0] = 0.0;
    dxrel[1] = 0.0;
    dxrel[2] = 0.0;
    
    
    for(int k=3;k<dim;k++)
    {
        dxrel[k] = dx[k] - dx0[k];
    }
    
// Now compute xrel'*S0*xrel using lapack
// First do S0*xrel
    double one = 1.0, zero = 0.0; // Seriously?
    long int ione = 1;
    mxArray *S0xrel = mxCreateDoubleMatrix(dim,1,mxREAL);
    double *dS0xrel = mxGetPr(S0xrel);
    char *chn = "N";
    dgemm(chn, chn, &dim, &ione, &dim, &one, S0, &dim, dxrel, &dim, &zero, dS0xrel, &dim);
    
    
// Now do xrel'*S0xrel
    mxArray *val = mxCreateDoubleScalar(mxREAL);
    double *dval = mxGetPr(val);
    char *chnT = "T"; // since we want xrel transpose
    dgemm(chnT, chn, &ione, &ione, &dim, &one, dxrel, &dim, dS0xrel, &dim, &zero, dval, &ione);
    
// mexPrintf("val: %f\n", dval[0]);
    
// Now check if we're inside
    bool inside;
    if(dval[0] < 1.0)
    {
        inside = true;
    }
    else
    {
        inside = false;
    }
    
    return inside;
    
}

/*******************************************************************************************************/




/********** Functions for shifting funnels with QCQP ***************************************************/

/* Sets up quadratic part of constraint in QCQP
 */
void setupQuadraticConstraint(double *ql, double *qr)
{
    
    // Initialize some variables
    // mexPrintf("funnelIdx: %d \n", funnelIdx);
    mxArray *x0 = mxGetField(funnelLibrary, funnelIdx, "x0"); // all points on trajectory
    
    double *dx0 = mxGetPr(x0);
    // double *dx_current = mxGetPr(x_current);
    
    long int dim = mxGetM(x_current); // Dimension of state
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
    

    double minus_one = -1.0;
    *qr = 1.0;
    char *chnT = "T"; // since we want xrel transpose    
    dgemm(chnT, chn, &ione, &ione, &dimv, &minus_one, v_d, &dimv, dS22v, &dimv, &one, qr, &ione);
    
    // Now compute ql = 2*S12*v;
    double two = 2.0;
    long int ithree = 3;
    dgemm(chn, chn, &ithree, &ione, &dimv, &two, S12, &ithree, v_d, &dimv, &zero, ql, &ithree);
    
    
    
}


// Shift funnel using qcqp
bool shiftFunnelQCQP(mxArray *A_ineq_mx, mxArray *b_ineq_mx, double *x_opt)
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
     *  Returns the shiftted position of funnel.
     * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    
    double *A_ineq = mxGetPr(A_ineq_mx);
    double *b_ineq = mxGetPr(b_ineq_mx);
    
    // Linear constraints have already been set up. So, we just need to
    // setup the quadratic constraints
    mxArray *ql = mxCreateDoubleMatrix(3,1,mxREAL);
    double *ql_d = mxGetPr(ql);
    double qr;
    setupQuadraticConstraint(ql_d,&qr);
    
    /*mexPrintf("qr: %f \n", qr);
     * mexPrintf("ql[0]: %f \n", ql_d[0]);
     * mexPrintf("ql[1]: %f \n", ql_d[1]);
     * mexPrintf("ql[2]: %f \n", ql_d[2]);*/
    
    // Now create problem struct to pass to forces solver mex code.
    const char *field_names[] = {"A", "b", "ql", "qr"};
    mwSize ndim = 1;
    const mwSize ndims = 1;
    mxArray *problem = mxCreateStructArray(ndim,&ndims,4,field_names);
    mxSetField(problem, 0, "A", A_ineq_mx);
    mxSetField(problem, 0, "b", b_ineq_mx);
    mxSetField(problem, 0, "ql", ql);
    mxSetField(problem, 0, "qr", mxCreateDoubleScalar(qr));
    
    // Initialize result struct
    mxArray *lhs_forces[2];
    mxArray *result;
    lhs_forces[0] = result;
    lhs_forces[1] = mxCreateDoubleScalar(0.0); // exitflag of solver
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
        mexCallMATLAB(2, lhs_forces, 1, forces_rhs, function_name_char);
    }
    else
    {
        char function_name[14];
        snprintf(function_name, sizeof function_name, "%s%d%s", "funnel", funnelIdx+1, "_shift");
        const char *function_name_char = &function_name[0];
        // mexCallMATLAB(1, lhs_forces, 1, forces_rhs, "funnel1_shift");
        mexCallMATLAB(2, lhs_forces, 1, forces_rhs, function_name_char);
    }
    
    // Get exit flag
    double *exitflag;
    exitflag = mxGetPr(lhs_forces[1]);
    // mexPrintf("exitflag: %f \n", *exitflag);
    bool collFree = false;
    
    if (*exitflag != 1.0)
    {
        mexPrintf("Forces Pro exitflag was not 1. \n");
        
        // If exitflag is not 1, we don't trust the answer
        collFree = false;
        
        // Assign to x_current
        x_opt[0] = dx_current[0];
        x_opt[1] = dx_current[1];
        x_opt[2] = dx_current[2];
        
    }
    else
    {
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
        
        // Assign to x_opt (position of shifted funnel)
        x_opt[0] = *x_opt_forces_d + dx_current[0];
        x_opt[1] = *(x_opt_forces_d+1) + dx_current[1];
        x_opt[2] = *(x_opt_forces_d+2) + dx_current[2];
        
        // Check if we are collision free (tau <= 1e-4. well, really tau == 0,
        // but there will be numerical crap from the solver.)
        if (*tau_d < 1e-4)
        {
            collFree = true;
        }
    }
    
    // Destroy struct and other stuff we don't need
    mxDestroyArray(*lhs_forces);
    mxDestroyArray(*forces_rhs);
    // mxDestroyArray(result);
    
    
    
    // Return collFree
    return collFree;
    
}
/*******************************************************************************************************/






/************************ Main mex function ***************************************************************/
/* Main mex funtion*/
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    
    // Choose whether to use snopt or QCQP for shifting. Default is no shift;
    const char *shift_method;
    bool shift_using_qcqp = false;
    bool shift_using_snopt = false;
    
    // Shift later. If this is set to true, we go through all the funnels without
    // trying to shift them first. If we don't find something collision free, we try
    // to shift stuff. If this is false, we try to shift any promising funnels on the
    // first pass.
    bool shift_later = false;
    double *shift_later_ptr;
    
// Penetraion threshold: used to decide if we should try shifting funnel.
// Should be less than 1.0.
    double *penetration_thresh_ptr;
    double penetration_thresh = 0.75; // Default
    
// Failsafe penetration threshold: used to decide if we should revert to
// failsafe. Setting this to 1.0 means that we never take risks and revert
// to the failsafe any time we can't find a collision free funnel.
    double *failsafe_penetration_ptr;
    double failsafe_penetration = 1.0; // Default
    
// Get options if we are passed them
    if (nrhs > 3)
    {
        const mxArray *options; // Options structure
        options = prhs[3];
        
        // Check if shift_method option exists. If so, use it.
        mxArray *shift_method_mx = mxGetField(options, 0, "shift_method");
        if (shift_method_mx != NULL){
            shift_method = mxArrayToString(shift_method_mx);
            
            if (strcmp(shift_method, "qcqp") == 0) // if qcqp
            {
                shift_using_qcqp = true;
            }
            
            if (strcmp(shift_method, "snopt") == 0) // if qcqp
            {
                shift_using_snopt = true;
            }
            
        }
        
        // Check if penetration_thresh field exists. If not, leave it at default.
        mxArray *penetration_thresh_mx = mxGetField(options, 0, "penetration_thresh");
        if (penetration_thresh_mx != NULL){
            penetration_thresh_ptr = mxGetPr(penetration_thresh_mx);
            penetration_thresh = *penetration_thresh_ptr;
        }
        
        // Check if failsafe_penetration field exists. If not, leave it at default.
        mxArray *failsafe_penetration_mx = mxGetField(options, 0, "failsafe_penetration");
        if (failsafe_penetration_mx != NULL){
            failsafe_penetration_ptr = mxGetPr(failsafe_penetration_mx);
            failsafe_penetration = *failsafe_penetration_ptr;
        }
        
        // Check if shift_later field exists. If not, leave it at default.
        mxArray *shift_later_mx = mxGetField(options, 0, "shift_later");
        if (shift_later_mx != NULL){
            // shift_later_ptr = mxGetPr(shift_later_mx);
            // shift_later = *shift_later_mx;
            shift_later = mxGetLogicals(shift_later_mx);
        }
        
    }
    
// mexPrintf("penetration_thresh: %f \n", penetration_thresh);
// mexPrintf("failsafe_penetration: %f \n", failsafe_penetration);
// mexPrintf("shift_later: %i \n", shift_later);
    
    
// Get current x (state)
    const mxArray *x;
    x = prhs[0];
    
// Set current state (from which nominal/unshifted funnel would be executed)
    x_current = x;
    dx_current = mxGetPr(x_current);
    
// Get state from which nextFunnel should be executed
// We only care about the cyclic dimensions.
// Set it to x_current to begin with.
    mxArray *x_execute_next;
    x_execute_next = mxCreateDoubleMatrix(3,1,mxREAL);
    double *dx_execute_next = mxGetPr(x_execute_next);
    dx_execute_next[0] = (*dx_current);
    dx_execute_next[1] = (*(dx_current+1));
    dx_execute_next[2] = (*(dx_current+2));
    
    // mexPrintf("x_e_n %f \n", dx_execute_next[0]);
    
// Deal with forest cell (second input)
    forest = prhs[1]; // Get forest cell (second input)
    mwSize numObs = mxGetNumberOfElements(forest); // Number of obstacles
    
    // Maximum number of obstacles we can handle (for QCQP)
    int maxObs = 30;
    if (numObs > maxObs)
    {
        // Error
        mexErrMsgTxt("Number of obstacles is greater than maximum obstacles we can handle (30). If you want more obstacles, regenerate forces code and change maxObs above");
    }
    
    
// Now deal with funnel library object (third input)
    funnelLibrary = prhs[2]; // Get funnel library (third input)
    mwSize numFunnels = mxGetNumberOfElements(funnelLibrary);
    
// Initialize next funnel (output of this function)
    int nextFunnel = -1;
    bool collFree = false;
    bool inside;
    
// Initialize best penetration. Bigger is better.
    double best_penetration = -1000000.0;
    
    // Whether to setup QCQP constraints or not in isCollisionFree
    bool setupQCQP_cons = true;
    
    // Initialize array to keep track of penetrations
    mxArray *penetrations_array_mx = mxCreateDoubleMatrix(numFunnels,1,mxREAL);
    double *penetrations_array_d = mxGetPr(penetrations_array_mx);
    
    
// Now, try and find collision free funnel
    for(int ii=0;ii<numFunnels;ii++)
    // for(int funnelIdx=0;funnelIdx<numFunnels;funnelIdx++)
    {
        
        funnelIdx = ii; // Funnel idx is global so we have access to it in some other functions
        
        // First, check that we're inside the inlet of funnel
        inside = isInsideInlet(funnelIdx, x, funnelLibrary);
        
        // If we're not inside the funnel, continue (don't do collision checking)
        if(inside != true){
            continue;
        }
        
        // Initialize penetration.
        double penetration = 1000000.0;
        
        // If we are inside this funnel, do collision checking
        // Also returns (updates) penetration variable. If this is greater than 1, then the funnel is collision free
        // If penetration is less than one, than funnel is not collision free
        // Initialize linear constraints of QCQP
        // Get number of time samples
        mwSize N = mxGetNumberOfElements(mxGetField(funnelLibrary, funnelIdx, "cS"));
        int numRows_A = maxObs*N+1;
        mxArray *A_ineq_mx = mxCreateDoubleMatrix(numRows_A,4,mxREAL);
        double *A_ineq = mxGetPr(A_ineq_mx);
        mxArray *b_ineq_mx = mxCreateDoubleMatrix(numRows_A,1,mxREAL);
        double *b_ineq = mxGetPr(b_ineq_mx);
        setupQCQP_cons = true;
        collFree = isCollisionFree(funnelIdx, x, funnelLibrary, forest, numObs, &penetration, setupQCQP_cons, A_ineq, b_ineq, numRows_A, maxObs);
        
        // Save penetration in array
        penetrations_array_d[funnelIdx] = penetration;
        
        // Check if we have less penetration than previous funnels.
        // If so, make this the next funnel, but don't return yet (in the hope that we can find an even better funnel).
        if (penetration > best_penetration)
        {
            best_penetration = penetration;
            nextFunnel = funnelIdx;
            dx_execute_next[0] = (*dx_current);
            dx_execute_next[1] = (*(dx_current+1));
            dx_execute_next[2] = (*(dx_current+2));
        }
        
        
        // If this funnel is collision free, break out of the loop
        if (collFree == true){
            nextFunnel = funnelIdx;
            best_penetration = penetration;
            dx_execute_next[0] = (*dx_current);
            dx_execute_next[1] = (*(dx_current+1));
            dx_execute_next[2] = (*(dx_current+2));
            break;
        }
        
        // Optimal shift using QCQP
        mxArray *x_opt = mxCreateDoubleMatrix(3,1,mxREAL);
        double *x_opt_d = mxGetPr(x_opt);
        
        if ((shift_using_qcqp) && (penetration > penetration_thresh) && !shift_later)
        {
            
            // If the funnel is not collision free, but there is only little penetration, then let's try to shift the funnel
            collFree = shiftFunnelQCQP(A_ineq_mx, b_ineq_mx, x_opt_d);
            
            // See how much penetration we have with the shifted funnel
            penetration = 1000000.0;
            setupQCQP_cons = false;
            
            // Sometimes the collFree flag returned by QCQP is not correct (since it is only checking a sufficient condition).
            // This is why we do this check again. Is it worth it? ***TIME***
            collFree = isCollisionFree(funnelIdx, x_opt, funnelLibrary, forest, numObs, &penetration, setupQCQP_cons, A_ineq, b_ineq, numRows_A, maxObs);
            
            // If we were able to successfully shift the funnel out of collision, then return with this funnel
            if (collFree){
                nextFunnel = funnelIdx;
                best_penetration = penetration;
                dx_execute_next[0] = (*x_opt_d);
                dx_execute_next[1] = (*(x_opt_d+1));
                dx_execute_next[2] = (*(x_opt_d+2));
                break;
            }
            
            // If we didn't succeed in shifting it out of collision, check if we at least
            // have less penetration than previous funnels. If so, make this the next funnel,
            // but don't return yet (in the hope that we can find an even better funnel).
            if (penetration > best_penetration)
            {
                best_penetration = penetration;
                nextFunnel = funnelIdx;
                dx_execute_next[0] = (*x_opt_d);
                dx_execute_next[1] = (*(x_opt_d+1));
                dx_execute_next[2] = (*(x_opt_d+2));
            }
            
            
        }
        
        
        if ((shift_using_snopt) && (penetration > penetration_thresh) && !shift_later)
        {
            
            // If the funnel is not collision free, but there is only little penetration, then let's try to shift the funnel
            collFree = shiftFunnel_snopt(funnelIdx, funnelLibrary, forest, numObs, &penetration, x_opt_d);
            
            // If we were able to successfully shift the funnel out of collision, then return with this funnel
            if (collFree){
                nextFunnel = funnelIdx;
                best_penetration = penetration;
                dx_execute_next[0] = (*x_opt_d);
                dx_execute_next[1] = (*(x_opt_d+1));
                dx_execute_next[2] = (*(x_opt_d+2));
                break;
                
            }
            
            // If we didn't succeed in shifting it out of collision, check if we at least
            // have less penetration than previous funnels. If so, make this the next funnel,
            // but don't return yet (in the hope that we can find an even better funnel).
            if (penetration > best_penetration)
            {
                best_penetration = penetration;
                nextFunnel = funnelIdx;
                dx_execute_next[0] = (*x_opt_d);
                dx_execute_next[1] = (*(x_opt_d+1));
                dx_execute_next[2] = (*(x_opt_d+2));
            }
            
            
        }
        
        
        
    }
    
    /* for(int ii=0;ii<numFunnels;ii++)
    {
        mexPrintf("penetrations before sorting %d: %f \n", ii, penetrations_array_d[ii]);
    }*/
    
    
    
    // If we've reached this point with collFree = true, then we've found a collision
    // free funnel and we pass through. If however, we have shift_later = true, then
    // we haven't tried shifting funnels yet and we should try to shift them now.
    if (!collFree && shift_later)
    {
        
        // Sort penetrations array
        mxArray *Out[2];
        mxArray *In[3]; // we need three inputs to call matlab sort
        In[0] = penetrations_array_mx;
        In[1] = mxCreateDoubleScalar(1.0);
        In[2] = mxCreateString("descend");
        
        mexCallMATLAB(2, Out, 3, In, "sort");
        
        mxArray *sorted_inds_mx = mxCreateDoubleMatrix(numFunnels,1,mxREAL);
        sorted_inds_mx = Out[1];
        double *sorted_inds_d = mxGetPr(sorted_inds_mx);
        
        mxArray *penetrations_sorted_mx = mxCreateDoubleMatrix(numFunnels,1,mxREAL);
        penetrations_sorted_mx = Out[0];
        double *penetrations_sorted_d = mxGetPr(penetrations_sorted_mx);
        
        /*int ind_ii;
        for(int ii=0;ii<numFunnels;ii++)
        {
         ind_ii = sorted_inds_d[ii]-1; // Subtract 1 since we got this from matlab
         // mexPrintf("penetrations %d: %f \n", ind_ii, penetrations_sorted_d[ind_ii]);
        }*/
        
        for(int ii=0;ii<numFunnels;ii++)
        {
            
            // Next funnel: going through sorted funnels in sequence
            funnelIdx = sorted_inds_d[ii]-1;
            
            // First, check that we're inside the inlet of funnel
            inside = isInsideInlet(funnelIdx, x, funnelLibrary);
            
            // If we're not inside the funnel, continue (don't do collision checking)
            if(inside != true){
                continue;
            }
            
            // Initialize penetration.
            double penetration = 1000000.0;
            
            // If we are inside this funnel, do collision checking
            // Also returns (updates) penetration variable. If this is greater than 1, then the funnel is collision free
            // If penetration is less than one, than funnel is not collision free
            // Initialize linear constraints of QCQP
            // Get number of time samples
            mwSize N = mxGetNumberOfElements(mxGetField(funnelLibrary, funnelIdx, "cS"));
            int numRows_A = maxObs*N+1;
            mxArray *A_ineq_mx = mxCreateDoubleMatrix(numRows_A,4,mxREAL);
            double *A_ineq = mxGetPr(A_ineq_mx);
            mxArray *b_ineq_mx = mxCreateDoubleMatrix(numRows_A,1,mxREAL);
            double *b_ineq = mxGetPr(b_ineq_mx);
            setupQCQP_cons = true;
            collFree = isCollisionFree(funnelIdx, x, funnelLibrary, forest, numObs, &penetration, setupQCQP_cons, A_ineq, b_ineq, numRows_A, maxObs);
            
            
            // Optimal shift using QCQP
            mxArray *x_opt = mxCreateDoubleMatrix(3,1,mxREAL);
            double *x_opt_d = mxGetPr(x_opt);
            
            if ((shift_using_qcqp) && (penetration > penetration_thresh))
            {
               
                // If the funnel is not collision free, but there is only little penetration, then let's try to shift the funnel
                collFree = shiftFunnelQCQP(A_ineq_mx, b_ineq_mx, x_opt_d);
                
                // See how much penetration we have with the shifted funnel
                penetration = 1000000.0;
                setupQCQP_cons = false;
                
                // Sometimes the collFree flag returned by QCQP is not correct (since it is only checking a sufficient condition).
                // This is why we do this check again. Is it worth it? ***TIME***
                collFree = isCollisionFree(funnelIdx, x_opt, funnelLibrary, forest, numObs, &penetration, setupQCQP_cons, A_ineq, b_ineq, numRows_A, maxObs);
                
                // If we were able to successfully shift the funnel out of collision, then return with this funnel
                if (collFree){
                    nextFunnel = funnelIdx;
                    best_penetration = penetration;
                    dx_execute_next[0] = (*x_opt_d);
                    dx_execute_next[1] = (*(x_opt_d+1));
                    dx_execute_next[2] = (*(x_opt_d+2));
                    break;
                }
                
                // If we didn't succeed in shifting it out of collision, check if we at least
                // have less penetration than previous funnels. If so, make this the next funnel,
                // but don't return yet (in the hope that we can find an even better funnel).
                if (penetration > best_penetration)
                {
                    best_penetration = penetration;
                    nextFunnel = funnelIdx;
                    dx_execute_next[0] = (*x_opt_d);
                    dx_execute_next[1] = (*(x_opt_d+1));
                    dx_execute_next[2] = (*(x_opt_d+2));
                }
                
                
            }
            
            
            if ((shift_using_snopt) && (penetration > penetration_thresh))
            {
                
                // If the funnel is not collision free, but there is only little penetration, then let's try to shift the funnel
                collFree = shiftFunnel_snopt(funnelIdx, funnelLibrary, forest, numObs, &penetration, x_opt_d);
                
                // If we were able to successfully shift the funnel out of collision, then return with this funnel
                if (collFree){
                    nextFunnel = funnelIdx;
                    best_penetration = penetration;
                    dx_execute_next[0] = (*x_opt_d);
                    dx_execute_next[1] = (*(x_opt_d+1));
                    dx_execute_next[2] = (*(x_opt_d+2));
                    break;
                    
                }
                
                // If we didn't succeed in shifting it out of collision, check if we at least
                // have less penetration than previous funnels. If so, make this the next funnel,
                // but don't return yet (in the hope that we can find an even better funnel).
                if (penetration > best_penetration)
                {
                    best_penetration = penetration;
                    nextFunnel = funnelIdx;
                    dx_execute_next[0] = (*x_opt_d);
                    dx_execute_next[1] = (*(x_opt_d+1));
                    dx_execute_next[2] = (*(x_opt_d+2));
                }
                
                
            }
            
            
            
        }
        
    }
    
    
    
// If we've reached this point with collFree = true, then we've found a collision
// free funnel (possibly after shifting) and we pass through.
// If collFree = false, then we haven't been able to do so. We need to
// decide if the best funnel we've found has little enough penetration
// that it is worth risking executing it. If the best funnel has too much penetration,
// then the situation is hopeless and we revert to our failsafe (funnelIdx = -1, or 0 in Matlab).
    if (!collFree)
    {
        // If best penetration is too low, we revert to failsafe
        if (best_penetration <  failsafe_penetration)
        {
            nextFunnel = -1;
            
        }
    }
    
// Return next funnel index (add 1 since this will be used in Matlab)
    plhs[0] = mxCreateDoubleScalar(nextFunnel+1);
    
// Return x_execute if asked for
    if (nlhs > 1){
        plhs[1] =  x_execute_next;
    }
    
    // Return whether we were able to find a collision free funnel
    if (nlhs > 2){
        plhs[2] =   mxCreateLogicalScalar(collFree);
    }
    
    // Return penetration of best funnel if asked for
    if (nlhs > 3){
        plhs[3] =  mxCreateDoubleScalar(best_penetration);
    }
    
    
    
    return;
}
















