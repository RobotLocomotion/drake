/* Tries to shift a funnel using snopt to bring it out of collision
 *
 * Inputs:
 * x: current state
 * obstacles: cell array containing vertices of obstacles
 * funnelLibrary: struct containing funnel library
 * funnelLibrary(*).xyz: xyz positions at time points (double 3 X N)
 * funnelLibrary(*).cS: cholesky factorization of projection of S matrix (cell
 *array)
 * funnelIdx: which funnel in library to shift
 *
 * Outputs:
 * collFree: whether funnel is collision free after shift
 * Start position of shifted funnel
 * penetration distance after shift
 *
 * Author: Anirudha Majumdar
 * Date: March 14, 2015.
 */

// Mex stuff
#include <mex.h>
#include <matrix.h>
#include <blas.h>

#include <math.h>
#include "drake/matlab/util/drakeMexUtil.h"
#include <memory>
#include <algorithm>

// Snopt stuff
namespace snopt {
#include "snopt.hh"
#include "snfilewrapper.hh"
}

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
 * We could probably make the radius 0 and be ok, but I'm not sure if bullet
 * expects things to be non-degenrate.
 */
static const double g_radius = 1.0;
static btSphereShape *g_point = new btSphereShape(g_radius);

// Make funnel library, funnelIdx, x_current, obstacles global variables
static const mxArray *funnelLibrary;  // Funnel library (pointer)
static int funnelIdx;
static const mxArray *x_current;
static double *dx_current;
static const mxArray *obstacles;  // Cell array containing obstacles (pointer)
static double min_dist_snopt;

// lenrw, leniw, lencw
static unique_ptr<snopt::doublereal[]> rw;
static unique_ptr<snopt::integer[]> iw;
static unique_ptr<char[]> cw;
static snopt::integer lenrw = 0;
static snopt::integer leniw = 0;
static snopt::integer lencw = 0;

const int DEFAULT_LENRW = 500000;
const int DEFAULT_LENIW = 500000;
const int DEFAULT_LENCW = 500;

/* Constraint to make sure current state is in inlet of shifted funnel
 */
double containmentConstraint(snopt::doublereal x_shift[],
                             double *containment_grad) {
  // Initialize some variables
  mxArray *x0 =
      mxGetField(funnelLibrary, funnelIdx, "x0");  // all points on trajectory

  double *dx0 = mxGetPrSafe(x0);

  long int dim = 12;  // NOLINT(runtime/int)
  long int dimx0 = mxGetM(x0);  // NOLINT(runtime/int)

  // Check that we got the right dimensions
  if (dim > 1) {
    if (dim != dimx0) {
      mexErrMsgTxt("x and x0 have different dimensions!");
    }
  } else {
    mexErrMsgTxt("State seems to have the wrong dimension");
  }

  // Get S matrix at time 0
  mxArray *pS0 = mxGetField(funnelLibrary, funnelIdx, "S0");
  double *S0 = mxGetPrSafe(pS0);

  // Get x - x0(:, 1) (but zero out x, y, z)
  mxArray *xrel = mxCreateDoubleMatrix(dim, 1, mxREAL);
  double *dxrel = mxGetPrSafe(xrel);

  // Set the non-cyclic dimensions to be the difference of x0 and x_current
  for (int k = 3; k < dim; k++) {
    dxrel[k] = dx0[k] - dx_current[k];
  }

  // Set the cyclic dimensions to be those of x_shift
  for (int k = 0; k < 3; k++) {
    dxrel[k] = x_shift[k];
  }

  // Now compute xrel'*S0*xrel using lapack
  // First do S0*xrel
  double one = 1.0, zero = 0.0;  // Seriously?
  long int ione = 1;  // NOLINT(runtime/int)
  mxArray *S0xrel = mxCreateDoubleMatrix(dim, 1, mxREAL);
  double *dS0xrel = mxGetPrSafe(S0xrel);
  char chn[] = "N";
  dgemm(chn, chn, &dim, &ione, &dim, &one, S0, &dim, dxrel, &dim, &zero,
        dS0xrel, &dim);

  // Use this to compute gradient: First 3 elements of 2*S0*xrel
  containment_grad[0] = 2 * dS0xrel[0];
  containment_grad[1] = 2 * dS0xrel[1];
  containment_grad[2] = 2 * dS0xrel[2];

  // Now do xrel'*S0xrel
  mxArray *val = mxCreateDoubleScalar(mxREAL);
  double *dval = mxGetPrSafe(val);
  char chnT[] = "T";  // since we want xrel transpose
  dgemm(chnT, chn, &ione, &ione, &dim, &one, dxrel, &dim, dS0xrel, &dim, &zero,
        dval, &ione);

  // Return dval
  return *dval;
}

double ptToPolyBullet(double *vertsPr, size_t nRows, size_t nCols,
                      mxArray *normal_vec) {
  // Initialize polytope object with single point
  btConvexHullShape polytope(btVector3(vertsPr[0], vertsPr[1], vertsPr[2]), 1);

  // Add rest of the points (note the indexing starts from 1 on the loop)
  for (size_t i = 1; i < nCols; i++) {
    polytope.addPoint(btVector3(vertsPr[i * nRows], vertsPr[i * nRows + 1],
                                vertsPr[i * nRows + 2]));
  }

  // Assign elements of verts (input) to polytope
  btTransform tr;
  btGjkPairDetector::ClosestPointInput input;
  tr.setIdentity();
  input.m_transformA = tr;
  input.m_transformB = tr;

  btGjkPairDetector convexConvex(g_point, &polytope, &sGjkSimplexSolver,
                                 &epaSolver);

  // Output
  btPointCollector gjkOutput;

  convexConvex.getClosestPoints(input, gjkOutput, 0);
  double *normal_vec_d = mxGetPrSafe(normal_vec);

  normal_vec_d[0] = gjkOutput.m_normalOnBInWorld[0];
  normal_vec_d[1] = gjkOutput.m_normalOnBInWorld[1];
  normal_vec_d[2] = gjkOutput.m_normalOnBInWorld[2];

  return gjkOutput.m_distance + CONVEX_DISTANCE_MARGIN + g_radius;
}

/* Shift and transform vertices
 */
double *shiftAndTransform(double *verts, double *vertsT, const mxArray *x,
                          mxArray *x0, int k, mxArray *cSk, size_t nRows,
                          size_t nCols) {
  /* This can and maybe should be sped up. For example, we know that cSk is
   * upper triangular.*/
  double *dcSk = mxGetPrSafe(cSk);
  double *dx0 = mxGetPrSafe(x0);
  double *dx = mxGetPrSafe(x);

  for (size_t i = 0; i < nRows; i++) {
    for (size_t j = 0; j < nCols; j++) {
      vertsT[j * nRows + i] =
          dcSk[i] * (verts[j * nRows + 0] - dx0[k * nRows + 0] - dx[0]) +
          dcSk[nRows + i] *
              (verts[j * nRows + 1] - dx0[k * nRows + 1] - dx[1]) +
          dcSk[2 * nRows + i] *
              (verts[j * nRows + 2] - dx0[k * nRows + 2] - dx[2]);
    }
  }

  return vertsT;
}

/* Penetration cost */
/* Computes f and df for penetration. f is > 1 iff there is no penetration. Also
 * returns a boolean which is true if
 * the funnel is collision free and false otherwise
 */
bool penetrationCost(snopt::doublereal x[], double *min_dist,
                     double *normal_vec_transformed) {
  // Convert snopt doublereal to mex array so we can call shift and transform
  // function.
  mxArray *x_shifted = mxCreateDoubleMatrix(3, 1, mxREAL);
  double *dx_shifted = mxGetPrSafe(x_shifted);

  dx_shifted[0] = x[0] + dx_current[0];  // Get shifted position of funnel
  dx_shifted[1] = x[1] + dx_current[1];
  dx_shifted[2] = x[2] + dx_current[2];

  // Get number of obstacles
  mwSize numObs = mxGetNumberOfElements(obstacles);  // Number of obstacles

  // Initialize some variables
  double *verts;  // cell element (i.e. vertices)
  mxArray *x0 =
      mxGetField(funnelLibrary, funnelIdx, "xyz");  // all points on trajectory
  mxArray *obstacle;
  mxArray *cS = mxGetField(funnelLibrary, funnelIdx, "cS");
  mxArray *cSk;
  size_t nCols;
  size_t nRows;
  double distance;
  mxArray *normal_vec;
  normal_vec = mxCreateDoubleMatrix(1, 3, mxREAL);

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
  // mxArray *collisions = mxCreateLogicalMatrix(numObs, N);

  for (mwSize k = 0; k < N; k++) {
    // Get pointer to cholesky factorization of S at this time
    cSk = mxGetCell(cS, k);

    for (mwIndex obstacleIndex = 0; obstacleIndex < numObs; obstacleIndex++) {
      // Get vertices of this obstacle
      obstacle = mxGetCell(obstacles, obstacleIndex);
      verts = mxGetPrSafe(mxGetCell(obstacles, obstacleIndex));  // Get vertices
      nCols = mxGetN(obstacle);
      nRows = mxGetM(obstacle);

      double *vertsT = mxGetPrSafe(mxCreateDoubleMatrix(nRows, nCols, mxREAL));

      // Shift vertices so that point on trajectory is at origin and transform
      // by cholesky of S
      vertsT =
          shiftAndTransform(verts, vertsT, x_shifted, x0, k, cSk, nRows, nCols);

      // Call bullet to find f (distance) and df (normal_vec)
      distance = ptToPolyBullet(vertsT, nRows, nCols, normal_vec);

      // Update min_dist
      if (distance < *min_dist) {
        *min_dist = distance;

        // Multiply normal_vec by cSk to get it back in the correct coordinate
        // frame (i.e., normal_vec'*cSk)
        double one = 1.0, zero = 0.0;  // Seriously?
        long int ione = 1;  // NOLINT(runtime/int)
        long int dim = 3;  // NOLINT(runtime/int)
        char chn[] = "N";
        dgemm(chn, chn, &ione, &dim, &dim, &one, mxGetPrSafe(normal_vec), &ione,
              mxGetPrSafe(cSk), &dim, &zero, normal_vec_transformed, &ione);
      }
    }
  }

  if (*min_dist < 1) {
    collFree = false;
  }

  return collFree;
}

int snopt_userfun(snopt::integer *Status, snopt::integer *n,
                  snopt::doublereal x[], snopt::integer *needF,
                  snopt::integer *neF, snopt::doublereal F[],
                  snopt::integer *needG, snopt::integer *neG,
                  snopt::doublereal G[], char *cu, snopt::integer *lencu,
                  snopt::integer iu[], snopt::integer *leniu,
                  snopt::doublereal ru[], snopt::integer *lenru) {
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
  // The triples (g(k), iGfun(k), jGvar(k)), k = 1:neG, define
  // the sparsity pattern and values of the nonlinear elements
  // of the Jacobian.
  //==================================================================

  // Penetration cost and gradient
  double min_dist = 1000000.0;
  mxArray *normal_vec = mxCreateDoubleMatrix(1, 3, mxREAL);
  double *normal_vec_d = mxGetPrSafe(normal_vec);
  penetrationCost(x, &min_dist, normal_vec_d);

  min_dist_snopt = min_dist;

  // Ideal min_dist (should be greater than 1)
  double min_dist_ideal = 1.1;

  F[0] =
      min_dist *
      (min_dist - 2.0 * min_dist_ideal);  // We're ignoring a constant term here

  if (*needG > 0) {
    // Penetration gradient
    G[0] = 2.0 * min_dist * normal_vec_d[0] -
           2.0 * min_dist_ideal * normal_vec_d[0];
    G[1] = 2.0 * min_dist * normal_vec_d[1] -
           2.0 * min_dist_ideal * normal_vec_d[1];
    G[2] = 2.0 * min_dist * normal_vec_d[2] -
           2.0 * min_dist_ideal * normal_vec_d[2];
  }

  // Containment cost and gradient
  mxArray *containment_grad = mxCreateDoubleMatrix(3, 1, mxREAL);
  double *containment_grad_d = mxGetPrSafe(containment_grad);
  F[1] = containmentConstraint(x, containment_grad_d);

  if (*needG > 0) {
    // Containment gradient
    G[3] = containment_grad_d[0];
    G[4] = containment_grad_d[1];
    G[5] = containment_grad_d[2];
  }

  return 0;
}

// Shift funnel using snopt
bool shiftFunnel(int funnelIdx, const mxArray *funnelLibrary,
                 const mxArray *obstacles, mwSize numObs, double *min_dist,
                 double *x_opt) {
  // Number of decision variables (3 in our case: we're searching for shifted
  // x, y, z)
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
  char Prob[200] = "";

  // iGfun, jGvar
  snopt::integer lenG = 6;

  snopt::integer *iGfun = new snopt::integer[lenG];
  iGfun[0] = 1;
  iGfun[1] = 1;
  iGfun[2] = 1;
  iGfun[3] = 2;
  iGfun[4] = 2;
  iGfun[5] = 2;

  snopt::integer *jGvar = new snopt::integer[lenG];
  jGvar[0] = 1;
  jGvar[1] = 2;
  jGvar[2] = 3;
  jGvar[3] = 1;
  jGvar[4] = 2;
  jGvar[5] = 3;

  // Linear part of gradient (we don't have a linear part, so it's all zeros)
  snopt::integer lenA = 0;
  snopt::doublereal *A = new snopt::doublereal[0];
  snopt::integer *iAfun = new snopt::integer[0];
  snopt::integer *jAvar = new snopt::integer[0];

  // xlow, xupp, Flow, Fupp
  snopt::doublereal *xlow = new snopt::doublereal[nx];
  xlow[0] = -1.0 / 0.0;
  xlow[1] = -1.0 / 0.0;
  xlow[2] = -1.0 / 0.0;

  snopt::doublereal *xupp = new snopt::doublereal[nx];
  xupp[0] = 1.0 / 0.0;
  xupp[1] = 1.0 / 0.0;
  xupp[2] = 1.0 / 0.0;

  snopt::doublereal *Flow = new snopt::doublereal[nF];
  Flow[0] = -1.0 / 0.0;
  Flow[1] = -1.0 / 0.0;

  snopt::doublereal *Fupp = new snopt::doublereal[nF];
  Fupp[0] = 1.0 / 0.0;
  Fupp[1] = 1.0;

  // xnames, Fnames
  snopt::integer INFO_snopt;
  char xnames[8 * 1];  // should match nxname
  char Fnames[8 * 1];  // should match nFname

  // Initial guess for x_shift is 0.
  snopt::doublereal *x_guess = new snopt::doublereal[nx];
  x_guess[0] = 0.0;  // (*dx_current);
  x_guess[1] = 0.0;  // (*(dx_current+1));
  x_guess[2] = 0.0;  // (*(dx_current+2));

  // xstate
  snopt::integer *xstate = new snopt::integer[nx];
  for (snopt::integer i = 0; i < nx; i++) {
    xstate[i] = 0;
  }

  // xmul
  snopt::doublereal *xmul = new snopt::doublereal[nx];

  // F, Fmul, Fstate
  snopt::doublereal *F = new snopt::doublereal[nF];
  snopt::doublereal *Fmul = new snopt::doublereal[nF];
  snopt::integer *Fstate = new snopt::integer[nF];
  for (snopt::integer i = 0; i < nF; i++) {
    Fstate[i] = 0;
  }

  // mincw, miniw, minrw
  snopt::integer minrw, miniw, mincw;

  // nS, nInf, sInf
  snopt::integer nS, nInf;
  snopt::doublereal sInf;

  snopt::integer lenrw = DEFAULT_LENRW, leniw = DEFAULT_LENIW,
                 lencw = DEFAULT_LENCW;

  // Memory allocation
  snopt::integer iSumm = -1;   // 1
  snopt::integer iPrint = -1;  // 9

  snopt::sninit_(&iPrint, &iSumm, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, 8 * lencw);
  snopt::snmema_(&INFO_snopt, &nF, &nx, &nxname, &nFname, &lenA, &lenG, &mincw,
                 &miniw, &minrw, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, 8 * lencw);
  if (minrw > lenrw) {
    lenrw = minrw;
    rw.reset(new snopt::doublereal[lenrw]);
  }
  if (miniw > leniw) {
    leniw = miniw;
    iw.reset(new snopt::integer[leniw]);
  }
  if (mincw > lencw) {
    lencw = mincw;
    cw.reset(new char[8 * lencw]);
  }

  snopt::sninit_(&iPrint, &iSumm, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, 8 * lencw);

  snopt::snopta_(&Cold, &nF, &nx, &nxname, &nFname, &ObjAdd, &ObjRow, Prob,
                 snopt_userfun, iAfun, jAvar, &lenA, &lenA, A, iGfun, jGvar,
                 &lenG, &lenG, xlow, xupp, xnames, Flow, Fupp, Fnames, x_guess,
                 xstate, xmul, F, Fstate, Fmul, &INFO_snopt, &mincw, &miniw,
                 &minrw, &nS, &nInf, &sInf, cw.get(), &lencw, iw.get(), &leniw,
                 rw.get(), &lenrw, cw.get(), &lencw, iw.get(), &leniw, rw.get(),
                 &lenrw, npname, 8 * nxname, 8 * nFname, 8 * lencw, 8 * lencw);

  snopt::snclose_(&iPrint);

  mexPrintf("Info: %d \n", INFO_snopt);

  // Optimized value
  x_opt[0] = x_guess[0];
  x_opt[1] = x_guess[1];
  x_opt[2] = x_guess[2];

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
  delete[] Fmul;
  delete[] Fstate;

  // Set min distance
  *min_dist = min_dist_snopt;

  if (*min_dist > 1.0) {  // Collision free
    delete[] F;
    return true;
  } else {
    delete[] F;
    return false;
  }
}

/* Main mex funtion*/
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (lenrw == 0) {  // then initialize (sninit needs some default allocation)
    lenrw = DEFAULT_LENRW;
    rw.reset(new snopt::doublereal[lenrw]);
    leniw = DEFAULT_LENIW;
    iw.reset(new snopt::integer[leniw]);
    lencw = DEFAULT_LENCW;
    cw.reset(new char[8 * lencw]);
  }
  // Get current state (from which nominal/unshifted funnel would be executed)
  // const mxArray *x_current;
  x_current = prhs[0];
  dx_current = mxGetPrSafe(x_current);

  // Deal with obstacles cell (second input)
  obstacles = prhs[1];  // Get obstacles cell (second input)
  mwSize numObs = mxGetNumberOfElements(obstacles);  // Number of obstacles

  funnelLibrary = prhs[2];  // Get funnel library (third input)

  // Get funnelIdx (subtract 1 since index is coming from matlab)
  // int funnelIdx;
  funnelIdx = (int)mxGetScalar(prhs[3]);
  funnelIdx = funnelIdx - 1;

  // Initialize penetration (after shifting)
  double min_dist = 1000000.0;

  // Optimal shift from snopt
  mxArray *x_opt = mxCreateDoubleMatrix(3, 1, mxREAL);
  double *x_opt_d = mxGetPrSafe(x_opt);

  // Initialize next funnel (output of this function)
  bool collFree = shiftFunnel(funnelIdx, funnelLibrary, obstacles, numObs,
                              &min_dist, x_opt_d);

  // Add in x_current to get SHIFTED position of funnel
  x_opt_d[0] = x_opt_d[0] + dx_current[0];
  x_opt_d[1] = x_opt_d[1] + dx_current[1];
  x_opt_d[2] = x_opt_d[2] + dx_current[2];

  // Return collFree
  plhs[0] = mxCreateLogicalScalar(collFree);

  // Return x_opt if asked for
  if (nlhs > 1) {
    plhs[1] = x_opt;
  }

  // Return min_dist if asked for
  if (nlhs > 2) {
    plhs[2] = mxCreateDoubleScalar(min_dist);
  }

  return;
}
