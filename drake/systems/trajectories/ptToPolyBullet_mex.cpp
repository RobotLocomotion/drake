/* Finds the distance between the origin and a polytope (defined by its
vertices) using bullet's low level functions. Uses GJK and EPA algorithms.

Author: Anirudha Majumdar
Date: Nov 8 2013
*/

#include <mex.h>

/// We need internal access to bullet
// #include "GL_Simplex1to4.h"
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

#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;

// Set solvers
static btVoronoiSimplexSolver sGjkSimplexSolver;
static btGjkEpaPenetrationDepthSolver epaSolver;

/*Sphere of radius r representing the point.
We could probably make the radius 0 and be ok, but I'm not sure if bullet
expects things to be non-degenrate.
*/
static const double radius = 0.5;
static btSphereShape *point = new btSphereShape(radius);

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // Check for proper number of arguments.
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("Drake:ptToPolyDist:invalidNumInputs",
                      "One input required.");
  } else if (nlhs > 1) {
    mexErrMsgIdAndTxt("Drake:ptToPolyDist:maxlhs",
                      "Too many output arguments.");
  }

  // Get inputs
  double *vertsPr;
  vertsPr = mxGetPrSafe(prhs[0]);
  const int nRows = mxGetM(prhs[0]);  // number of rows
  const int nCols = mxGetN(prhs[0]);  // number of columns

  // Check to see if number of rows of vertices is 3
  if (nRows != 3) {
    mexErrMsgIdAndTxt("Drake:ptToPolyDist:invalidInputs",
                      "Input should be 3 x N");
  }

  // Initialize polytope object with single point
  btConvexHullShape polytope(btVector3(vertsPr[0], vertsPr[1], vertsPr[2]), 1);

  // Add rest of the points (note the indexing starts from 1 on the loop)
  for (int i = 1; i < nCols; i++) {
    polytope.addPoint(btVector3(vertsPr[i * nRows], vertsPr[i * nRows + 1],
                                vertsPr[i * nRows + 2]));

    /*
    mexPrintf("i: %d\n", i);
    mexPrintf("x: %f", vertsPr[i*nRows]);
    mexPrintf("y: %f", vertsPr[i*nRows+1]);
    mexPrintf("z: %f\n", vertsPr[i*nRows+2]);*/
  }

  // btConvexHullShape polytope(&points0[0].getX(), 6);

  // Assign elements of verts (input) to polytope
  btTransform tr;
  btGjkPairDetector::ClosestPointInput input;
  tr.setIdentity();
  input.m_transformA = tr;
  input.m_transformB = tr;

  btGjkPairDetector convexConvex(point, &polytope, &sGjkSimplexSolver,
                                 &epaSolver);

  // Output
  btPointCollector gjkOutput;

  convexConvex.getClosestPoints(input, gjkOutput, 0);

  // mexPrintf("Hello!\n");

  plhs[0] = mxCreateDoubleScalar(gjkOutput.m_distance + radius +
                                 CONVEX_DISTANCE_MARGIN);

  // cout << gjkOutput.m_distance + radius + CONVEX_DISTANCE_MARGIN << endl;
}
