#include <mex.h>
#include <cstring>
#include <climits>
#include <cmath>
#include <string>
#include <Eigen/Core>

#include "drakeUtil.h"

extern "C" void matlabQPSplineSolve(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

const int GRID_STEPS = 10;
const int MAX_NRHS = 2;
const int NUM_SPLINE_PARTS = 3;
const int Q_SIZE = NUM_SPLINE_PARTS - 1;

using namespace std;
using namespace Eigen;

Matrix<double, Q_SIZE, Q_SIZE> computeQ(double t0, double t1) {
  Matrix<double, Q_SIZE, Q_SIZE> Q;
  Q << std::pow(2.0, 2.0) * (t1 - t0), 6.0 * std::pow(t1 - t0, 2.0),
      6.0 * std::pow(t1 - t0, 2.0), std::pow(6.0, 2.0) / 3.0 * std::pow(t1 - t0, 3.0);
  return Q;
}

void setUpQPSplineSolveFields(double ts[NUM_SPLINE_PARTS + 1], mxArray* t1_mx, mxArray* t2_mx, mxArray* Q_mx[NUM_SPLINE_PARTS])
{
  *mxGetPr(t1_mx) = ts[1];
  *mxGetPr(t2_mx) = ts[2];
  for (int i = 0; i < NUM_SPLINE_PARTS; i++) {
    Map<Matrix<double, Q_SIZE, Q_SIZE> > Q(mxGetPr(Q_mx[i]));
    Q = computeQ(ts[i], ts[i + 1]);
  }
}

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > MAX_NRHS)
    mexErrMsgTxt("Too many arguments.\n");

  // get t0, tf
  double t0 = mxGetPrSafe(mxGetFieldSafe(prhs[0], 0, "t0"))[0];
  double tf = mxGetPrSafe(mxGetFieldSafe(prhs[0], 0, "tf"))[0];

  double ts[NUM_SPLINE_PARTS + 1];
  ts[0] = t0;
  for (int i = 1; i < NUM_SPLINE_PARTS; i++)
    ts[NUM_SPLINE_PARTS] = NAN;
  ts[NUM_SPLINE_PARTS] = tf;

  // copy right hand side since we're going to modify things
  mxArray* prhs_copy[MAX_NRHS];
  for (int i = 0; i < nrhs; i++) {
    prhs_copy[i] = mxDuplicateArray(prhs[i]);
  }

  // create fields for varying quantities
  double t_step = (tf - t0) / GRID_STEPS;

  mxArray* t1_mx = mxCreateDoubleScalar(0.0);
  mxSetFieldSafe(prhs_copy[0], 0, "t1", t1_mx);

  mxArray* t2_mx = mxCreateDoubleScalar(0.0);
  mxSetFieldSafe(prhs_copy[0], 0, "t2", t2_mx);

  mxArray* Q_mx[NUM_SPLINE_PARTS];
  for (int i = 0; i < NUM_SPLINE_PARTS; i++) {
    Q_mx[i] = mxCreateDoubleMatrix(Q_SIZE, Q_SIZE, mxREAL);
    mxSetFieldSafe(prhs_copy[0], 0, "Q" + to_string(i), Q_mx[i]);
  }

  // find optimal t1, t2
  double ts_opt[NUM_SPLINE_PARTS + 1];
  ts_opt[0] = t0;
  ts_opt[NUM_SPLINE_PARTS] = tf;
  double min_objective_value = numeric_limits<double>::infinity();

  for (int t1_index = 0; t1_index < GRID_STEPS; t1_index++) {
    ts[1] = t0 + t1_index * t_step;

    for (int t2_index = t1_index; t2_index < GRID_STEPS; t2_index++) {
      ts[2] = t0 + t2_index * t_step;

      setUpQPSplineSolveFields(ts, t1_mx, t2_mx, Q_mx);
      matlabQPSplineSolve(nlhs, plhs, nrhs, const_cast<const mxArray**>(prhs_copy));

      bool converged = (bool) mxGetPrSafe(mxGetFieldSafe(plhs[1], 0, "converged"))[0];
      if (converged) {
        double objective_value = mxGetPrSafe(mxGetFieldSafe(plhs[1], 0, "optval"))[0];
        if (objective_value < min_objective_value) {
          ts_opt[1] = ts[1];
          ts_opt[2] = ts[2];
          min_objective_value = objective_value;
        }
      }
    }
  }

  // call one last time at optimal value
  setUpQPSplineSolveFields(ts_opt, t1_mx, t2_mx, Q_mx);
  matlabQPSplineSolve(nlhs, plhs, nrhs, const_cast<const mxArray**>(prhs_copy));
  mxSetFieldSafe(plhs[0], 0, "t1", mxCreateDoubleScalar(ts_opt[1]));
  mxSetFieldSafe(plhs[0], 0, "t2", mxCreateDoubleScalar(ts_opt[2]));

  for (int i = 0; i < nrhs; i++) {
    mxDestroyArray(prhs_copy[i]);
  }
}
