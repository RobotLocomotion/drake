#include <mex.h>
#include <Eigen/Core>
#include "splineGeneration.h"
#include "drakeUtil.h"
#include <iostream>
#include <limits>

using namespace std;
using namespace Eigen;

const int GRID_STEPS = 10;

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {
  string usage = "[coefs, ts, objective_value] = nWaypointCubicSplineFreeKnotTimesmex.cpp(t0, tf, xs, xd0, xdf)";
  if (nrhs != 5)
    mexErrMsgIdAndTxt("Drake:nWaypointCubicSplineFreeKnotTimesmex.cpp:WrongNumberOfInputs", usage.c_str());
  if (nlhs < 2 || nlhs > 3)
    mexErrMsgIdAndTxt("Drake:nWaypointCubicSplineFreeKnotTimesmex.cpp:WrongNumberOfOutputs", usage.c_str());

  double t0 = mxGetPrSafe(prhs[0])[0];
  double tf = mxGetPrSafe(prhs[1])[0];
  MatrixXd xs = matlabToEigen<Dynamic, Dynamic>(prhs[2]);
  auto xd0 = matlabToEigen<Dynamic, 1>(prhs[3]);
  auto xdf = matlabToEigen<Dynamic, 1>(prhs[4]);

  mwSize ndof = static_cast<mwSize>(xs.rows());
  mwSize num_segments = static_cast<mwSize>(xs.cols())-1;
  mwSize num_knots = num_segments - 1;
  if (num_knots >= 3)
    mexWarnMsgTxt("More knots than two is likely to be super slow in a grid search!\n");
  mwSize num_coeffs_per_segment = 4;
  mwSize dims[] = {ndof, num_segments, num_coeffs_per_segment};
  plhs[0] = mxCreateNumericArray(num_segments, dims, mxDOUBLE_CLASS, mxREAL);

  std::vector<double> segment_times;
  segment_times.resize(static_cast<size_t>(num_segments) + 1);
  segment_times[0] = t0;
  segment_times[static_cast<size_t>(num_segments)] = tf;
  std::vector<double> best_segment_times = segment_times;
  double t_step = (tf - t0) / GRID_STEPS;
  double min_objective_value = numeric_limits<double>::infinity();

  // assemble the knot point locations for input to nWaypointCubicSpline
  std::vector<std::vector<double>> xi(ndof, std::vector<double>(num_knots));
  for (int dof = 0; dof < ndof; dof++)
    for (int knot = 0; knot < num_knots; knot++)
      xi[dof][knot] = xs(dof, knot);

  for (int t_superindex = 0; t_superindex < pow(GRID_STEPS, num_knots); t_superindex++){
    // Separate out our superindex over the entire grid search into 
    // the individual indices we're dealing with here
    // (we're functionally in base GRID_STEPS)
    for (int i=0; i < num_knots; i++){
      int t_index = (t_superindex / (int)pow(GRID_STEPS, i)) % GRID_STEPS;
      segment_times[i+1] = t0 + t_index * t_step;
    }

    bool valid_solution = true;
    double objective_value = 0.0;
    for (int dof = 0; dof < ndof && valid_solution; dof++) {
      try {
        PiecewisePolynomial spline = nWaypointCubicSpline(segment_times, xs(dof, 0), xd0[dof], xs(dof, num_segments), xdf[dof], xi[dof]);
        PiecewisePolynomial acceleration_squared = spline.derivative(2);
        acceleration_squared *= acceleration_squared;
        PiecewisePolynomial acceleration_squared_integral = acceleration_squared.integral();
        objective_value += acceleration_squared_integral.value(spline.getEndTime()) - acceleration_squared_integral.value(spline.getStartTime());
      }
      catch (ConstraintMatrixSingularError&) {
        valid_solution = false;
      }
    }

    if (valid_solution && objective_value < min_objective_value) {
      best_segment_times = segment_times;
      min_objective_value = objective_value;
    }
  }

  for (mwSize dof = 0; dof < ndof; dof++) {
    PiecewisePolynomial spline = nWaypointCubicSpline(best_segment_times, xs(dof, 0), xd0[dof], xs(dof, num_segments), xdf[dof], xi[dof]);
    for (mwSize segment_index = 0; segment_index < spline.getNumberOfSegments(); segment_index++) {
      for (mwSize coefficient_index = 0; coefficient_index < num_coeffs_per_segment; coefficient_index++) {
        mwSize sub[] = {dof, segment_index, num_coeffs_per_segment - coefficient_index - 1}; // Matlab's reverse coefficient indexing...
        *(mxGetPr(plhs[0]) + sub2ind(num_segments, dims, sub)) = spline.getPolynomial(segment_index).getCoefficients()[coefficient_index];
      }
    }
  }
  plhs[1] = stdVectorToMatlab(best_segment_times);

  if (nlhs > 2)
    plhs[2] = mxCreateDoubleScalar(min_objective_value);
}
