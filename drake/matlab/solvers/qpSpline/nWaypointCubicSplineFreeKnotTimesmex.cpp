#include <mex.h>

#include <Eigen/Core>
#include "drake/solvers/qpSpline/splineGeneration.h"
#include "drake/matlab/util/drakeMexUtil.h"
#include <iostream>
#include <limits>

using namespace std;
using namespace Eigen;

const int GRID_STEPS = 10;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  string usage =
      "[coefs, ts, objective_value] = "
      "nWaypointCubicSplineFreeKnotTimesmex.cpp(t0, tf, xs, xd0, xdf)";
  if (nrhs != 5)
    mexErrMsgIdAndTxt(
        "Drake:nWaypointCubicSplineFreeKnotTimesmex.cpp:WrongNumberOfInputs",
        usage.c_str());
  if (nlhs < 2 || nlhs > 3)
    mexErrMsgIdAndTxt(
        "Drake:nWaypointCubicSplineFreeKnotTimesmex.cpp:WrongNumberOfOutputs",
        usage.c_str());

  double t0 = mxGetPrSafe(prhs[0])[0];
  double tf = mxGetPrSafe(prhs[1])[0];
  MatrixXd xs = matlabToEigen<Dynamic, Dynamic>(prhs[2]);
  auto xd0 = matlabToEigen<Dynamic, 1>(prhs[3]);
  auto xdf = matlabToEigen<Dynamic, 1>(prhs[4]);

  mwSize ndof = static_cast<mwSize>(xs.rows());
  mwSize num_segments = static_cast<mwSize>(xs.cols()) - 1;
  mwSize num_knots = num_segments - 1;
  if (num_knots >= 3)
    mexWarnMsgTxt(
        "More knots than two is likely to be super slow in a grid search!\n");
  if (num_knots <= 0)
    mexErrMsgIdAndTxt(
        "Drake:nWaypointCubicSplineFreeKnotTimesmex.cpp:"
        "NotEnoughKnotsToJustifyThisFunction",
        usage.c_str());
  mwSize num_coeffs_per_segment = 4;
  mwSize dims[] = {ndof, num_segments, num_coeffs_per_segment};
  plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);

  std::vector<double> segment_times;
  segment_times.resize(static_cast<size_t>(num_segments) + 1);
  segment_times[0] = t0;
  segment_times[static_cast<size_t>(num_segments)] = tf;
  std::vector<double> best_segment_times = segment_times;
  double t_step = (tf - t0) / GRID_STEPS;
  double min_objective_value = numeric_limits<double>::infinity();

  // assemble the knot point locations for input to nWaypointCubicSpline
  MatrixXd xi = xs.block(0, 1, ndof, num_knots);

  if (GRID_STEPS <= num_knots) {
    // If we have have too few grid steps, then by pigeonhole it's
    // impossible to give each a unique time in our grid search.
    mexErrMsgIdAndTxt(
        "Drake:nWaypointCubicSplineFreeKnotTimesmex.cpp:"
        "TooManyKnotsForNumGridSteps",
        usage.c_str());
  }
  std::vector<int> t_indices;
  t_indices.reserve(num_knots);
  for (mwSize i = 0; i < num_knots; i++) {
    t_indices.push_back(i + 1);  // assume knot point won't be the same time as
                                 // the initial state, or previous knot point
  }

  while (t_indices[0] < (GRID_STEPS - static_cast<int>(num_knots) + 1)) {
    for (mwSize i = 0; i < num_knots; i++)
      segment_times[i + 1] = t0 + t_indices[i] * t_step;

    bool valid_solution = true;
    double objective_value = 0.0;
    for (mwSize dof = 0; dof < ndof && valid_solution; dof++) {
      try {
        PiecewisePolynomial<double> spline = nWaypointCubicSpline(
            segment_times, xs(dof, 0), xd0[dof], xs(dof, num_segments),
            xdf[dof], xi.row(dof).transpose());
        PiecewisePolynomial<double> acceleration_squared = spline.derivative(2);
        acceleration_squared *= acceleration_squared;
        PiecewisePolynomial<double> acceleration_squared_integral =
            acceleration_squared.integral();
        objective_value +=
            acceleration_squared_integral.scalarValue(spline.getEndTime()) -
            acceleration_squared_integral.scalarValue(spline.getStartTime());
      } catch (ConstraintMatrixSingularError &) {
        valid_solution = false;
      }
    }

    if (valid_solution && objective_value < min_objective_value) {
      best_segment_times = segment_times;
      min_objective_value = objective_value;
    }

    // Advance grid search counter or terminate, counting from
    // the latest t_index, and on overflow carrying to the
    // next lowest t_index and resetting to the new value of that
    // next lowest t_index. (since times must always be in order!)
    t_indices[num_knots - 1]++;
    // carry, except for the lowest place, which we
    // use to detect doneness.
    for (size_t i = num_knots - 1; i > 0; i--) {
      if ((i == num_knots - 1 && t_indices[i] >= GRID_STEPS) ||
          (i < num_knots - 1 && t_indices[i] >= t_indices[i + 1])) {
        t_indices[i - 1]++;
        t_indices[i] = t_indices[i - 1] + 1;
      }
    }
  }

  for (mwSize dof = 0; dof < ndof; dof++) {
    PiecewisePolynomial<double> spline = nWaypointCubicSpline(
        best_segment_times, xs(dof, 0), xd0[dof], xs(dof, num_segments),
        xdf[dof], xi.row(dof).transpose());
    for (mwSize segment_index = 0;
         segment_index < static_cast<mwSize>(spline.getNumberOfSegments());
         segment_index++) {
      for (mwSize coefficient_index = 0;
           coefficient_index < num_coeffs_per_segment; coefficient_index++) {
        mwSize sub[] = {dof, segment_index,
                        num_coeffs_per_segment - coefficient_index -
                            1};  // Matlab's reverse coefficient indexing...
        *(mxGetPr(plhs[0]) + sub2ind(3, dims, sub)) =
            spline.getPolynomial(static_cast<int>(segment_index))
                .GetCoefficients()[coefficient_index];
      }
    }
  }
  plhs[1] = stdVectorToMatlab(best_segment_times);

  if (nlhs > 2) plhs[2] = mxCreateDoubleScalar(min_objective_value);
}
