#include <mex.h>

#include <Eigen/Core>
#include "drake/solvers/qpSpline/splineGeneration.h"
#include "drake/matlab/util/drakeMexUtil.h"
#include <iostream>

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  string usage = "[coefs, objval] = nWaypointCubicSplinemex(ts, xs, xd0, xdf)";
  if (nrhs != 4)
    mexErrMsgIdAndTxt("Drake:nWaypointCubicSplinemex:WrongNumberOfInputs",
                      usage.c_str());
  if (nlhs > 2)
    mexErrMsgIdAndTxt("Drake:nWaypointCubicSplinemex:WrongNumberOfOutputs",
                      usage.c_str());

  const std::vector<double> segment_times = matlabToStdVector<double>(prhs[0]);
  MatrixXd xs = matlabToEigen<Dynamic, Dynamic>(prhs[1]);
  auto xd0 = matlabToEigen<Dynamic, 1>(prhs[2]);
  auto xdf = matlabToEigen<Dynamic, 1>(prhs[3]);

  mwSize ndof = static_cast<mwSize>(xs.rows());
  mwSize num_segments = static_cast<mwSize>(xs.cols()) - 1;
  mwSize num_knots = num_segments - 1;
  mwSize num_coeffs_per_segment = 4;
  mwSize dims[] = {ndof, num_segments, num_coeffs_per_segment};
  plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
  double objective_value = 0.0;
  for (mwSize dof = 0; dof < ndof; dof++) {
    VectorXd xi = xs.block(dof, 1, 1, num_knots).transpose();

    PiecewisePolynomial<double> spline =
        nWaypointCubicSpline(segment_times, xs(dof, 0), xd0[dof],
                             xs(dof, num_segments), xdf[dof], xi);

    PiecewisePolynomial<double> acceleration_squared = spline.derivative(2);
    acceleration_squared *= acceleration_squared;
    PiecewisePolynomial<double> acceleration_squared_integral =
        acceleration_squared.integral();
    objective_value +=
        acceleration_squared_integral.scalarValue(spline.getEndTime()) -
        acceleration_squared_integral.scalarValue(spline.getStartTime());

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

  if (nlhs > 1) {
    plhs[1] = mxCreateDoubleScalar(objective_value);
  }
}
