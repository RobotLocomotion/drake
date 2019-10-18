#include <limits>

#include "drake/common/proto/call_python.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mixed_integer_rotation_constraint.h"
#include "drake/solvers/rotation_constraint.h"
#include "drake/solvers/solve.h"

/// Provides a simple utility for developers to visualize (slices of) the
/// rotation matrix relaxations.  Sets up the problem:
///    min_R max_(i,j) |R(i,j) - R_sample(i,j)|
///   subject to  [ chosen list of constraints on R ]
/// It then evaluates this program over a large number of samples and plots the
/// points (using call_python) for which R==R_sample (within some tol).

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace drake {
namespace solvers {
namespace {

// Use this method to change the feasibility envelope for all of the plotting
// tools.

void AddTestConstraints(MathematicalProgram* prog,
                        const MatrixDecisionVariable<3, 3>& R) {
  // Add your favorite constraints here.
  AddRotationMatrixBoxSphereIntersectionMilpConstraints(R, 1, prog);
}

bool IsFeasible(
    const MathematicalProgram& prog,
    const std::shared_ptr<LinearEqualityConstraint>& feasibility_constraint,
    const Eigen::Ref<const Eigen::MatrixXd>& R_sample) {
  Eigen::Map<const Eigen::VectorXd> R_sample_vec(R_sample.data(),
                                                 R_sample.size());
  feasibility_constraint->UpdateLowerBound(R_sample_vec);
  feasibility_constraint->UpdateUpperBound(R_sample_vec);

  return Solve(prog).is_success();
}

void DrawCircle(double radius = 1.0) {
  const int N = 100;

  Eigen::Matrix2Xd points(2, N);

  // Draw circle.
  for (int i = 0; i < N; i++) {
    const double theta = i * 2.0 * M_PI / (N - 1);
    points.col(i) =
        Eigen::Vector2d(radius * std::cos(theta), radius * std::sin(theta));
  }

  common::CallPython("plot", points.row(0), points.row(1), "k", "linewidth",
                     4.0);
}

void PlotFeasiblePoints(const Eigen::Matrix2Xd& points, double radius = 1.0,
                        int fig_num = 1) {
  using common::CallPython;
  CallPython("figure", fig_num);
  CallPython("clf");
  CallPython("plot", points.row(0), points.row(1), ".", "markersize", 20.0);
  DrawCircle(radius);
  CallPython("xlim", Eigen::RowVector2d(-1.1, 1.1));
  CallPython("ylim", Eigen::RowVector2d(-1.1, 1.1));
  CallPython("axis", "equal");
}

void PlotColumnVectorXYSlice(double z = 0.0, int fig_num = 1) {
  Vector3d sample(0, 0, z);

  MathematicalProgram prog;
  MatrixDecisionVariable<3, 3> R = NewRotationMatrixVars(&prog);
  AddTestConstraints(&prog, R);

  // Add a feasibility constraint.
  std::shared_ptr<LinearEqualityConstraint> feasibility_constraint =
      prog.AddLinearEqualityConstraint(Eigen::Matrix3d::Identity(),
                                       Eigen::Vector3d::Zero(), R.col(0))
          .evaluator();

  const int num_samples_per_axis = 50;
  Eigen::Matrix2Xd feasible_points(2,
                                   num_samples_per_axis * num_samples_per_axis);

  int num_feasible = 0;
  double minval = -1.0, maxval = 1.0;
  for (int i = 0; i < num_samples_per_axis; i++) {
    double x = minval + i * (maxval - minval) / (num_samples_per_axis - 1);
    sample(0) = x;
    for (int j = 0; j < num_samples_per_axis; j++) {
      double y = minval + j * (maxval - minval) / (num_samples_per_axis - 1);
      sample(1) = y;
      if (IsFeasible(prog, feasibility_constraint, sample))
        feasible_points.col(num_feasible++) = Eigen::Vector2d(x, y);
    }
    std::cout << "." << std::flush;
  }
  feasible_points.conservativeResize(2, num_feasible);

  PlotFeasiblePoints(feasible_points, std::sqrt(1 - sample(2) * sample(2)),
                     fig_num);
  using common::CallPython;
  CallPython("xlabel", "R(0,0)");
  CallPython("ylabel", "R(1,0)");
  CallPython("title", "R(2,0) = " + std::to_string(z));
}

void DoMain() {
  // Make some plots

  // Plots z=0 slice of the first column.
  // To plot z=0.25, set R0(2,0) = 0.25.
  PlotColumnVectorXYSlice(0.0);
}

}  // namespace
}  // namespace solvers
}  // namespace drake

int main() {
  drake::solvers::DoMain();
  return 0;
}
