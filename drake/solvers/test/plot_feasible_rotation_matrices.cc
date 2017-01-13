
#include <limits>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcm/lcm_call_matlab.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/rotation_constraint.h"

/// Provides a simple utility for developers to visualize (slices of) the
/// rotation matrix relaxations.  Sets up the problem:
///    min_R max_(i,j) |R(i,j) - R_sample(i,j)|
///   subject to  [ chosen list of constraints on R ]
/// It then evaluates this program over a large number of samples and plots the
/// points (using lcm_call_matlab) for which R==R_sample (within some tol).

using Eigen::Matrix3d;

namespace drake {
namespace solvers {

// Use this method to change the feasibility envelope for all of the plotting
// tools.

void AddTestConstraints(MathematicalProgram* prog,
                        const MatrixDecisionVariable<3, 3>& R) {
  // Add your favorite constraints here.
  AddRotationMatrixMcCormickEnvelopeMilpConstraints(
      prog, R, 1);
}

bool IsFeasible(
    MathematicalProgram* prog,
    const std::shared_ptr<LinearEqualityConstraint>& feasibility_constraint,
    const Eigen::Ref<const Eigen::MatrixXd>& R_sample) {
  Eigen::Map<const Eigen::VectorXd> R_sample_vec(R_sample.data(),
                                                 R_sample.size());
  feasibility_constraint->UpdateLowerBound(R_sample_vec);
  feasibility_constraint->UpdateUpperBound(R_sample_vec);

  return (prog->Solve() == kSolutionFound);
}

void drawCircle(double radius = 1.0) {
  const int N = 100;

  Eigen::Matrix2Xd points(2, N);

  // Draw circle.
  for (int i = 0; i < N; i++) {
    const double theta = i * 2.0 * M_PI / (N - 1);
    points.col(i) =
        Eigen::Vector2d(radius * std::cos(theta), radius * std::sin(theta));
  }

  lcm::LcmCallMatlab("plot", points.row(0), points.row(1), "k", "LineWidth",
                     4.0);
}

void plotFeasiblePoints(const Eigen::Matrix2Xd& points, double radius = 1.0,
                        int fig_num = 1) {
  using lcm::LcmCallMatlab;
  LcmCallMatlab("figure", fig_num);
  LcmCallMatlab("clf");
  LcmCallMatlab("hold", "on");
  LcmCallMatlab("plot", points.row(0), points.row(1), ".", "MarkerSize", 20.0);
  drawCircle(radius);
  LcmCallMatlab("xlim", Eigen::RowVector2d(-1.1, 1.1));
  LcmCallMatlab("ylim", Eigen::RowVector2d(-1.1, 1.1));
  LcmCallMatlab("axis", "equal");
}

void plotColumnVectorXYSlice(int column = 0,
                             const Matrix3d R_otherdims = Matrix3d::Identity(),
                             int fig_num = 1) {
  Matrix3d R_sample = R_otherdims;

  MathematicalProgram prog;
  MatrixDecisionVariable<3, 3> R = NewRotationMatrixVars(&prog);
  AddTestConstraints(&prog, R);

  // Add a feasibility constraint.
  std::shared_ptr<LinearEqualityConstraint> feasibility_constraint =
      prog.AddLinearEqualityConstraint(Eigen::Matrix3d::Identity(),
                                       Eigen::Vector3d::Zero(),
                                       R.col(column));

  const int num_samples_per_axis = 50;
  Eigen::Matrix2Xd feasible_points(2,
                                   num_samples_per_axis * num_samples_per_axis);

  int num_feasible = 0;
  double minval = -1.0, maxval = 1.0;
  for (int i = 0; i < num_samples_per_axis; i++) {
    double x = minval + i * (maxval - minval) / (num_samples_per_axis - 1);
    R_sample(0, column) = x;
    for (int j = 0; j < num_samples_per_axis; j++) {
      double y = minval + j * (maxval - minval) / (num_samples_per_axis - 1);
      R_sample(1, column) = y;
      if (IsFeasible(&prog, feasibility_constraint, R_sample.col(column)))
        feasible_points.col(num_feasible++) = Eigen::Vector2d(x, y);
    }
    std::cout << ".";
  }
  feasible_points.conservativeResize(2, num_feasible);

  plotFeasiblePoints(feasible_points,
                     std::sqrt(1 - R_sample(2, column) * R_sample(2, column)),
                     fig_num);
  lcm::LcmCallMatlab("xlabel", "R(0," + std::to_string(column) + ")");
  lcm::LcmCallMatlab("ylabel", "R(1," + std::to_string(column) + ")");
  lcm::LcmCallMatlab("title", "R(2," + std::to_string(column) + ") = " +
                                  std::to_string(R_sample(2, column)));
}


void do_main() {
  // Make some plots

  // Plots z=0 slice of the first column.
  Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
  //  R0(2,0) = .25;
  plotColumnVectorXYSlice(0, R0);
}

}  // namespace solvers
}  // namespace drake

int main(int argc, char* argv[]) {
  drake::solvers::do_main();
  return 0;
}
