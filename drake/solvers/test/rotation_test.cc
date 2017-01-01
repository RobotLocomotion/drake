#include "drake/solvers/rotation.h"

#include <random>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

GTEST_TEST(RotationTest, TestRelaxations) {
  std::mt19937 generator(42);
  // Random quaternion (algorithm from Eigen::Quaternion::UnitRandom, but using
  // c++11 rands).
  std::uniform_real_distribution<> rand(0, 1);
  const double u1 = rand(generator), u2 = 2 * M_PI * rand(generator),
               u3 = 2 * M_PI * rand(generator);
  const double a = sqrt(1 - u1), b = sqrt(u1);
  Eigen::Quaternion<double> q(a * sin(u2), a * cos(u2), b * sin(u3),
                              b * cos(u3));

  // Generate a few random points
  int num_points = 2;
  std::normal_distribution<double> randn;
  Eigen::Matrix3Xd points(3, num_points), rotated_points(3, num_points);

  for (int k = 0; k < num_points; k++) {
    for (int j = 0; j < 3; j++) {
      points(j, k) = randn(generator);
    }
    rotated_points.col(k) = q._transformVector(points.col(k));
    // Add some noise.
    for (int j = 0; j < 3; j++) {
      rotated_points(j, k) += .0001 * randn(generator);
    }
  }

  enum RelaxationType { kUnconstrained = 0, kSpectralSdpRelaxation = 1, kOrthogonalSocpRelaxation = 2 };

  for (int relaxation_type = 0; relaxation_type < 2; relaxation_type++) {
    MathematicalProgram prog;

    // min_R  sum_k sqrt | R*points.col(k) - rotated_points.col(k) |^2
    //  s.t.  R is a rotation matrix (approx)
    // To formulate it as a linear objective (for SDP), we write
    // min sum_k sigma_k
    //   s.t. Z.col(k) = R * points.col(k) - rotated_points.col(k),
    //        sigma_k >= sqrt(z0^2 + ... z2^2)

    DecisionVariableMatrixX R;

    switch (relaxation_type) {
      case kUnconstrained:
        R = prog.AddContinuousVariables<3, 3>("R");
        break;
      case kSpectralSdpRelaxation:
        R = NewRotationMatrixSpectrahedralSdpRelaxation(&prog);
        break;
      case kOrthogonalSocpRelaxation:
        R = NewRotationMatrixOrthonormalSocpRelaxation(&prog);
        break;
      default:
        throw std::runtime_error("Bad relaxation_type.");
    }

    auto Z = prog.AddContinuousVariables(3, points.cols(), "Z");
    auto sigma = prog.AddContinuousVariables(points.cols(), "sigma");

    Eigen::Matrix3Xd A(3, 9 + 3);
    A << Eigen::Matrix<double, 3, 9>::Zero(), -Eigen::Matrix3d::Identity();
    for (int k = 0; k < points.cols(); k++) {
      A(0, 0) = points(0, k);
      A(1, 1) = points(0, k);
      A(2, 2) = points(0, k);

      A(0, 3) = points(1, k);
      A(1, 4) = points(1, k);
      A(2, 5) = points(1, k);

      A(0, 6) = points(2, k);
      A(1, 7) = points(2, k);
      A(2, 8) = points(2, k);

      // R*points.col(k) - Z.col(k) = rotated_points.col(k).
      prog.AddLinearEqualityConstraint(
          A, rotated_points.col(k), {R.col(0), R.col(1), R.col(2), Z.col(k)});

      // sigma(k) > sqrt(z(0,k)^2 + z(1,k)^2 + z(2,k)^2).
      prog.AddLorentzConeConstraint({sigma.segment<1>(k), Z.col(k)});
    }

    // min sum_k sigma_k
    prog.AddLinearCost(Eigen::VectorXd::Ones(points.cols()), {sigma});

    auto r = prog.Solve();
    //    prog.PrintSolution();

    std::string solver_name;
    int solver_result;
    prog.GetSolverResult(&solver_name, &solver_result);
    std::cout << solver_name << " exit code = " << static_cast<int>(r)
              << std::endl;

    Eigen::Matrix3d Rmat = GetSolution(R);
    Eigen::Quaternion<double> q_estimated(Rmat);

    double tol = 1e-2;
    if (relaxation_type == kUnconstrained) {
      // The interesting case is when the unconstrained version doesn't actually
      // find a rotation matrix.
      EXPECT_FALSE(CompareMatrices(Rmat * Rmat.transpose(),
                                   Eigen::Matrix3d::Identity(),tol));
    } else {
      // Check that the quaternions are the same (up to a negation).
      EXPECT_NEAR(std::abs(q.coeffs().dot(q_estimated.coeffs())), 1,tol);

      // Check that we actually found a rotation matrix.
      EXPECT_TRUE(CompareMatrices(Rmat * Rmat.transpose(),
                                  Eigen::Matrix3d::Identity(),tol));
    }
  }
}

}  // namespace solvers
}  // namespace drake
