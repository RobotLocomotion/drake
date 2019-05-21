#include "drake/solvers/test/csdp_test_examples.h"

#include <limits>
#include <vector>

namespace drake {
namespace solvers {
const double kInf = std::numeric_limits<double>::infinity();
SDPwithOverlappingVariables::SDPwithOverlappingVariables()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<3>()} {
  prog_->AddLinearCost(2 * x_(0) + x_(2));
  prog_->AddBoundingBoxConstraint(1, 1, x_(1));
  prog_->AddPositiveSemidefiniteConstraint(
      (Matrix2<symbolic::Variable>() << x_(0), x_(1), x_(1), x_(0)).finished());
  prog_->AddPositiveSemidefiniteConstraint(
      (Matrix2<symbolic::Variable>() << x_(0), x_(2), x_(2), x_(0)).finished());
}

CsdpDocExample::CsdpDocExample()
    : prog_{new MathematicalProgram()},
      X1_{prog_->NewSymmetricContinuousVariables<2>()},
      X2_{prog_->NewSymmetricContinuousVariables<3>()},
      y_{prog_->NewContinuousVariables<2>()} {
  prog_->AddPositiveSemidefiniteConstraint(X1_);
  prog_->AddPositiveSemidefiniteConstraint(X2_);
  prog_->AddBoundingBoxConstraint(Eigen::Vector2d(0, 0),
                                  Eigen::Vector2d(kInf, kInf), y_);
  prog_->AddLinearCost(-(2 * X1_(0, 0) + 2 * X1_(0, 1) + 2 * X1_(1, 1) +
                         3 * X2_(0, 0) + 2 * X2_(1, 1) + 2 * X2_(0, 2) +
                         3 * X2_(2, 2)));
  prog_->AddLinearEqualityConstraint(
      3 * X1_(0, 0) + 2 * X1_(0, 1) + 3 * X1_(1, 1) + y_(0), 1);
  prog_->AddLinearEqualityConstraint(
      3 * X2_(0, 0) + 4 * X2_(1, 1) + 2 * X2_(0, 2) + 5 * X2_(2, 2) + y_(1), 2);
}

LinearProgram1::LinearProgram1()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<7>()} {
  Eigen::Matrix<double, 7, 1> lower, upper;
  lower << 0, 0, -1, -kInf, -2, 0, 1;
  upper << kInf, 5, kInf, 10, 5, 0, 1;
  prog_->AddBoundingBoxConstraint(lower, upper, x_);
  prog_->AddLinearCost(-(-x_(0) + x_(1) - 2 * x_(2) + 3 * x_(3) + x_(4) + 1));
}

LinearProgram2::LinearProgram2()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<3>()} {
  prog_->AddLinearEqualityConstraint(2 * x_(0) + 3 * x_(1) + x_(2), 1);
  Eigen::Matrix<double, 4, 3> A;
  // clang-format off
    A << 1, 0, -2,
         1, 2, 0,
         -1, 0, 3,
         1, 1, 4;
  // clang-format on
  Eigen::Vector4d lower(-kInf, -2, -2, 3);
  Eigen::Vector4d upper(-1, kInf, 3, 3);
  prog_->AddLinearConstraint(A, lower, upper, x_);
  prog_->AddLinearCost(x_(0) + 2 * x_(1) + 3 * x_(2));
}

TrivialSDP1::TrivialSDP1()
    : prog_{new MathematicalProgram()},
      X1_{prog_->NewSymmetricContinuousVariables<3>()} {
  prog_->AddPositiveSemidefiniteConstraint(X1_);
  prog_->AddLinearEqualityConstraint(X1_(0, 0) + X1_(1, 1) + X1_(2, 2), 1);
  prog_->AddLinearConstraint(X1_(0, 1) + X1_(1, 2) - 2 * X1_(0, 2), -kInf, 0);
  prog_->AddLinearCost(-(X1_(0, 1) + X1_(1, 2)));
}

TrivialSDP2::TrivialSDP2()
    : prog_{new MathematicalProgram()},
      X1_{prog_->NewSymmetricContinuousVariables<2>()},
      y_{prog_->NewContinuousVariables<1>()(0)} {
  prog_->AddPositiveSemidefiniteConstraint(X1_);
  Eigen::Matrix2d F0 = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d F1;
  F1 << 1, 2, 2, 3;
  Eigen::Matrix2d F2;
  F2 << 2, 0, 0, 4;
  prog_->AddConstraint(
      std::make_shared<LinearMatrixInequalityConstraint>(
          std::vector<Eigen::Ref<const Eigen::MatrixXd>>{F0, F1, F2}),
      Vector2<symbolic::Variable>(y_, X1_(0, 0)));
  prog_->AddLinearEqualityConstraint(X1_(0, 0) + 2 * X1_(1, 1) + 3 * y_, 1);
  prog_->AddLinearCost(-y_);
}
}  // namespace solvers
}  // namespace drake
