#include "drake/solvers/test/csdp_test_examples.h"

#include <limits>
#include <vector>

namespace drake {
namespace solvers {
const double kInf = std::numeric_limits<double>::infinity();
SDPwithOverlappingVariables1::SDPwithOverlappingVariables1()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<3>()} {
  prog_->AddLinearCost(2 * x_(0) + x_(2));
  prog_->AddBoundingBoxConstraint(1, 1, x_(1));
  prog_->AddBoundingBoxConstraint(0.5, kInf, x_(0));
  prog_->AddBoundingBoxConstraint(-kInf, 2, x_(2));
  prog_->AddPositiveSemidefiniteConstraint(
      (Matrix2<symbolic::Variable>() << x_(0), x_(1), x_(1), x_(0)).finished());
  prog_->AddPositiveSemidefiniteConstraint(
      (Matrix2<symbolic::Variable>() << x_(0), x_(2), x_(2), x_(0)).finished());
}

SDPwithOverlappingVariables2::SDPwithOverlappingVariables2()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<2>()} {
  prog_->AddLinearCost(2 * x_(0) + x_(1));
  prog_->AddBoundingBoxConstraint(Eigen::Vector2d(2, 1),
                                  Eigen::Vector2d(3, kInf), x_);
  prog_->AddPositiveSemidefiniteConstraint(
      (Matrix2<symbolic::Variable>() << x_(0), x_(1), x_(1), x_(0)).finished());
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

LinearProgramBoundingBox1::LinearProgramBoundingBox1()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<8>()} {
  Eigen::Matrix<double, 8, 1> lower, upper;
  lower << 0, 0, -1, -kInf, -2, 0, 1, -kInf;
  upper << kInf, 5, kInf, 10, 5, 0, 1, kInf;
  prog_->AddBoundingBoxConstraint(lower, upper, x_);
  prog_->AddLinearCost(-(-x_(0) + x_(1) - 2 * x_(2) + 3 * x_(3) + x_(4) + 1));
}

CsdpLinearProgram2::CsdpLinearProgram2()
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

CsdpLinearProgram3::CsdpLinearProgram3()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<3>()} {
  prog_->AddBoundingBoxConstraint(Eigen::Vector2d(-1, -kInf),
                                  Eigen::Vector2d(10, 8), x_.head<2>());
  Eigen::Matrix<double, 4, 3> A;
  // clang-format off
  A << 1, 2, 3,
       2, 0, -1,
       0, 1, -3,
       1, 0, 1;
  // clang-format on
  prog_->AddLinearConstraint(A, Eigen::Vector4d(3, -1, -kInf, -4),
                             Eigen::Vector4d(3, kInf, 5, 9), x_);
  prog_->AddLinearCost(-(2 * x_(0) + 3 * x_(1) + 4 * x_(2) + 3));
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

TrivialSOCP1::TrivialSOCP1()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<3>()} {
  prog_->AddBoundingBoxConstraint(Eigen::Vector2d::Zero(),
                                  Eigen::Vector2d(kInf, kInf), x_.tail<2>());
  prog_->AddLinearEqualityConstraint(x_(0) + x_(1) + x_(2), 10);
  Eigen::Matrix3d A;
  // clang-format off
  A << 2, 0, 0,
       0, 3, 0,
       1, 0, 1;
  // clang-format on
  Eigen::Vector3d b(1, 2, 3);
  prog_->AddLorentzConeConstraint(A, b, x_);

  prog_->AddLinearCost(-x_(0));
}

TrivialSOCP2::TrivialSOCP2()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<2>()} {
  prog_->AddLinearCost(-x_(1));
  Eigen::Matrix<double, 3, 2> A;
  // clang-format off
  A << 1, 0,
       1, 1,
       1, -1;
  // clang-format on
  Eigen::Vector3d b(2, 1, 1);
  prog_->AddLorentzConeConstraint(A, b, x_);
}

TrivialSOCP3::TrivialSOCP3()
    : prog_{new MathematicalProgram()}, x_{prog_->NewContinuousVariables<2>()} {
  prog_->AddLinearCost(x_(1));
  Eigen::Matrix<double, 4, 2> A;
  // clang-format off
  A << 2, 0,
       0, 3,
       1, 0,
       3, 1;
  // clang-format on
  Eigen::Vector4d b(2, 4, 2, 1);
  prog_->AddRotatedLorentzConeConstraint(A, b, x_);
}
}  // namespace solvers
}  // namespace drake
