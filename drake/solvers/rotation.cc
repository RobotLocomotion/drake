#include "drake/solvers/rotation.h"

namespace drake {
namespace solvers {

DecisionVariableMatrixX NewRotationMatrixVars(MathematicalProgram* prog,
                                              const std::string& name) {
  DecisionVariableMatrixX R = prog->AddContinuousVariables<3, 3>(name);
  prog->AddBoundingBoxConstraint(-1, 1, {R});
  return R;
}

void AddRollPitchYawLimitBoundingBoxConstraints(
    MathematicalProgram* prog, const DecisionVariableMatrixX& R,
    RollPitchYawLimits limits) {
  // Based on the RPY to Rotation Matrix conversion:
  // [ cp*cy, cy*sp*sr - cr*sy, sr*sy + cr*cy*sp]
  // [ cp*sy, cr*cy + sp*sr*sy, cr*sp*sy - cy*sr]
  // [   -sp,            cp*sr,            cp*cr]
  // where cz = cos(z) and sz = sin(z), and using
  //  kRoll_NegPI_2_to_PI_2 = 1 << 1,   // => cos(r)>=0
  //  kRoll_0_to_PI = 1 << 2,           // => sin(r)>=0
  //  kPitch_NegPI_2_to_PI_2 = 1 << 3,  // => cos(p)>=0
  //  kPitch_0_to_PI = 1 << 4,          // => sin(p)>=0
  //  kYaw_NegPI_2_to_PI_2 = 1 << 5,    // => cos(y)>=0
  //  kYaw_0_to_PI = 1 << 6,            // => sin(y)>=0

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2))
    prog->AddBoundingBoxConstraint(0, 1, R(0, 0));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kYaw_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(1, 0));

  if (limits & kPitch_0_to_PI) prog->AddBoundingBoxConstraint(-1, 0, R(2, 0));

  if ((limits & kRoll_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2) &&
      (limits & kPitch_0_to_PI) && (limits & kRoll_0_to_PI) &&
      (limits & kYaw_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(1, 1));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kRoll_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(2, 1));

  if ((limits & kRoll_0_to_PI) && (limits & kYaw_0_to_PI) &&
      (limits & kRoll_NegPI_2_to_PI_2) && (limits & kYaw_NegPI_2_to_PI_2) &&
      (limits & kPitch_0_to_PI))
    prog->AddBoundingBoxConstraint(0, 1, R(0, 2));

  if ((limits & kPitch_NegPI_2_to_PI_2) && (limits & kRoll_NegPI_2_to_PI_2))
    prog->AddBoundingBoxConstraint(0, 1, R(2, 2));
}

void AddRotationMatrixSpectrahedralSdpConstraint(
    MathematicalProgram* prog, const DecisionVariableMatrixX& R) {
  // TODO(russt): Clean this up using symbolic expressions!
  Eigen::Matrix4d F0 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d F11 = Eigen::Matrix4d::Zero();
  F11(0, 0) = -1;
  F11(1, 1) = 1;
  F11(2, 2) = 1;
  F11(3, 3) = -1;
  Eigen::Matrix4d F21 = Eigen::Matrix4d::Zero();
  F21(0, 2) = -1;
  F21(1, 3) = 1;
  F21(2, 0) = -1;
  F21(3, 1) = 1;
  Eigen::Matrix4d F31 = Eigen::Matrix4d::Zero();
  F31(0, 1) = 1;
  F31(1, 0) = 1;
  F31(2, 3) = 1;
  F31(3, 2) = 1;
  Eigen::Matrix4d F12 = Eigen::Matrix4d::Zero();
  F12(0, 2) = 1;
  F12(1, 3) = 1;
  F12(2, 0) = 1;
  F12(3, 1) = 1;
  Eigen::Matrix4d F22 = Eigen::Matrix4d::Zero();
  F22(0, 0) = -1;
  F22(1, 1) = -1;
  F22(2, 2) = 1;
  F22(3, 3) = 1;
  Eigen::Matrix4d F32 = Eigen::Matrix4d::Zero();
  F32(0, 3) = 1;
  F32(1, 2) = -1;
  F32(2, 1) = -1;
  F32(3, 0) = 1;
  Eigen::Matrix4d F13 = Eigen::Matrix4d::Zero();
  F13(0, 1) = 1;
  F13(1, 0) = 1;
  F13(2, 3) = -1;
  F13(3, 2) = -1;
  Eigen::Matrix4d F23 = Eigen::Matrix4d::Zero();
  F23(0, 3) = 1;
  F23(1, 2) = 1;
  F23(2, 1) = 1;
  F23(3, 0) = 1;
  Eigen::Matrix4d F33 = Eigen::Matrix4d::Zero();
  F33(0, 0) = 1;
  F33(1, 1) = -1;
  F33(2, 2) = 1;
  F33(3, 3) = -1;

  prog->AddLinearMatrixInequalityConstraint(
      {F0, F11, F21, F31, F12, F22, F32, F13, F23, F33},
      {R.col(0), R.col(1), R.col(2)});
}

void AddOrthogonalConstraint(MathematicalProgram* prog,
                             const DecisionVariableVector<3>& v1,
                             const DecisionVariableVector<3>& v2,
                             const DecisionVariableVector<1>& one) {
  // We do this by introducing
  //   t_plus  >= |v1+v2|^2 = v1'v1 + 2v1'v2 + v2'v2 <= 2
  //   t_minus >= |v1-v2|^2 = v1'v1 - 2v1'v2 + v2'v2 <= 2
  // This is tight when v1'v1 = 1 and v2'v2 = 1.

  auto v1_plus_v2 = prog->AddContinuousVariables<3>("v1_plus_v2");
  auto v1_minus_v2 = prog->AddContinuousVariables<3>("v1_minus_v2");

  Eigen::Matrix3Xd A(3, 9);
  A << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity(),
      -Eigen::Matrix3d::Identity();
  prog->AddLinearEqualityConstraint(A, Eigen::Vector3d::Zero(),
                                    {v1, v2, v1_plus_v2});
  A.leftCols<3>() = -Eigen::Matrix3d::Identity();
  prog->AddLinearEqualityConstraint(A, Eigen::Vector3d::Zero(),
                                    {v2, v1, v1_minus_v2});

  auto t_plus = prog->AddContinuousVariables<1>("t_plus");
  auto t_minus = prog->AddContinuousVariables<1>("t_minus");
  prog->AddBoundingBoxConstraint(0, 2, {t_plus, t_minus});

  //   t_plus  >= |v1+v2|^2
  //   t_minus >= |v1-v2|^2
  prog->AddRotatedLorentzConeConstraint({t_plus.head<1>(), one, v1_plus_v2});
  prog->AddRotatedLorentzConeConstraint({t_minus.head<1>(), one, v1_minus_v2});
}

void AddRotationMatrixOrthonormalSocpConstraint(
    MathematicalProgram* prog, const DecisionVariableMatrixX& R) {
  auto one = prog->AddContinuousVariables<1, 1>("one");
  prog->AddBoundingBoxConstraint(1, 1, one(0, 0));

  // All columns should be unit length (but we can only write Ri'Ri<=1).
  for (int i = 0; i < 3; i++) {
    prog->AddLorentzConeConstraint({one, R.col(i)});
  }

  AddOrthogonalConstraint(prog, R.col(0), R.col(1), one);  // R1'*R2 = 0.
  AddOrthogonalConstraint(prog, R.col(1), R.col(2), one);  // R2'*R3 = 0.
  AddOrthogonalConstraint(prog, R.col(0), R.col(2), one);  // R1'*R3 = 0.
}

void AddVectorL1NormConstraint(MathematicalProgram* prog,
                               const DecisionVariableVector<3>& v,
                               const std::string& suffix = "") {
  // Declare a binary variable for every orthant of R(3),
  // B(0) = B---, B(1) = B--+, ..., B(7) = B+++.
  auto B = prog->AddBinaryVariables<8>("bR" + suffix);

  // Sum B = 1 (can only be in one orthant).
  prog->AddLinearEqualityConstraint(Eigen::RowVectorXd::Ones(1, 8), 1, {B});

  {  // v(0) >= 0 iff at least one B+** is true, written as
    // -B--- - B--+ - B-+- - B-++ <= v(0) <= B+++ + B+-+ + B+-+ + B+--, etc.
    Eigen::Matrix<double, 1, 9> a;
    a << 1, 1, 1, 1, 1, -1, -1, -1, -1;
    prog->AddLinearEqualityConstraint(a, 0, {v.segment<1>(0), B});

    // -B--- - B--+ - B+-- - B+-+ <= v(1) <= B+++ + B++- + B-++ + B-+-, etc.
    a << 1, 1, 1, -1, -1, 1, 1, -1, -1;
    prog->AddLinearEqualityConstraint(a, 0, {v.segment<1>(1), B});

    // -B--- - B-+- - B+-- - B++- <= v(2) <= B+++ + B+-+ + B-++ + B--+, etc.
    a << 1, 1, -1, 1, -1, 1, -1, 1, -1;
    prog->AddLinearEqualityConstraint(a, 0, {v.segment<1>(2), B});
  }

  {  // |x|+|y|+|z|>=1, written as
    //    -2+3*B+++ <=  x + y + z <= 2-3*B---
    Eigen::Matrix<double, 1, 5> a;
    a << 1, 1, 1, -3, 3;
    prog->AddLinearConstraint(a, -2, 2, {v, B.segment<1>(7), B.segment<1>(0)});
    //    -2+3*B++- <=  x + y - z <= 2-3*B--+
    a << 1, 1, -1, -3, 3;
    prog->AddLinearConstraint(a, -2, 2, {v, B.segment<1>(6), B.segment<1>(1)});
    //    -2+3*B+-+ <=  x - y + z <= 2-3*B-+-
    a << 1, -1, 1, -3, 3;
    prog->AddLinearConstraint(a, -2, 2, {v, B.segment<1>(5), B.segment<1>(2)});
    //    -2+3*B-++ <= -x + y + z <= 2-3*B+--
    a << -1, 1, 1, -3, 3;
    prog->AddLinearConstraint(a, -2, 2, {v, B.segment<1>(3), B.segment<1>(4)});
  }
}

void AddRotationMatrixL1NormMilpConstraint(MathematicalProgram* prog,
                                           const DecisionVariableMatrixX& R,
                                           RollPitchYawLimits limits) {
  AddVectorL1NormConstraint(prog, R.col(0), "0");
  AddVectorL1NormConstraint(prog, R.col(1), "1");
  AddVectorL1NormConstraint(prog, R.col(2), "2");
}

}  // namespace solvers
}  // namespace drake
