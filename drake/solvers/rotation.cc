#include "drake/solvers/rotation.h"

namespace drake {
namespace solvers {

DecisionVariableMatrixX NewRotationMatrixSpectrahedralSdpRelaxation(
    MathematicalProgram* prog, const std::string& name) {
  DecisionVariableMatrixX R = prog->AddContinuousVariables<3, 3>(name);

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

  return R;
}

void AddOrthogonalConstraint(MathematicalProgram* prog,
                             const DecisionVariableVector<3>& v1,
                             const DecisionVariableVector<3>& v2, const DecisionVariableVector<1>& one) {
  // We do this by introducing
  //   t_plus  >= |v1+v2|^2 = v1'v1 + 2v1'v2 + v2'v2 <= 2
  //   t_minus >= |v1-v2|^2 = v1'v1 - 2v1'v2 + v2'v2 <= 2
  // This is tight when v1'v1 = 1 and v2'v2 = 1.

  auto v1_plus_v2 = prog->AddContinuousVariables<3>("v1_plus_v2");
  auto v1_minus_v2 = prog->AddContinuousVariables<3>("v1_minus_v2");
  
  Eigen::Matrix3Xd A(3,9);
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

DecisionVariableMatrixX NewRotationMatrixOrthonormalSocpRelaxation(
    MathematicalProgram* prog, const std::string& name) {
  DecisionVariableMatrixX R = prog->AddContinuousVariables<3, 3>(name);
  auto one = prog->AddContinuousVariables<1, 1>(name + "_one");
  prog->AddBoundingBoxConstraint(1, 1, one(0, 0));

  // All columns should be unit length (but we can only write Ri'Ri<=1).
  for (int i = 0; i < 3; i++) {
    prog->AddLorentzConeConstraint({one, R.col(i)});
  }

  AddOrthogonalConstraint(prog, R.col(0), R.col(1), one);  // R1'*R2 = 0.
  AddOrthogonalConstraint(prog, R.col(1), R.col(2), one);  // R2'*R3 = 0.
  AddOrthogonalConstraint(prog, R.col(0), R.col(2), one);  // R1'*R3 = 0.

  // R3 = cross(R1,R2)
  {
    // s = u x v = [ 0 -u3 u2; u3 ... ]*v.
  }

  return R;
}

}  // namespace solvers
}  // namespace drake
