#include "drake/solvers/rotation.h"

namespace drake {
namespace solvers {

DecisionVariableMatrix<3, 3> NewRotationMatrixVars(MathematicalProgram* prog,
                                                   const std::string& name) {
  DecisionVariableMatrixX R = prog->AddContinuousVariables<3, 3>(name);
  prog->AddBoundingBoxConstraint(-1, 1, {R});
  return R;
}

void AddRollPitchYawLimitBoundingBoxConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R,
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
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R) {
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

  // TODO(russt): Consider generalizing this to |v1+alpha*v2|^2 <= 1+\alpha^2,
  // for any real-valued alpha.  When |R1|<|R2|<=1 or |R2|<|R1|<=1,
  // different alphas represent different constraints.

  // TODO(hongkai.dai): Rewrite this using the generalized Lorenz cone once
  // PR #4650 lands.

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
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R) {
  auto one = prog->AddContinuousVariables<1, 1>("one");
  prog->AddBoundingBoxConstraint(1, 1, one(0, 0));

  // All columns should be unit length (but we can only write Ri'Ri<=1).
  for (int i = 0; i < 3; i++) {
    prog->AddLorentzConeConstraint({one, R.col(i)});
  }

  AddOrthogonalConstraint(prog, R.col(0), R.col(1), one);  // R0'*R1 = 0.
  AddOrthogonalConstraint(prog, R.col(1), R.col(2), one);  // R1'*R2 = 0.
  AddOrthogonalConstraint(prog, R.col(0), R.col(2), one);  // R0'*R2 = 0.
}

DecisionVariableVector<8> AddVectorL1NormConstraint(
    MathematicalProgram* prog, const DecisionVariableVector<3>& v,
    const std::string& suffix = "") {
  // Declare a binary variable for every orthant of R^3,
  // B(0) = B---, B(1) = B--+, ..., B(7) = B+++,
  // where --- is in the order xyz.
  auto B = prog->AddBinaryVariables<8>("bR" + suffix);

  // Sum B = 1 (can only be in one orthant).
  prog->AddLinearEqualityConstraint(Eigen::Matrix<double, 1, 8>::Ones(), 1,
                                    {B});

  {  // v(0) >= 0 iff at least one B+** is true, exploiting that |v(0)|<=1,
    // can be written as
    // -B--- - B--+ - B-+- - B-++ <= v(0) <= B+++ + B+-+ + B+-+ + B+--, etc.
    Eigen::Matrix<double, 1, 5> aneg = Eigen::Matrix<double, 1, 5>::Ones();
    Eigen::Matrix<double, 1, 5> apos;
    apos << 1, -1, -1, -1, -1;
    prog->AddLinearConstraint(aneg, 0, 1, {v.segment<1>(0), B.segment<4>(0)});
    prog->AddLinearConstraint(apos, -1, 0, {v.segment<1>(0), B.segment<4>(4)});

    // -B--- - B--+ - B+-- - B+-+ <= v(1) <= B+++ + B++- + B-++ + B-+-, etc.
    prog->AddLinearConstraint(
        aneg, 0, 1, {v.segment<1>(1), B.segment<2>(0), B.segment<2>(4)});
    prog->AddLinearConstraint(
        apos, -1, 0, {v.segment<1>(1), B.segment<2>(2), B.segment<2>(6)});

    // -B--- - B-+- - B+-- - B++- <= v(2) <= B+++ + B+-+ + B-++ + B--+, etc.
    prog->AddLinearConstraint(
        aneg, 0, 1, {v.segment<1>(2), B.segment<1>(0), B.segment<1>(2),
                     B.segment<1>(4), B.segment<1>(6)});
    prog->AddLinearConstraint(
        apos, -1, 0, {v.segment<1>(2), B.segment<1>(1), B.segment<1>(3),
                      B.segment<1>(5), B.segment<1>(7)});
  }

  {  // 1 <= |x|+|y|+|z| <= sqrt(3), written as
    //    -sqrt(3)+(1+sqrt(3))*B+++ <=  x + y + z <= sqrt(3)-(1+sqrt(3))*B---
    // or -sqrt(3) <= x+y+z - (1+sqrt(3))*B+++ + (1-sqrt(3))*B-- <= sqrt(3), and
    //    -sqrt(3) <= x+y+z <= sqrt(3)
    const double s3 = std::sqrt(3);
    Eigen::Matrix<double, 1, 5> a;
    a << 1, 1, 1, -(1 + s3), (1 + s3);
    prog->AddLinearConstraint(a, -s3, s3,
                              {v, B.segment<1>(7), B.segment<1>(0)});
    prog->AddLinearConstraint(a.head<3>(), -s3, s3, {v});

    //    -s3+(1+s3)*B++- <=  x + y - z <= s3-(1+s3)*B--+
    a << 1, 1, -1, -(1 + s3), (1 + s3);
    prog->AddLinearConstraint(a, -s3, s3,
                              {v, B.segment<1>(6), B.segment<1>(1)});
    prog->AddLinearConstraint(a.head<3>(), -s3, s3, {v});

    //    -s3+(1+s3)*B+-+ <=  x - y + z <= s3-(1+s3)*B-+-
    a << 1, -1, 1, -(1 + s3), (1 + s3);
    prog->AddLinearConstraint(a, -s3, s3,
                              {v, B.segment<1>(5), B.segment<1>(2)});
    prog->AddLinearConstraint(a.head<3>(), -s3, s3, {v});

    //    -s3+(1+s3)*B-++ <= -x + y + z <= s3-(1+s3)*B+--
    a << -1, 1, 1, -(1 + s3), (1 + s3);
    prog->AddLinearConstraint(a, -s3, s3,
                              {v, B.segment<1>(3), B.segment<1>(4)});
    prog->AddLinearConstraint(a.head<3>(), -s3, s3, {v});
  }

  return B;
}

int orthantIndex(bool positive_x, bool positive_y, bool positive_z) {
  return (static_cast<int>(positive_x) << 2) +
         (static_cast<int>(positive_y) << 1) + static_cast<int>(positive_z);
}

void AddCrossProductConstraint(MathematicalProgram* prog,
                               const DecisionVariableVector<8>& b0,
                               const DecisionVariableVector<8>& b1,
                               const DecisionVariableVector<8>& b2,
                               int i,  // the orthant index of R0
                               int j   // the orthant index of R1
                               ) {
  // Use the following trick to discover the two orthants that are possible for
  // R2 given the orthant of R0 and R1:  Take the vectors (+/-1,+/-1,+/-1) for
  // R0 and R1 in their respective orthants.  Their cross product will have
  // exactly one zero (denoting the coordinate that can be either positive or
  // negative).

  const Eigen::Vector3d R0(i & (1 << 2) ? 1 : -1, i & (1 << 1) ? 1 : -1,
                           i & (1 << 0) ? 1 : -1);
  const Eigen::Vector3d R1(j & (1 << 2) ? 1 : -1, j & (1 << 1) ? 1 : -1,
                           j & (1 << 0) ? 1 : -1);
  const Eigen::Vector3d R2 = R0.cross(R1);

  // positive if >0, negative if <0, positive if ==0.
  const int k = orthantIndex(R2(0) >= 0, R2(1) >= 0, R2(2) >= 0);
  // positive if >0, negative if <0, negative if ==0.
  const int l = orthantIndex(R2(0) > 0, R2(1) > 0, R2(2) > 0);
  DRAKE_DEMAND(k != l);  // Check that the assumption held (numerically).

  // b2(k) + b2(l) >= -1 + b0(i) + b1(j), aka -1 <= b0(i)+b1(j)-b2(k)-b2(l)
  prog->AddLinearConstraint(
      Eigen::RowVector4d(1, 1, -1, -1), -1, 1,
      {b0.segment<1>(i), b1.segment<1>(j), b2.segment<1>(k), b2.segment<1>(l)});
}

void AddRotationMatrixOrthantMilpConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const DecisionVariableMatrix<3, 3>>& R) {
  auto b0 = AddVectorL1NormConstraint(prog, R.col(0), "0");
  auto b1 = AddVectorL1NormConstraint(prog, R.col(1), "1");
  auto b2 = AddVectorL1NormConstraint(prog, R.col(2), "2");

  for (int i = 0; i < 8; i++) {
    bool positive_x = i & (1 << 2), positive_y = i & (1 << 1),
         positive_z = i & (1 << 0);
    // R1 cannot lie in the same orthant as R0 nor -R0.
    // b0(i) + b1(i) + b1(j) <= 1, where j is the index representing -R0.
    prog->AddLinearConstraint(
        Eigen::RowVector3d::Ones(), 0, 1,
        {b0.segment<1>(i), b1.segment<1>(i),
         b1.segment<1>(orthantIndex(!positive_x, !positive_y, !positive_z))});

    // R2 = cross(R0,R1), which can be written as
    //    b0(i) && b1(j) => b2(k) || b2(l), via  b2(k)+b2(l)>=-1+b0(i)+b1(j)
    // where k,l are the two orthants implied by the cross product of vectors in
    // orthants i and j.  We need to write this constraint for every viable
    // i,j combination (e.g. those with 1 or 2 signs flipped).
    AddCrossProductConstraint(
        prog, b0, b1, b2, i, orthantIndex(positive_x, positive_y, !positive_z));
    AddCrossProductConstraint(
        prog, b0, b1, b2, i, orthantIndex(positive_x, !positive_y, positive_z));
    AddCrossProductConstraint(
        prog, b0, b1, b2, i, orthantIndex(!positive_x, positive_y, positive_z));
    AddCrossProductConstraint(
        prog, b0, b1, b2, i,
        orthantIndex(positive_x, !positive_y, !positive_z));
    AddCrossProductConstraint(
        prog, b0, b1, b2, i,
        orthantIndex(!positive_x, positive_y, !positive_z));
    AddCrossProductConstraint(
        prog, b0, b1, b2, i,
        orthantIndex(!positive_x, !positive_y, positive_z));
  }
}

}  // namespace solvers
}  // namespace drake
