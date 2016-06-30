#include <cmath>
#include <iostream>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"
#include "drake/core/Gradient.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"
#include "gtest/gtest.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Dynamic;
using Eigen::Quaterniond;
using Eigen::Translation3d;
using std::default_random_engine;
using drake::util::MatrixCompareType;

namespace drake {
namespace util {
namespace {

GTEST_TEST(DrakeGeometryUtilTest, RotationConversionFunctions) {
  int ntests = 1;
  default_random_engine generator;
  // quat2axis, axis2quat
  for (int i = 0; i < ntests; i++) {
    Vector4d q = uniformlyRandomQuat(generator);
    auto a = quat2axis(q);
    auto q_back = axis2quat(a);
    valuecheck(acos(std::abs(q.transpose() * q_back)), 0.0, 1e-6);
  }
  // quat2rotmat, rotmat2quat
  for (int i = 0; i < ntests; i++) {
    Vector4d q = uniformlyRandomQuat(generator);
    Matrix3d R = quat2rotmat(q);
    Vector4d q_back = rotmat2quat(R);
    valuecheck(acos(std::abs(q.transpose() * q_back)), 0.0, 1e-6);
  }
  // quat2rpy, rpy2quat
  for (int i = 0; i < ntests; i++) {
    Vector4d q = uniformlyRandomQuat(generator);
    Vector3d rpy = quat2rpy(q);
    Vector4d q_back = rpy2quat(rpy);
    valuecheck(acos(std::abs(q.transpose() * q_back)), 0.0, 1e-6);
  }
  // rotmat2axis, axis2rotmat
  for (int i = 0; i < ntests; i++) {
    Matrix3d R = uniformlyRandomRotmat(generator);
    Vector4d a = rotmat2axis(R);
    Matrix3d R_back = axis2rotmat(a);
    EXPECT_TRUE(CompareMatrices(R, R_back, 1e-6, MatrixCompareType::absolute));
  }
  // rotmat2rpy, rpy2rotmat
  for (int i = 0; i < ntests; i++) {
    Matrix3d R = uniformlyRandomRotmat(generator);
    Vector3d rpy = rotmat2rpy(R);
    Matrix3d R_back = rpy2rotmat(rpy);
    EXPECT_TRUE(CompareMatrices(R, R_back, 1e-6, MatrixCompareType::absolute));
  }
  // rpy2axis, axis2rpy
  for (int i = 0; i < ntests; i++) {
    Vector3d rpy = uniformlyRandomRPY(generator);
    Vector4d axis = rpy2axis(rpy);
    Vector3d rpy_back = axis2rpy(axis);
    EXPECT_TRUE(
        CompareMatrices(rpy, rpy_back, 1e-6, MatrixCompareType::absolute));
  }
  // quat2eigenQuaternion
  Vector4d quat = uniformlyRandomQuat(generator);
  Quaterniond eigenQuat = quat2eigenQuaternion(quat);
  Matrix3d R_expected = quat2rotmat(quat);
  Matrix3d R_eigen = eigenQuat.matrix();
  EXPECT_TRUE(
      CompareMatrices(R_expected, R_eigen, 1e-6, MatrixCompareType::absolute));
}

GTEST_TEST(DrakeGeometryUtilTest, DHomogTrans) {
  const int ntests = 1;
  Isometry3d T;
  std::default_random_engine generator;

  for (int testnr = 0; testnr < ntests; testnr++) {
    Vector4d q = uniformlyRandomQuat(generator);
    T = Quaterniond(q(0), q(1), q(2), q(3));
    //  T.setIdentity();
    //  T = AngleAxisd(M_PI_2, Vector3d(1.0, 0.0, 0.0));

    const int nv = 6;
    const int nq = 7;

    auto S = Matrix<double, 6, Dynamic>::Random(6, nv).eval();
    //    setLinearIndices(S);
    //    S.setIdentity();
    //    std::cout << S << "\n\n";

    auto qdot_to_v = MatrixXd::Random(nv, nq).eval();
    //    setLinearIndices(qdot_to_v);
    //    std::cout << qdot_to_v << "\n\n";

    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    volatile auto vol = dT;
    //  std::cout << dT << std::endl << std::endl;
  }
}

GTEST_TEST(DrakeGeometryUtilTest, DHomogTransInv) {
  const int ntests = 1;
  const bool check = true;
  Isometry3d T;
  std::default_random_engine generator;
  for (int testnr = 0; testnr < ntests; testnr++) {
    Vector4d q = uniformlyRandomQuat(generator);
    //    T = Quaterniond(q(0), q(1), q(2), q(3)) *
    //    Translation3d(Vector3d::Random());
    T = Quaterniond(q(0), q(1), q(2), q(3));

    const int nv = 6;
    const int nq = 7;

    auto S = Matrix<double, 6, Dynamic>::Random(6, nv).eval();
    auto qdot_to_v = MatrixXd::Random(nv, nq).eval();

    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    auto dTInv = dHomogTransInv(T, dT);
    volatile auto vol = dTInv;

    if (check) {
      auto dTInvInv = dHomogTransInv(T.inverse(), dTInv);

      if (!dT.matrix().isApprox(dTInvInv.matrix(), 1e-10)) {
        std::cout << "dTInv:\n" << dTInv << "\n\n";
        std::cout << "dT:\n" << dT << "\n\n";
        std::cout << "dTInvInv:\n" << dTInvInv << "\n\n";
        std::cout << "dTInvInv - dT:\n" << dTInvInv - dT << "\n\n";

        throw std::runtime_error("wrong");
      }
    }
  }
}

GTEST_TEST(DrakeGeometryUtilTest, DTransformAdjoint) {
  const int ntests = 1;
  const int nv = 6;
  const int nq = 34;
  const int cols_X = 3;

  Isometry3d T;
  std::default_random_engine generator;

  for (int testnr = 0; testnr < ntests; testnr++) {
    Vector4d q = uniformlyRandomQuat(generator);
    T = Quaterniond(q(0), q(1), q(2), q(3)) * Translation3d(Vector3d::Random());
    auto S = Matrix<double, 6, Dynamic>::Random(6, nv).eval();
    auto qdot_to_v = MatrixXd::Random(nv, nq).eval();
    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    auto X = Matrix<double, 6, Dynamic>::Random(6, cols_X).eval();
    auto dX = MatrixXd::Random(X.size(), nq).eval();
    //    auto dX = Matrix<double, X.SizeAtCompileTime, nq>::Random().eval();
    auto dAdT_times_X = dTransformSpatialMotion(T, X, dT, dX).eval();
    volatile auto vol = dAdT_times_X;
  }
}

GTEST_TEST(DrakeGeometryUtilTest, DTransformAdjointTranspose) {
  const int ntests = 1;
  const int nv = 6;
  const int nq = 34;
  const int cols_X = 3;

  Isometry3d T;
  std::default_random_engine generator;

  for (int testnr = 0; testnr < ntests; testnr++) {
    Vector4d q = uniformlyRandomQuat(generator);
    T = Quaterniond(q(0), q(1), q(2), q(3)) * Translation3d(Vector3d::Random());
    auto S = Matrix<double, 6, Dynamic>::Random(6, nv).eval();
    auto qdot_to_v = MatrixXd::Random(nv, nq).eval();
    auto dT = dHomogTrans(T, S, qdot_to_v).eval();
    auto X = Matrix<double, 6, Dynamic>::Random(6, cols_X).eval();
    auto dX = MatrixXd::Random(X.size(), nq).eval();
    //    auto dX = Matrix<double, X.SizeAtCompileTime, nq>::Random().eval();
    auto dAdTtranspose_times_X = dTransformSpatialForce(T, X, dT, dX).eval();
    volatile auto vol = dAdTtranspose_times_X;
  }
}

GTEST_TEST(DrakeGeometryUtilTest, NormalizeVec) {
  const int ntests = 1;
  const int x_rows = 4;

  for (int testnr = 0; testnr < ntests; testnr++) {
    auto x = Matrix<double, x_rows, 1>::Random().eval();
    Matrix<double, x_rows, 1> x_norm;
    Matrix<double, x_rows, x_rows> dx_norm;
    Matrix<double, x_rows * x_rows, x_rows> ddx_norm;
    normalizeVec(x, x_norm, &dx_norm, &ddx_norm);
  }
}

GTEST_TEST(DrakeGeometryUtilTest, SpatialCrossProduct) {
  auto a = (drake::TwistVector<double>::Random()).eval();
  auto b = (drake::SquareTwistMatrix<double>::Identity()).eval();
  auto a_crm_b = crossSpatialMotion(a, b);
  auto a_crf_b = crossSpatialForce(a, b);
  EXPECT_TRUE(CompareMatrices(a_crf_b, -a_crm_b.transpose(), 1e-8,
                              MatrixCompareType::absolute));
}

GTEST_TEST(DrakeGeometryUtilTest, drpy2rotmat) {
  default_random_engine generator;
  Vector3d rpy = uniformlyRandomRPY(generator);
  Matrix3d R = rpy2rotmat(rpy);
  Matrix<double, 9, 3> dR = drpy2rotmat(rpy);
  Matrix<double, 9, 3> dR_num = Matrix<double, 9, 3>::Zero();
  for (int i = 0; i < 3; i++) {
    Vector3d err = Vector3d::Zero();
    err(i) = 1e-7;
    Vector3d rpyi = rpy + err;
    Matrix3d Ri = rpy2rotmat(rpyi);
    Matrix3d Ri_err = (Ri - R) / err(i);
    for (int j = 0; j < 9; j++) {
      dR_num(j, i) = Ri_err(j);
      valuecheck(dR(j, i), dR_num(j, i), 1e-3);
    }
  }
}

}  // namespace
}  // namespace util
}  // namespace drake
