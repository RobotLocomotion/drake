#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/random_rotation.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/util/drakeGeometryUtil.h"

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

namespace drake {
namespace util {
namespace {

GTEST_TEST(DrakeGeometryUtilTest, DHomogTrans) {
  const int ntests = 1;
  Isometry3d T;
  std::default_random_engine generator;

  for (int testnr = 0; testnr < ntests; testnr++) {
    Vector4d q = drake::math::UniformlyRandomQuat(generator);
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
    Vector4d q = drake::math::UniformlyRandomQuat(generator);
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
    Vector4d q = drake::math::UniformlyRandomQuat(generator);
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
    Vector4d q = drake::math::UniformlyRandomQuat(generator);
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

GTEST_TEST(DrakeGeometryUtilTest, SpatialCrossProduct) {
  auto a = (drake::TwistVector<double>::Random()).eval();
  auto b = (drake::SquareTwistMatrix<double>::Identity()).eval();
  auto a_crm_b = crossSpatialMotion(a, b);
  auto a_crf_b = crossSpatialForce(a, b);
  EXPECT_TRUE(CompareMatrices(a_crf_b, -a_crm_b.transpose(), 1e-8,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace util
}  // namespace drake
