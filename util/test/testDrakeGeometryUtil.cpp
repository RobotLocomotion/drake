#include <Eigen/Core>
#include "drakeGeometryUtil.h"
#include "testUtil.h"
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

void testExpmap2quat(const Vector4d &quat);

void testRotationConversionFunctions()
{
  int ntests = 100;
  default_random_engine generator;
  // quat2axis, axis2quat
  for (int i = 0; i < ntests; i++) {
    Vector4d q = uniformlyRandomQuat(generator);
    auto a = quat2axis(q);
    auto q_back = axis2quat(a);
    valuecheck(acos(abs(q.transpose() * q_back)), 0.0, 1e-6);
  }
  // quat2rotmat, rotmat2quat
  for (int i = 0; i < ntests; i++) {
    Vector4d q = uniformlyRandomQuat(generator);
    Matrix3d R = quat2rotmat(q);
    Vector4d q_back = rotmat2quat(R);
    valuecheck(acos(abs(q.transpose() * q_back)), 0.0, 1e-6);
  }
  // quat2rpy, rpy2quat
  for (int i = 0; i < ntests; i++) {
    Vector4d q = uniformlyRandomQuat(generator);
    Vector3d rpy = quat2rpy(q);
    Vector4d q_back = rpy2quat(rpy);
    valuecheck(acos(abs(q.transpose() * q_back)), 0.0, 1e-6);
  }
  // rotmat2axis, axis2rotmat
  for (int i = 0; i < ntests; i++) {
    Matrix3d R = uniformlyRandomRotmat(generator);
    Vector4d a = rotmat2axis(R);
    Matrix3d R_back = axis2rotmat(a);
    valuecheckMatrix(R, R_back, 1e-6);
  }
  // rotmat2rpy, rpy2rotmat
  for (int i = 0; i < ntests; i++) {
    Matrix3d R = uniformlyRandomRotmat(generator);
    Vector3d rpy = rotmat2rpy(R);
    Matrix3d R_back = rpy2rotmat(rpy);
    valuecheckMatrix(R, R_back, 1e-6);
  }
  // rpy2axis, axis2rpy
  for (int i = 0; i < ntests; i++) {
    Vector3d rpy = uniformlyRandomRPY(generator);
    Vector4d axis = rpy2axis(rpy);
    Vector3d rpy_back = axis2rpy(axis);
    valuecheckMatrix(rpy, rpy_back, 1e-6);
  }
  // expmap2quat, quat2expmap
  Vector4d quat_degenerate = Vector4d::Zero();
  quat_degenerate(0) = 1.0;
  testExpmap2quat(quat_degenerate);
  quat_degenerate(0) = -1.0;
  testExpmap2quat(quat_degenerate);
  for (int i = 0; i<ntests; i++)
  {
    Vector4d quat = uniformlyRandomQuat(generator);
    testExpmap2quat(quat);
  }
  // quat2eigenQuaternion
  Vector4d quat = uniformlyRandomQuat(generator);
  Quaterniond eigenQuat = quat2eigenQuaternion(quat);
  Matrix3d R_expected = quat2rotmat(quat);
  Matrix3d R_eigen = eigenQuat.matrix();
  valuecheckMatrix(R_expected, R_eigen, 1e-6);
}

void testDHomogTrans(int ntests) {
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

void testDHomogTransInv(int ntests, bool check) {
  Isometry3d T;
  std::default_random_engine generator;
  for (int testnr = 0; testnr < ntests; testnr++) {
    Vector4d q = uniformlyRandomQuat(generator);
//    T = Quaterniond(q(0), q(1), q(2), q(3)) * Translation3d(Vector3d::Random());
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

void testDTransformAdjoint(int ntests) {
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

void testDTransformAdjointTranspose(int ntests) {
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

void testNormalizeVec(int ntests) {
  const int x_rows = 4;

  for (int testnr = 0; testnr < ntests; testnr++) {
    auto x = Matrix<double, x_rows, 1>::Random().eval();
    Matrix<double, x_rows, 1> x_norm;
    Matrix<double, x_rows, x_rows> dx_norm;
    Matrix<double, x_rows * x_rows, x_rows> ddx_norm;
    normalizeVec(x, x_norm, &dx_norm, &ddx_norm);
//    std::cout << "gradientNumRows: " << gradientNumRows(x_rows, x_rows, 1) << std::endl;

    volatile auto volx_norm = x_norm;
    volatile auto voldx_norm = dx_norm;
    volatile auto volddx_norm = ddx_norm;

//    std::cout << "x_norm:\n" << x_norm << std::endl << std::endl;
//    std::cout << "dx_norm:\n" << dx_norm << std::endl << std::endl;
//    std::cout << "ddx_norm:\n" << ddx_norm << std::endl << std::endl;
  }
}

void testSpatialCrossProduct()
{
  auto a = (Matrix<double, TWIST_SIZE, 1>::Random()).eval();
  auto b = (Matrix<double, TWIST_SIZE, TWIST_SIZE>::Identity()).eval();
  auto a_crm_b = crossSpatialMotion(a, b);
  auto a_crf_b = crossSpatialForce(a, b);
  valuecheckMatrix(a_crf_b, -a_crm_b.transpose(), 1e-8);
}

void testdrpy2rotmat()
{
  default_random_engine generator;
  Vector3d rpy = uniformlyRandomRPY(generator);
  Matrix3d R = rpy2rotmat(rpy);
  Matrix<double,9,3> dR = drpy2rotmat(rpy);
  Matrix<double,9,3> dR_num = Matrix<double,9,3>::Zero();
  for(int i = 0;i<3;i++)
  {
    Vector3d err = Vector3d::Zero();
    err(i) = 1e-7;
    Vector3d rpyi = rpy+err;
    Matrix3d Ri = rpy2rotmat(rpyi);
    Matrix3d Ri_err = (Ri-R)/err(i);
    for(int j = 0;j<9;j++)
    {
      dR_num(j,i) = Ri_err(j);
      valuecheck(dR(j,i),dR_num(j,i),1e-3);
    }
  }
}

void testExpmap2quat(const Vector4d &quat)
{
  auto expmap = quat2expmap(quat,1);
  auto quat_back  = expmap2quat(expmap.value(),2);
  valuecheck(std::abs(quat.transpose() * quat_back.value()), 1.0, 1e-8);
  Matrix3d expmap_back = expmap.gradient().value()*quat_back.gradient().value();
  Matrix3d identity = Matrix3d::Identity();
  valuecheckMatrix(expmap_back,identity,1E-10);
}

int main(int argc, char **argv)
{
  testRotationConversionFunctions();

  int ntests = 100000;
  std::cout << "testDHomogTrans elapsed time: " << measure<>::execution(testDHomogTrans, ntests) << std::endl;
  std::cout << "testDHomogTransInv elapsed time: " << measure<>::execution(testDHomogTransInv, ntests, false) << std::endl;
  std::cout << "testDTransformAdjoint elapsed time: " << measure<>::execution(testDTransformAdjoint, ntests) << std::endl;
  std::cout << "testDTransformAdjointTranspose elapsed time: " << measure<>::execution(testDTransformAdjointTranspose, ntests) << std::endl;
  std::cout << "testNormalizeVec elapsed time: " << measure<>::execution(testNormalizeVec, ntests) << std::endl;

  testDHomogTransInv(1000, true);
  testSpatialCrossProduct();
	testdrpy2rotmat();

  return 0;
}
