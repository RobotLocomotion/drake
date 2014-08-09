#include <Eigen/Core>
#include "drakeGeometryUtil.h"
#include <exception>
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

template<typename DerivedA, typename DerivedB>
void valuecheck(const DenseBase<DerivedA>& a, const DenseBase<DerivedB>& b, typename DerivedA::Scalar tolerance = 1e-8)
{
  if (!a.isApprox(b)) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

void valuecheck(double a, double b, double tolerance = 1e-8)
{
  if (std::abs(a - b) > tolerance) {
    std::ostringstream stream;
    stream << "Expected:\n" << a << "\nbut got:" << b << "\n";
    throw std::runtime_error(stream.str());
  }
}

int main(int argc, char **argv)
{
  int ntests = 100;

  std::default_random_engine generator;

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
    valuecheck(R, R_back, 1e-6);
  }

  // rotmat2rpy, rpy2rotmat
  for (int i = 0; i < ntests; i++) {
    Matrix3d R = uniformlyRandomRotmat(generator);
    Vector3d rpy = rotmat2rpy(R);
    Matrix3d R_back = rpy2rotmat(rpy);
    valuecheck(R, R_back, 1e-6);
  }

  // rpy2axis, axis2rpy
  for (int i = 0; i < ntests; i++) {
    Vector3d rpy = uniformlyRandomRPY(generator);
    Vector4d axis = rpy2axis(rpy);
    Vector3d rpy_back = axis2rpy(axis);
    valuecheck(rpy, rpy_back, 1e-6);
  }

  return 0;
}
