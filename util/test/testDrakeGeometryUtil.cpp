#include <Eigen/Core>
#include "drakeGeometryUtil.h"
#include <exception>
#include <iostream>

using namespace Eigen;
using namespace std;

template <typename DerivedA, typename DerivedB>
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

int main(int argc, char **argv) {
  int ntests = 100;
  for (int i = 0; i < ntests; i++) {
    Vector4d a = Vector4d::Random();
    a.head<3>().normalize();
    auto q = axis2quat(a);
    auto a_back = quat2axis(q);
    valuecheck(0.0, quatNorm(quatDiff(a, a_back)), 1e-12);
  }

//  auto R_from_a = axis2rotmat(a);
//  auto rpy_from_a = axis2rpy(a);

//  std::cout << "q:" << q << "\n";



  return 0;
}
