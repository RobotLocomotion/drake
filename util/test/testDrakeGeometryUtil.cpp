#include <Eigen/Core>
#include "drakeGeometryUtil.h"
#include "drakeQuatUtil.h"
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

int main(int argc, char **argv) {
  Vector4d a = Vector4d::Random();
  a.head<3>().normalize();
  auto q = axis2quat(a);
  auto R = axis2rotmat(a);
  auto rpy = axis2rpy(a);

//  std::cout << "q:" << q << "\n";

  auto a_from_q = quat2axis(q);


  valuecheck(a, a_from_q, 1e-12);



  return 0;
}
