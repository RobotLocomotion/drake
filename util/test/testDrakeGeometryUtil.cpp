#include <Eigen/Core>
#include "drakeGeometryUtil.h"
#include <exception>

template <typename DerivedA, typename DerivedB>
void valuecheck(const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b)
{
  if (!a.isApprox(b)) {
    throw std::runtime_error("wrong");
  }
}

int main(int argc, char **argv) {
  Eigen::Vector4d a = Eigen::Vector4d::Random();
  a.head<3>().normalize();
  auto q = axis2quat(a);
  auto R = axis2rotmat(a);
  auto rpy = axis2rpy(a);

  auto a_from_q = quat2axis(q);

  valuecheck(a, a_from_q);



  return 0;
}
