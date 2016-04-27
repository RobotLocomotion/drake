//
// Created by Twan Koolen on 4/27/16.
//

#include "drake/systems/plants/joints/Joints.h"
#include <iostream>

using namespace Drake;
using namespace Eigen;
using namespace std;

int main() {

  QuaternionFloating<double> qf;
  VectorXd q = VectorXd::Random(7);
  cout << qf.jointTransform(q).matrix() << endl;

  const JointType<double>& jt = qf;
  cout << jointTransform(jt, q).matrix() << endl;

  auto q2 = ConvertMatrix<float>()(q);
  auto& q3 = ConvertMatrix<double>()(q);

  cout << boolalpha << (&q == &q3) << endl;


  return 0;
};




//  template <typename DerivedQ, typename DerivedV>
//  SpatialVector<Promote<J, typename DerivedQ::Scalar>> motionSubspaceDotTimesV(const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v) {
//    using Q = typename DerivedQ::Scalar;
//    using V = typename DerivedV::Scalar;
//    static_assert(std::is_same<Q, V>::value, "types of Q and V must be the same");
//
//  };

